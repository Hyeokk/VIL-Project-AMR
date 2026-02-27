// terrain_costmap_node.cpp
// Generates a robot-centric costmap from GroundGrid output.
//
// Inputs:
//   /groundgrid/grid_map      (grid_map_msgs/GridMap)  — elevation, variance, confidence
//   /groundgrid/filtered_cloud (PointCloud2)            — ground(49) + obstacle(99) labeled
//   TF: odom → base_link chain
//
// Outputs:
//   /terrain_costmap           (nav_msgs/OccupancyGrid) — traversability costmap
//   /terrain_costmap/terrain_cloud (PointCloud2)        — accumulated terrain for debug
//
// Cost factors:
//   1. Obstacle density   — non-ground points in each cell
//   2. Slope              — elevation gradient between neighbor cells
//   3. Roughness          — height variance within cell
//   4. Uncertainty        — low groundpatch confidence = unknown area
//
// [v2] Temporal smoothing (IIR filter) — reduces costmap flickering
// [v3] Costmap-level inflation — exponential cost decay around obstacles
//      Both A* and DWA benefit from the same inflated cost field.
//      Path planner no longer needs its own inflation step.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>
#include <mutex>
#include <cmath>
#include <algorithm>

// Per-cell statistics from accumulated terrain points
struct TerrainCellStats {
    int count = 0;
    double sum_z = 0.0;
    double sum_z2 = 0.0;

    double mean_z() const { return count > 0 ? sum_z / count : 0.0; }
    double variance() const {
        if (count < 2) return 0.0;
        double m = mean_z();
        return std::max(0.0, sum_z2 / count - m * m);
    }
};

class TerrainCostmapNode : public rclcpp::Node
{
public:
    TerrainCostmapNode() : Node("terrain_costmap_node")
    {
        declare_parameters();
        load_parameters();
        setup_tf();
        setup_subscribers();
        setup_publishers();

        // Pre-allocate terrain accumulation buffer
        terrain_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
        terrain_cloud_->reserve(terrain_max_points_);

        RCLCPP_INFO(get_logger(),
            "TerrainCostmap: %.1fm x %.1fm, resolution=%.2fm, cells=%dx%d",
            costmap_size_, costmap_size_, costmap_resolution_,
            costmap_cells_, costmap_cells_);
        RCLCPP_INFO(get_logger(),
            "Weights: obstacle=%.2f, slope=%.2f, roughness=%.2f, unknown=%.2f",
            w_obstacle_, w_slope_, w_roughness_, w_unknown_);
        RCLCPP_INFO(get_logger(),
            "[v2] Temporal smoothing: alpha=%.2f, obstacle_persist=%d frames",
            temporal_alpha_, temporal_obstacle_persist_);
        RCLCPP_INFO(get_logger(),
            "[v3] Costmap inflation: radius=%.2fm (%d cells), decay_rate=%.1f",
            inflation_radius_,
            static_cast<int>(std::ceil(inflation_radius_ / costmap_resolution_)),
            inflation_decay_rate_);
    }

private:
    // =========================================================================
    // Parameters
    // =========================================================================
    void declare_parameters()
    {
        // Costmap geometry
        this->declare_parameter<double>("costmap_size", 3.0);
        this->declare_parameter<double>("costmap_resolution", 0.1);
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("odom_frame", "odom");

        // Cost weights
        this->declare_parameter<double>("weight_obstacle", 0.4);
        this->declare_parameter<double>("weight_slope", 0.3);
        this->declare_parameter<double>("weight_roughness", 0.15);
        this->declare_parameter<double>("weight_unknown", 0.15);

        // Thresholds
        this->declare_parameter<double>("slope_max_deg", 35.0);
        this->declare_parameter<double>("roughness_max", 0.05);
        this->declare_parameter<double>("obstacle_count_max", 5.0);
        this->declare_parameter<double>("confidence_threshold", 0.1);
        this->declare_parameter<double>("lethal_threshold", 0.8);

        // Terrain accumulation
        this->declare_parameter<bool>("enable_terrain_accumulation", true);
        this->declare_parameter<int>("terrain_max_points", 200000);
        this->declare_parameter<double>("terrain_radius", 10.0);
        this->declare_parameter<double>("terrain_publish_rate", 1.0);

        // [v2] Temporal smoothing
        this->declare_parameter<double>("temporal_alpha", 0.3);
        this->declare_parameter<int>("temporal_obstacle_persist", 3);

        // [v3] Costmap inflation
        // Exponential cost decay around lethal cells
        // Higher inflation_radius = A* and DWA keep more distance from walls
        // Higher decay_rate = cost drops faster with distance (narrower buffer)
        this->declare_parameter<double>("inflation_radius", 0.8);     // [m]
        this->declare_parameter<double>("inflation_decay_rate", 2.5); // exponential steepness
        this->declare_parameter<int>("inflation_lethal_cost", 90);    // cells >= this get inflated

        // Topics
        this->declare_parameter<std::string>("gridmap_topic", "/groundgrid/grid_map");
        this->declare_parameter<std::string>("filtered_cloud_topic", "/groundgrid/filtered_cloud");
        this->declare_parameter<std::string>("costmap_topic", "/terrain_costmap");
        this->declare_parameter<std::string>("terrain_cloud_topic", "/terrain_costmap/terrain_cloud");
    }

    void load_parameters()
    {
        costmap_size_ = get_parameter("costmap_size").as_double();
        costmap_resolution_ = get_parameter("costmap_resolution").as_double();
        robot_frame_ = get_parameter("robot_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();

        w_obstacle_ = get_parameter("weight_obstacle").as_double();
        w_slope_ = get_parameter("weight_slope").as_double();
        w_roughness_ = get_parameter("weight_roughness").as_double();
        w_unknown_ = get_parameter("weight_unknown").as_double();

        slope_max_rad_ = get_parameter("slope_max_deg").as_double() * M_PI / 180.0;
        roughness_max_ = get_parameter("roughness_max").as_double();
        obstacle_count_max_ = get_parameter("obstacle_count_max").as_double();
        confidence_threshold_ = get_parameter("confidence_threshold").as_double();
        lethal_threshold_ = get_parameter("lethal_threshold").as_double();

        enable_terrain_accum_ = get_parameter("enable_terrain_accumulation").as_bool();
        terrain_max_points_ = get_parameter("terrain_max_points").as_int();
        terrain_radius_ = get_parameter("terrain_radius").as_double();

        // [v2] Temporal smoothing
        temporal_alpha_ = get_parameter("temporal_alpha").as_double();
        temporal_obstacle_persist_ = get_parameter("temporal_obstacle_persist").as_int();

        // [v3] Inflation
        inflation_radius_ = get_parameter("inflation_radius").as_double();
        inflation_decay_rate_ = get_parameter("inflation_decay_rate").as_double();
        inflation_lethal_cost_ = get_parameter("inflation_lethal_cost").as_int();

        costmap_cells_ = static_cast<int>(std::round(costmap_size_ / costmap_resolution_));

        gridmap_topic_ = get_parameter("gridmap_topic").as_string();
        filtered_cloud_topic_ = get_parameter("filtered_cloud_topic").as_string();
        costmap_topic_ = get_parameter("costmap_topic").as_string();
        terrain_cloud_topic_ = get_parameter("terrain_cloud_topic").as_string();
    }

    // =========================================================================
    // Setup
    // =========================================================================
    void setup_tf()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void setup_subscribers()
    {
        sub_gridmap_ = create_subscription<grid_map_msgs::msg::GridMap>(
            gridmap_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TerrainCostmapNode::gridmap_callback, this, std::placeholders::_1));

        if (enable_terrain_accum_) {
            sub_filtered_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                filtered_cloud_topic_, rclcpp::SensorDataQoS(),
                std::bind(&TerrainCostmapNode::filtered_cloud_callback, this, std::placeholders::_1));
        }
    }

    void setup_publishers()
    {
        pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, rclcpp::SystemDefaultsQoS());

        if (enable_terrain_accum_) {
            pub_terrain_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                terrain_cloud_topic_, rclcpp::SystemDefaultsQoS());

            double rate = get_parameter("terrain_publish_rate").as_double();
            if (rate > 0.0) {
                terrain_timer_ = create_wall_timer(
                    std::chrono::duration<double>(1.0 / rate),
                    std::bind(&TerrainCostmapNode::publish_terrain_cloud, this));
            }
        }
    }

    // =========================================================================
    // GridMap callback — Main costmap generation pipeline
    // =========================================================================
    void gridmap_callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
    {
        // 1. Convert message to grid_map
        grid_map::GridMap full_map;
        if (!grid_map::GridMapRosConverter::fromMessage(*msg, full_map)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Failed to convert GridMap message");
            return;
        }

        // 2. Get robot position in odom frame via TF
        geometry_msgs::msg::TransformStamped tf_odom_to_robot;
        try {
            tf_odom_to_robot = tf_buffer_->lookupTransform(
                odom_frame_, robot_frame_,
                msg->header.stamp,
                tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "TF %s → %s failed: %s", odom_frame_.c_str(), robot_frame_.c_str(), ex.what());
            return;
        }

        const double robot_x = tf_odom_to_robot.transform.translation.x;
        const double robot_y = tf_odom_to_robot.transform.translation.y;

        // 3. Check if robot position is inside the GridMap
        grid_map::Position robot_pos(robot_x, robot_y);
        if (!full_map.isInside(robot_pos)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Robot position (%.2f, %.2f) is outside GridMap", robot_x, robot_y);
            return;
        }

        // 4. Cache yaw for coordinate transforms
        tf2::Quaternion q(
            tf_odom_to_robot.transform.rotation.x,
            tf_odom_to_robot.transform.rotation.y,
            tf_odom_to_robot.transform.rotation.z,
            tf_odom_to_robot.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        cos_yaw_ = std::cos(yaw);
        sin_yaw_ = std::sin(yaw);

        // 5. Extract submap around robot
        bool success = false;
        grid_map::GridMap submap = full_map.getSubmap(
            robot_pos,
            grid_map::Length(costmap_size_, costmap_size_),
            success);

        if (!success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Failed to extract submap at (%.2f, %.2f)", robot_x, robot_y);
            return;
        }

        // 6. Generate costmap from submap layers
        auto costmap_msg = generate_costmap(submap, tf_odom_to_robot);
        pub_costmap_->publish(*costmap_msg);
    }

    // =========================================================================
    // Costmap generation from GridMap submap
    // =========================================================================
    nav_msgs::msg::OccupancyGrid::UniquePtr generate_costmap(
        const grid_map::GridMap &submap,
        const geometry_msgs::msg::TransformStamped &tf_odom_to_robot)
    {
        auto costmap = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        // Header
        costmap->header.stamp = tf_odom_to_robot.header.stamp;
        costmap->header.frame_id = robot_frame_;

        // Grid metadata
        costmap->info.resolution = costmap_resolution_;
        costmap->info.width = costmap_cells_;
        costmap->info.height = costmap_cells_;
        costmap->info.origin.position.x = -costmap_size_ / 2.0;
        costmap->info.origin.position.y = -costmap_size_ / 2.0;
        costmap->info.origin.position.z = 0.0;
        costmap->info.origin.orientation.w = 1.0;

        costmap->data.resize(costmap_cells_ * costmap_cells_, -1);

        // Build terrain accumulation grid for fallback on unknown cells
        std::vector<TerrainCellStats> terrain_grid;
        if (enable_terrain_accum_ && terrain_cloud_ && !terrain_cloud_->empty()) {
            terrain_grid = build_terrain_grid(tf_odom_to_robot);
        }

        // Get submap layers
        const bool has_ground = submap.exists("ground");
        const bool has_points = submap.exists("points");
        const bool has_variance = submap.exists("variance");
        const bool has_confidence = submap.exists("groundpatch");

        if (!has_ground) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "GridMap missing 'ground' layer — cannot compute slope");
        }

        const grid_map::Position submap_center = submap.getPosition();
        const float submap_res = submap.getResolution();

        // ── Phase 1: Compute raw costs per cell ──
        for (int cy = 0; cy < costmap_cells_; ++cy) {
            for (int cx = 0; cx < costmap_cells_; ++cx) {
                const double rx = costmap->info.origin.position.x
                                + (cx + 0.5) * costmap_resolution_;
                const double ry = costmap->info.origin.position.y
                                + (cy + 0.5) * costmap_resolution_;

                const double ox = rx * cos_yaw_ - ry * sin_yaw_
                                + tf_odom_to_robot.transform.translation.x;
                const double oy = rx * sin_yaw_ + ry * cos_yaw_
                                + tf_odom_to_robot.transform.translation.y;

                grid_map::Position query_pos(ox, oy);
                if (!submap.isInside(query_pos))
                    continue;

                double obstacle_cost = 0.0;
                double slope_cost = 0.0;
                double roughness_cost = 0.0;
                double unknown_cost = 0.0;

                grid_map::Index idx;
                submap.getIndex(query_pos, idx);

                if (has_points) {
                    float pt_count = submap.at("points", idx);
                    obstacle_cost = std::min(
                        static_cast<double>(pt_count) / obstacle_count_max_, 1.0);
                }

                if (has_ground) {
                    slope_cost = compute_slope(submap, idx);
                }

                if (has_variance) {
                    float var = submap.at("variance", idx);
                    roughness_cost = std::min(
                        static_cast<double>(var) / roughness_max_, 1.0);
                }

                if (has_confidence) {
                    float conf = submap.at("groundpatch", idx);
                    if (conf < confidence_threshold_) {
                        unknown_cost = 1.0;
                    }
                }

                double total = w_obstacle_ * obstacle_cost
                             + w_slope_ * slope_cost
                             + w_roughness_ * roughness_cost
                             + w_unknown_ * unknown_cost;

                total = std::clamp(total, 0.0, 1.0);

                int8_t cell_value;
                if (obstacle_cost >= 0.99) {
                    cell_value = 100;
                } else if (total >= lethal_threshold_) {
                    cell_value = 100;
                } else {
                    cell_value = static_cast<int8_t>(total * 99.0);
                }

                costmap->data[cy * costmap_cells_ + cx] = cell_value;
            }
        }

        // ── Phase 2: Fill unknown cells from accumulated terrain ──
        if (enable_terrain_accum_ && !terrain_grid.empty()) {
            int filled = 0;
            for (int cy = 0; cy < costmap_cells_; ++cy) {
                for (int cx = 0; cx < costmap_cells_; ++cx) {
                    const int idx = cy * costmap_cells_ + cx;
                    if (costmap->data[idx] != -1) continue;

                    const auto &tcell = terrain_grid[idx];
                    if (tcell.count < 2) continue;

                    double roughness_cost = std::min(
                        tcell.variance() / roughness_max_, 1.0);
                    double slope_cost = compute_terrain_slope(
                        terrain_grid, cx, cy);

                    double total = w_slope_ * slope_cost
                                 + w_roughness_ * roughness_cost;
                    total = std::clamp(total, 0.0, 1.0);

                    if (total >= lethal_threshold_) {
                        costmap->data[idx] = 100;
                    } else {
                        costmap->data[idx] = static_cast<int8_t>(total * 99.0);
                    }
                    ++filled;
                }
            }
            if (filled > 0) {
                RCLCPP_DEBUG(get_logger(),
                    "Terrain fallback filled %d unknown cells", filled);
            }
        }

        // ── Phase 3: [v3] Inflate obstacles with exponential cost decay ──
        inflate_obstacles(costmap);

        // ── Phase 4: [v2] Temporal smoothing (IIR filter) ──
        apply_temporal_smoothing(costmap);

        return costmap;
    }

    // =========================================================================
    // [v3] Obstacle inflation — exponential cost decay
    //
    // For each lethal cell (>=90), add decaying cost to surrounding cells:
    //   cost = (lethal-1) * exp(-decay_rate * dist / radius)
    //
    // Properties:
    //   - Original lethal cells stay lethal
    //   - Inflated cells are NEVER lethal (always <= lethal-1)
    //   - Cost drops exponentially with distance
    //   - Both A* and DWA see the same inflated field
    // =========================================================================
    void inflate_obstacles(nav_msgs::msg::OccupancyGrid::UniquePtr &costmap)
    {
        const int rows = costmap_cells_;
        const int cols = costmap_cells_;
        const int lethal = inflation_lethal_cost_;
        const int inflate_r = static_cast<int>(
            std::ceil(inflation_radius_ / costmap_resolution_));

        // Work on a copy so we don't cascade inflation
        std::vector<int8_t> original(costmap->data.begin(), costmap->data.end());

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (original[r * cols + c] < lethal) continue;

                // This cell is lethal — inflate surrounding cells
                for (int dr = -inflate_r; dr <= inflate_r; ++dr) {
                    for (int dc = -inflate_r; dc <= inflate_r; ++dc) {
                        if (dr == 0 && dc == 0) continue;
                        int nr = r + dr, nc = c + dc;
                        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

                        double dist_cells = std::hypot(dr, dc);
                        if (dist_cells > inflate_r) continue;

                        // Exponential decay
                        double ratio = dist_cells / static_cast<double>(inflate_r);
                        int decay_cost = static_cast<int>(
                            (lethal - 1) * std::exp(-inflation_decay_rate_ * ratio));
                        decay_cost = std::clamp(decay_cost, 1, lethal - 1);

                        int idx = nr * cols + nc;
                        // Don't overwrite lethal cells or higher costs
                        if (original[idx] >= lethal) continue;
                        int8_t current = costmap->data[idx];
                        if (current == -1 || decay_cost > current) {
                            costmap->data[idx] = static_cast<int8_t>(decay_cost);
                        }
                    }
                }
            }
        }
    }

    // =========================================================================
    // [v2] Temporal smoothing (IIR filter)
    // =========================================================================
    void apply_temporal_smoothing(nav_msgs::msg::OccupancyGrid::UniquePtr &costmap)
    {
        const int total_cells = costmap_cells_ * costmap_cells_;

        // Initialize on first frame
        if (prev_costmap_.empty()) {
            prev_costmap_.resize(total_cells, -1.0);
            obstacle_age_.resize(total_cells, 0);
        }

        for (int i = 0; i < total_cells; ++i) {
            int8_t cur = costmap->data[i];

            if (cur == 100) {
                // Lethal: immediate, no smoothing
                prev_costmap_[i] = 100.0;
                obstacle_age_[i] = temporal_obstacle_persist_;
            } else if (cur == -1) {
                if (obstacle_age_[i] > 0) {
                    --obstacle_age_[i];
                    costmap->data[i] = static_cast<int8_t>(
                        std::clamp(prev_costmap_[i], 0.0, 100.0));
                } else if (prev_costmap_[i] >= 0.0) {
                    prev_costmap_[i] *= (1.0 - temporal_alpha_);
                    costmap->data[i] = static_cast<int8_t>(
                        std::clamp(prev_costmap_[i], 0.0, 99.0));
                }
            } else {
                // Normal cell: IIR blend
                double measured = static_cast<double>(cur);
                if (prev_costmap_[i] < 0.0) {
                    prev_costmap_[i] = measured;
                } else {
                    prev_costmap_[i] = temporal_alpha_ * measured
                                     + (1.0 - temporal_alpha_) * prev_costmap_[i];
                }
                obstacle_age_[i] = (prev_costmap_[i] > 50.0)
                    ? temporal_obstacle_persist_ : 0;
                costmap->data[i] = static_cast<int8_t>(
                    std::clamp(prev_costmap_[i], 0.0, 99.0));
            }
        }
    }

    // =========================================================================
    // Slope computation
    // =========================================================================
    double compute_slope(const grid_map::GridMap &map, const grid_map::Index &idx) const
    {
        const auto &size = map.getSize();
        const int i = idx(0);
        const int j = idx(1);

        if (i < 1 || j < 1 || i >= size(0) - 1 || j >= size(1) - 1)
            return 0.0;

        const float res = map.getResolution();
        const auto &ground = map["ground"];

        const float dz_dx = (ground(i + 1, j) - ground(i - 1, j)) / (2.0f * res);
        const float dz_dy = (ground(i, j + 1) - ground(i, j - 1)) / (2.0f * res);
        const float slope_rad = std::atan(std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy));

        return std::min(static_cast<double>(slope_rad) / slope_max_rad_, 1.0);
    }

    // =========================================================================
    // Terrain accumulation
    // =========================================================================
    std::vector<TerrainCellStats> build_terrain_grid(
        const geometry_msgs::msg::TransformStamped &tf_odom_to_robot)
    {
        std::vector<TerrainCellStats> grid(costmap_cells_ * costmap_cells_);

        std::lock_guard<std::mutex> lock(terrain_mutex_);
        if (!terrain_cloud_ || terrain_cloud_->empty())
            return grid;

        const double tx = tf_odom_to_robot.transform.translation.x;
        const double ty = tf_odom_to_robot.transform.translation.y;
        const double half = costmap_size_ / 2.0;

        for (const auto &pt : terrain_cloud_->points) {
            const double dx = pt.x - tx;
            const double dy = pt.y - ty;
            const double rx =  cos_yaw_ * dx + sin_yaw_ * dy;
            const double ry = -sin_yaw_ * dx + cos_yaw_ * dy;

            if (rx < -half || rx >= half || ry < -half || ry >= half)
                continue;

            const int cx = static_cast<int>((rx + half) / costmap_resolution_);
            const int cy = static_cast<int>((ry + half) / costmap_resolution_);

            if (cx < 0 || cx >= costmap_cells_ || cy < 0 || cy >= costmap_cells_)
                continue;

            auto &cell = grid[cy * costmap_cells_ + cx];
            cell.count++;
            cell.sum_z += pt.z;
            cell.sum_z2 += pt.z * pt.z;
        }

        return grid;
    }

    double compute_terrain_slope(
        const std::vector<TerrainCellStats> &tgrid, int cx, int cy) const
    {
        if (cx < 1 || cy < 1 || cx >= costmap_cells_ - 1 || cy >= costmap_cells_ - 1)
            return 0.0;

        const auto &left  = tgrid[cy * costmap_cells_ + (cx - 1)];
        const auto &right = tgrid[cy * costmap_cells_ + (cx + 1)];
        const auto &down  = tgrid[(cy - 1) * costmap_cells_ + cx];
        const auto &up    = tgrid[(cy + 1) * costmap_cells_ + cx];

        if (left.count == 0 || right.count == 0 ||
            down.count == 0 || up.count == 0)
            return 0.0;

        const double dz_dx = (right.mean_z() - left.mean_z())
                            / (2.0 * costmap_resolution_);
        const double dz_dy = (up.mean_z() - down.mean_z())
                            / (2.0 * costmap_resolution_);
        const double slope_rad = std::atan(
            std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy));

        return std::min(slope_rad / slope_max_rad_, 1.0);
    }

    // =========================================================================
    // Filtered cloud callback — Persistent terrain accumulation
    // =========================================================================
    void filtered_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        if (!enable_terrain_accum_)
            return;

        geometry_msgs::msg::TransformStamped tf_to_odom;
        try {
            tf_to_odom = tf_buffer_->lookupTransform(
                odom_frame_, msg->header.frame_id,
                msg->header.stamp,
                tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Terrain accum TF failed: %s", ex.what());
            return;
        }

        const auto &fields = msg->fields;
        int intensity_offset = -1;
        for (const auto &f : fields) {
            if (f.name == "intensity") {
                intensity_offset = f.offset;
                break;
            }
        }
        if (intensity_offset < 0) return;

        const double tx = tf_to_odom.transform.translation.x;
        const double ty = tf_to_odom.transform.translation.y;
        const double tz = tf_to_odom.transform.translation.z;

        tf2::Quaternion q(
            tf_to_odom.transform.rotation.x,
            tf_to_odom.transform.rotation.y,
            tf_to_odom.transform.rotation.z,
            tf_to_odom.transform.rotation.w);
        tf2::Matrix3x3 rot(q);

        pcl::PointCloud<pcl::PointXYZI> new_ground;
        const uint8_t *data = msg->data.data();
        const uint32_t step = msg->point_step;
        const uint32_t x_off = msg->fields[0].offset;
        const uint32_t y_off = msg->fields[1].offset;
        const uint32_t z_off = msg->fields[2].offset;

        for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t *ptr = data + i * step;
            float intensity = *reinterpret_cast<const float*>(ptr + intensity_offset);

            if (intensity < 40.0f || intensity > 60.0f)
                continue;

            float px = *reinterpret_cast<const float*>(ptr + x_off);
            float py = *reinterpret_cast<const float*>(ptr + y_off);
            float pz = *reinterpret_cast<const float*>(ptr + z_off);

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
                continue;

            tf2::Vector3 p_body(px, py, pz);
            tf2::Vector3 p_odom = rot * p_body + tf2::Vector3(tx, ty, tz);

            pcl::PointXYZI pt;
            pt.x = p_odom.x();
            pt.y = p_odom.y();
            pt.z = p_odom.z();
            pt.intensity = intensity;
            new_ground.push_back(pt);
        }

        if (new_ground.empty()) return;

        std::lock_guard<std::mutex> lock(terrain_mutex_);
        *terrain_cloud_ += new_ground;

        const double radius_sq = terrain_radius_ * terrain_radius_;
        const double robot_x = tf_to_odom.transform.translation.x;
        const double robot_y = tf_to_odom.transform.translation.y;

        pcl::PointCloud<pcl::PointXYZI> pruned;
        pruned.reserve(terrain_cloud_->size());
        for (const auto &pt : terrain_cloud_->points) {
            double dx = pt.x - robot_x;
            double dy = pt.y - robot_y;
            if (dx * dx + dy * dy <= radius_sq)
                pruned.push_back(pt);
        }

        if (static_cast<int>(pruned.size()) > terrain_max_points_) {
            int excess = pruned.size() - terrain_max_points_;
            pruned.points.erase(pruned.points.begin(), pruned.points.begin() + excess);
        }

        *terrain_cloud_ = std::move(pruned);
    }

    void publish_terrain_cloud()
    {
        if (!pub_terrain_ || pub_terrain_->get_subscription_count() == 0)
            return;

        std::lock_guard<std::mutex> lock(terrain_mutex_);
        if (!terrain_cloud_ || terrain_cloud_->empty())
            return;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*terrain_cloud_, msg);
        msg.header.stamp = now();
        msg.header.frame_id = odom_frame_;
        pub_terrain_->publish(msg);
    }

    // =========================================================================
    // Member variables
    // =========================================================================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_gridmap_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_;
    rclcpp::TimerBase::SharedPtr terrain_timer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
    std::mutex terrain_mutex_;

    double cos_yaw_ = 1.0;
    double sin_yaw_ = 0.0;

    // [v2] Temporal smoothing state
    double temporal_alpha_;
    int temporal_obstacle_persist_;
    std::vector<double> prev_costmap_;
    std::vector<int> obstacle_age_;

    // [v3] Inflation parameters
    double inflation_radius_;
    double inflation_decay_rate_;
    int inflation_lethal_cost_;

    // Parameters
    double costmap_size_;
    double costmap_resolution_;
    int costmap_cells_;
    std::string robot_frame_;
    std::string odom_frame_;

    double w_obstacle_;
    double w_slope_;
    double w_roughness_;
    double w_unknown_;

    double slope_max_rad_;
    double roughness_max_;
    double obstacle_count_max_;
    double confidence_threshold_;
    double lethal_threshold_;

    bool enable_terrain_accum_;
    int terrain_max_points_;
    double terrain_radius_;

    std::string gridmap_topic_;
    std::string filtered_cloud_topic_;
    std::string costmap_topic_;
    std::string terrain_cloud_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainCostmapNode>());
    rclcpp::shutdown();
    return 0;
}