// terrain_costmap_node.cpp
// Generates a 5m×5m robot-centric costmap from GroundGrid output.
//
// Inputs:
//   /groundgrid/grid_map      (grid_map_msgs/GridMap)  — elevation, variance, confidence
//   /groundgrid/filtered_cloud (PointCloud2)            — ground(49) + obstacle(99) labeled
//   TF: odom → base_link chain
//
// Outputs:
//   /terrain_costmap           (nav_msgs/OccupancyGrid) — 5m×5m traversability costmap
//   /terrain_costmap/terrain_cloud (PointCloud2)        — accumulated terrain for debug
//
// Cost factors:
//   1. Obstacle density   — non-ground points in each cell
//   2. Slope              — elevation gradient between neighbor cells
//   3. Roughness          — height variance within cell
//   4. Uncertainty        — low groundpatch confidence = unknown area
//
// [v3] Dynamic obstacle support:
//   Obstacle observations are tracked in odom-frame persistent grid with timestamps.
//   When GroundGrid reports ground in a cell, obstacle history is cleared immediately.
//   Obstacle costs decay over time (configurable half-life) so that
//   dynamic obstacles (people, animals) don't leave permanent traces.
//
// Persistent terrain map:
//   Ground points are accumulated in odom frame over time (sliding window)
//   to fill sensor blind spots as the robot moves.

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
#include <unordered_map>

// Per-cell statistics from accumulated terrain points (odom frame → robot frame)
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

// =========================================================================
// [v3] NEW: Persistent obstacle tracking cell (odom frame)
// =========================================================================
struct ObstacleCell {
    double obstacle_score = 0.0;     // 누적 장애물 점수 [0, 1]
    double last_observed_sec = 0.0;  // 마지막 장애물 관측 시각 (초)
    double last_cleared_sec = 0.0;   // 마지막으로 ground로 확인된 시각 (초)
};

// Hash for odom-frame grid cell (discretized by resolution)
struct OdomCellKey {
    int gx, gy;
    bool operator==(const OdomCellKey &o) const { return gx == o.gx && gy == o.gy; }
};

struct OdomCellHash {
    size_t operator()(const OdomCellKey &k) const {
        return std::hash<int>()(k.gx) ^ (std::hash<int>()(k.gy) << 16);
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
        // [v3]
        RCLCPP_INFO(get_logger(),
            "Dynamic obstacle: decay_time=%.1fs, clear_on_ground=%s",
            obstacle_decay_time_,
            obstacle_clear_on_ground_ ? "true" : "false");
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

        // Cost weights (sum should be 1.0 for clarity, but not enforced)
        this->declare_parameter<double>("weight_obstacle", 0.4);
        this->declare_parameter<double>("weight_slope", 0.3);
        this->declare_parameter<double>("weight_roughness", 0.15);
        this->declare_parameter<double>("weight_unknown", 0.15);

        // Thresholds
        this->declare_parameter<double>("slope_max_deg", 35.0);
        this->declare_parameter<double>("roughness_max", 0.05);
        this->declare_parameter<double>("obstacle_count_max", 5.0);
        this->declare_parameter<double>("confidence_threshold", 0.1);

        // Lethal cost: cells above this total cost → 100 (impassable)
        this->declare_parameter<double>("lethal_threshold", 0.8);

        // Terrain accumulation
        this->declare_parameter<bool>("enable_terrain_accumulation", true);
        this->declare_parameter<int>("terrain_max_points", 200000);
        this->declare_parameter<double>("terrain_radius", 10.0);
        this->declare_parameter<double>("terrain_publish_rate", 1.0);

        // Topics
        this->declare_parameter<std::string>("gridmap_topic", "/groundgrid/grid_map");
        this->declare_parameter<std::string>("filtered_cloud_topic", "/groundgrid/filtered_cloud");
        this->declare_parameter<std::string>("costmap_topic", "/terrain_costmap");
        this->declare_parameter<std::string>("terrain_cloud_topic", "/terrain_costmap/terrain_cloud");

        // === [v3] NEW: Dynamic obstacle parameters ===
        this->declare_parameter<double>("obstacle_decay_time", 2.0);       // [s] 장애물 비용 완전 소멸 시간
        this->declare_parameter<bool>("obstacle_clear_on_ground", true);   // ground 관측 시 즉시 클리어
        this->declare_parameter<double>("obstacle_grid_radius", 8.0);      // [m] odom 장애물 그리드 유지 반경
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

        costmap_cells_ = static_cast<int>(std::round(costmap_size_ / costmap_resolution_));

        gridmap_topic_ = get_parameter("gridmap_topic").as_string();
        filtered_cloud_topic_ = get_parameter("filtered_cloud_topic").as_string();
        costmap_topic_ = get_parameter("costmap_topic").as_string();
        terrain_cloud_topic_ = get_parameter("terrain_cloud_topic").as_string();

        // [v3]
        obstacle_decay_time_ = get_parameter("obstacle_decay_time").as_double();
        obstacle_clear_on_ground_ = get_parameter("obstacle_clear_on_ground").as_bool();
        obstacle_grid_radius_ = get_parameter("obstacle_grid_radius").as_double();
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
    // [v3] Odom-frame coordinate → grid key conversion
    // =========================================================================
    OdomCellKey odom_to_key(double ox, double oy) const
    {
        return {
            static_cast<int>(std::floor(ox / costmap_resolution_)),
            static_cast<int>(std::floor(oy / costmap_resolution_))
        };
    }

    // =========================================================================
    // [v3] Query decayed obstacle cost for an odom-frame position
    // =========================================================================
    double get_decayed_obstacle_cost(double ox, double oy, double now_sec) const
    {
        auto key = odom_to_key(ox, oy);
        std::lock_guard<std::mutex> lock(obstacle_grid_mutex_);
        auto it = obstacle_grid_.find(key);
        if (it == obstacle_grid_.end())
            return 0.0;

        const auto &cell = it->second;

        // ground로 클리어된 셀은 비용 0
        if (cell.last_cleared_sec > cell.last_observed_sec)
            return 0.0;

        // 시간 기반 선형 감쇠: elapsed가 decay_time 이상이면 완전 소멸
        double elapsed = now_sec - cell.last_observed_sec;
        if (elapsed >= obstacle_decay_time_)
            return 0.0;

        double decay_factor = 1.0 - (elapsed / obstacle_decay_time_);
        return cell.obstacle_score * decay_factor;
    }

    // =========================================================================
    // [v3] Update obstacle grid from filtered cloud (obstacle + ground)
    // =========================================================================
    void update_obstacle_grid(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &tf_to_odom)
    {
        const auto &fields = msg->fields;
        int intensity_offset = -1;
        for (const auto &f : fields) {
            if (f.name == "intensity") {
                intensity_offset = f.offset;
                break;
            }
        }
        if (intensity_offset < 0) return;

        const float tx = tf_to_odom.transform.translation.x;
        const float ty = tf_to_odom.transform.translation.y;
        const float tz = tf_to_odom.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(tf_to_odom.transform.rotation, q);
        tf2::Matrix3x3 rot(q);

        const double now_sec = rclcpp::Time(msg->header.stamp).seconds();
        const uint8_t *data = msg->data.data();
        const size_t step = msg->point_step;
        const int x_off = msg->fields[0].offset;
        const int y_off = msg->fields[1].offset;
        const int z_off = msg->fields[2].offset;

        // 셀별 프레임 내 카운트 (obstacle / ground)
        struct FrameCount { int obs = 0; int gnd = 0; };
        std::unordered_map<OdomCellKey, FrameCount, OdomCellHash> frame_counts;

        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            const float intensity = *reinterpret_cast<const float *>(
                data + i * step + intensity_offset);

            const float px = *reinterpret_cast<const float *>(data + i * step + x_off);
            const float py = *reinterpret_cast<const float *>(data + i * step + y_off);
            const float pz = *reinterpret_cast<const float *>(data + i * step + z_off);

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
                continue;

            // Transform to odom frame
            double ox = rot[0][0]*px + rot[0][1]*py + rot[0][2]*pz + tx;
            double oy = rot[1][0]*px + rot[1][1]*py + rot[1][2]*pz + ty;

            auto key = odom_to_key(ox, oy);

            bool is_obstacle = (std::abs(intensity - 99.0f) < 5.0f);
            bool is_ground = (std::abs(intensity - 49.0f) < 5.0f);

            if (is_obstacle) {
                frame_counts[key].obs++;
            } else if (is_ground) {
                frame_counts[key].gnd++;
            }
        }

        // 장애물 그리드 업데이트
        std::lock_guard<std::mutex> lock(obstacle_grid_mutex_);
        for (const auto &[key, cnt] : frame_counts) {
            auto &cell = obstacle_grid_[key];

            if (cnt.obs > 0) {
                // 장애물 관측: 점수 업데이트
                double new_score = std::min(
                    static_cast<double>(cnt.obs) / obstacle_count_max_, 1.0);
                // 기존 값과 새 값 중 큰 쪽 유지 (한 프레임 내)
                cell.obstacle_score = std::max(cell.obstacle_score, new_score);
                cell.last_observed_sec = now_sec;
            }

            if (obstacle_clear_on_ground_ && cnt.gnd > 0 && cnt.obs == 0) {
                // 이 프레임에서 ground만 관측 → 장애물 즉시 클리어
                cell.last_cleared_sec = now_sec;
            }
        }

        // 오래된 셀 정리 (로봇 위치 기준 반경 밖 + 완전 감쇠된 셀)
        prune_obstacle_grid(tx, ty, now_sec);
    }

    // =========================================================================
    // [v3] Prune old obstacle grid entries
    // =========================================================================
    void prune_obstacle_grid(double robot_x, double robot_y, double now_sec)
    {
        // 이미 lock 상태 (caller에서 잡음)
        const double r2 = obstacle_grid_radius_ * obstacle_grid_radius_;

        for (auto it = obstacle_grid_.begin(); it != obstacle_grid_.end(); ) {
            double cx = (it->first.gx + 0.5) * costmap_resolution_;
            double cy = (it->first.gy + 0.5) * costmap_resolution_;
            double dx = cx - robot_x;
            double dy = cy - robot_y;
            double dist2 = dx * dx + dy * dy;

            // 반경 밖이거나 완전히 감쇠된 셀 제거
            bool expired = (now_sec - it->second.last_observed_sec) >= obstacle_decay_time_;
            bool cleared = (it->second.last_cleared_sec > it->second.last_observed_sec);
            bool out_of_range = dist2 > r2;

            if (out_of_range || (expired && cleared)) {
                it = obstacle_grid_.erase(it);
            } else {
                ++it;
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

        // 4. Extract robot yaw for coordinate transform
        tf2::Quaternion q;
        tf2::fromMsg(tf_odom_to_robot.transform.rotation, q);
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

        // Header: costmap is in robot frame
        costmap->header.stamp = tf_odom_to_robot.header.stamp;
        costmap->header.frame_id = robot_frame_;

        // Grid metadata
        costmap->info.resolution = costmap_resolution_;
        costmap->info.width = costmap_cells_;
        costmap->info.height = costmap_cells_;

        // Origin: bottom-left corner of the costmap in robot frame
        costmap->info.origin.position.x = -costmap_size_ / 2.0;
        costmap->info.origin.position.y = -costmap_size_ / 2.0;
        costmap->info.origin.position.z = 0.0;
        costmap->info.origin.orientation.w = 1.0;

        costmap->data.resize(costmap_cells_ * costmap_cells_, -1);  // -1 = unknown

        // Build terrain accumulation grid for fallback on unknown cells
        std::vector<TerrainCellStats> terrain_grid;
        if (enable_terrain_accum_ && terrain_cloud_ && !terrain_cloud_->empty()) {
            terrain_grid = build_terrain_grid(tf_odom_to_robot);
        }

        // Get submap layers (check existence)
        const bool has_ground = submap.exists("ground");
        const bool has_points = submap.exists("points");
        const bool has_variance = submap.exists("variance");
        const bool has_confidence = submap.exists("groundpatch");

        if (!has_ground) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "GridMap missing 'ground' layer — cannot compute slope");
        }

        // Get robot position for coordinate mapping
        const grid_map::Position submap_center = submap.getPosition();
        const float submap_res = submap.getResolution();

        // [v3] 현재 시각 (감쇠 계산용)
        const double now_sec = rclcpp::Time(tf_odom_to_robot.header.stamp).seconds();

        // Iterate over costmap cells
        for (int cy = 0; cy < costmap_cells_; ++cy) {
            for (int cx = 0; cx < costmap_cells_; ++cx) {
                // Costmap cell center in robot frame
                const double rx = costmap->info.origin.position.x
                                + (cx + 0.5) * costmap_resolution_;
                const double ry = costmap->info.origin.position.y
                                + (cy + 0.5) * costmap_resolution_;

                // Transform to odom frame for GridMap lookup
                const double ox = rx * cos_yaw_ - ry * sin_yaw_
                                + tf_odom_to_robot.transform.translation.x;
                const double oy = rx * sin_yaw_ + ry * cos_yaw_
                                + tf_odom_to_robot.transform.translation.y;

                grid_map::Position query_pos(ox, oy);
                if (!submap.isInside(query_pos))
                    continue;

                // === Compute individual cost factors ===
                double obstacle_cost = 0.0;
                double slope_cost = 0.0;
                double roughness_cost = 0.0;
                double unknown_cost = 0.0;

                grid_map::Index idx;
                submap.getIndex(query_pos, idx);

                // (a) Obstacle density — [v3] from GroundGrid (instantaneous)
                // [v1] 원본: GroundGrid의 현재 프레임 obstacle만 사용
                // double instantaneous_obstacle = 0.0;
                // if (has_points) {
                //     float pt_count = submap.at("points", idx);
                //     instantaneous_obstacle = std::min(
                //         static_cast<double>(pt_count) / obstacle_count_max_, 1.0);
                // }
                //
                // [v3] 변경: 현재 프레임 + odom 기반 감쇠 장애물 중 큰 값 사용
                double instantaneous_obstacle = 0.0;
                if (has_points) {
                    float pt_count = submap.at("points", idx);
                    instantaneous_obstacle = std::min(
                        static_cast<double>(pt_count) / obstacle_count_max_, 1.0);
                }
                double decayed_obstacle = get_decayed_obstacle_cost(ox, oy, now_sec);
                obstacle_cost = std::max(instantaneous_obstacle, decayed_obstacle);

                // (b) Slope from elevation gradient
                if (has_ground) {
                    slope_cost = compute_slope(submap, idx);
                }

                // (c) Surface roughness
                if (has_variance) {
                    float var = submap.at("variance", idx);
                    roughness_cost = std::min(
                        static_cast<double>(var) / roughness_max_, 1.0);
                }

                // (d) Confidence / unknown penalty
                if (has_confidence) {
                    float conf = submap.at("groundpatch", idx);
                    if (conf < confidence_threshold_) {
                        unknown_cost = 1.0;  // Unknown area → high cost
                    } else {
                        unknown_cost = 0.0;
                    }
                }

                // === Weighted combination ===
                double total = w_obstacle_ * obstacle_cost
                             + w_slope_ * slope_cost
                             + w_roughness_ * roughness_cost
                             + w_unknown_ * unknown_cost;

                total = std::clamp(total, 0.0, 1.0);

                // Map to OccupancyGrid [0, 100]
                int8_t cell_value;
                if (obstacle_cost >= 0.99) {
                    cell_value = 100;  // Definite obstacle → lethal
                } else if (total >= lethal_threshold_) {
                    cell_value = 100;  // Above lethal threshold
                } else {
                    cell_value = static_cast<int8_t>(total * 99.0);
                }

                costmap->data[cy * costmap_cells_ + cx] = cell_value;
            }
        }

        // === Fallback: fill unknown cells from accumulated terrain ===
        if (enable_terrain_accum_ && !terrain_grid.empty()) {
            int filled = 0;
            for (int cy = 0; cy < costmap_cells_; ++cy) {
                for (int cx = 0; cx < costmap_cells_; ++cx) {
                    const int idx = cy * costmap_cells_ + cx;
                    if (costmap->data[idx] != -1)
                        continue;  // Already has data from GroundGrid

                    const auto &tcell = terrain_grid[idx];
                    if (tcell.count < 2)
                        continue;  // Not enough accumulated points

                    // Accumulated terrain contains ground-only points
                    // → obstacle_cost = 0, unknown_cost = 0
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

        return costmap;
    }

    // =========================================================================
    // Slope computation from elevation layer
    // =========================================================================
    double compute_slope(const grid_map::GridMap &map, const grid_map::Index &idx) const
    {
        const auto &size = map.getSize();
        const int i = idx(0);
        const int j = idx(1);

        // Need at least 1 neighbor on each side
        if (i < 1 || j < 1 || i >= size(0) - 1 || j >= size(1) - 1)
            return 0.0;

        const float res = map.getResolution();
        const auto &ground = map["ground"];

        // Central difference for gradient
        const float dz_dx = (ground(i + 1, j) - ground(i - 1, j)) / (2.0f * res);
        const float dz_dy = (ground(i, j + 1) - ground(i, j - 1)) / (2.0f * res);
        const float slope_rad = std::atan(std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy));

        // Normalize to [0, 1] based on max slope
        return std::min(static_cast<double>(slope_rad) / slope_max_rad_, 1.0);
    }

    // =========================================================================
    // Terrain grid from accumulated points (for unknown-cell fallback)
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
            // Odom → robot frame (2D rotation using cached yaw)
            const double dx = pt.x - tx;
            const double dy = pt.y - ty;
            const double rx =  cos_yaw_ * dx + sin_yaw_ * dy;
            const double ry = -sin_yaw_ * dx + cos_yaw_ * dy;

            // Check if within costmap bounds
            if (rx < -half || rx >= half || ry < -half || ry >= half)
                continue;

            // Convert to grid cell index
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

    // Compute slope from terrain grid mean elevations using central difference
    double compute_terrain_slope(
        const std::vector<TerrainCellStats> &tgrid, int cx, int cy) const
    {
        if (cx < 1 || cy < 1 || cx >= costmap_cells_ - 1 || cy >= costmap_cells_ - 1)
            return 0.0;

        const auto &left  = tgrid[cy * costmap_cells_ + (cx - 1)];
        const auto &right = tgrid[cy * costmap_cells_ + (cx + 1)];
        const auto &down  = tgrid[(cy - 1) * costmap_cells_ + cx];
        const auto &up    = tgrid[(cy + 1) * costmap_cells_ + cx];

        // Need neighbors to have data for gradient computation
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
    //                         + [v3] Dynamic obstacle tracking
    // =========================================================================
    void filtered_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        if (!enable_terrain_accum_)
            return;

        // Get TF: cloud frame → odom
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

        // [v3] 장애물 그리드 업데이트 (obstacle + ground 포인트 모두 활용)
        update_obstacle_grid(msg, tf_to_odom);

        // === 기존 terrain accumulation (ground 포인트만) ===
        const auto &fields = msg->fields;
        int intensity_offset = -1;
        for (const auto &f : fields) {
            if (f.name == "intensity") {
                intensity_offset = f.offset;
                break;
            }
        }
        if (intensity_offset < 0) return;

        const float tx = tf_to_odom.transform.translation.x;
        const float ty = tf_to_odom.transform.translation.y;
        const float tz = tf_to_odom.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(tf_to_odom.transform.rotation, q);
        tf2::Matrix3x3 rot(q);

        // Extract ground points and transform to odom frame
        std::lock_guard<std::mutex> lock(terrain_mutex_);

        const uint8_t *data = msg->data.data();
        const size_t step = msg->point_step;
        const int x_off = msg->fields[0].offset;
        const int y_off = msg->fields[1].offset;
        const int z_off = msg->fields[2].offset;

        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            const float intensity = *reinterpret_cast<const float *>(
                data + i * step + intensity_offset);

            // Only accumulate ground points (intensity ≈ 49)
            if (std::abs(intensity - 49.0f) > 5.0f)
                continue;

            const float px = *reinterpret_cast<const float *>(data + i * step + x_off);
            const float py = *reinterpret_cast<const float *>(data + i * step + y_off);
            const float pz = *reinterpret_cast<const float *>(data + i * step + z_off);

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
                continue;

            // Transform to odom frame
            tf2::Vector3 pt_in(px, py, pz);
            tf2::Vector3 pt_odom = tf2::Vector3(
                rot[0][0] * pt_in.x() + rot[0][1] * pt_in.y() + rot[0][2] * pt_in.z() + tx,
                rot[1][0] * pt_in.x() + rot[1][1] * pt_in.y() + rot[1][2] * pt_in.z() + ty,
                rot[2][0] * pt_in.x() + rot[2][1] * pt_in.y() + rot[2][2] * pt_in.z() + tz);

            pcl::PointXYZI pt;
            pt.x = pt_odom.x();
            pt.y = pt_odom.y();
            pt.z = pt_odom.z();
            pt.intensity = 49.0f;  // ground label preserved
            terrain_cloud_->points.push_back(pt);
        }

        // Sliding window: remove points outside radius from current robot position
        prune_terrain_cloud(tx, ty);
    }

    // =========================================================================
    // Terrain cloud management
    // =========================================================================
    void prune_terrain_cloud(float robot_x, float robot_y)
    {
        // Already locked by caller
        const double r2 = terrain_radius_ * terrain_radius_;
        auto &pts = terrain_cloud_->points;

        // Remove points outside sliding window
        pts.erase(
            std::remove_if(pts.begin(), pts.end(),
                [robot_x, robot_y, r2](const pcl::PointXYZI &p) {
                    const double dx = p.x - robot_x;
                    const double dy = p.y - robot_y;
                    return (dx * dx + dy * dy) > r2;
                }),
            pts.end());

        // If still over capacity, remove oldest (front of vector)
        if (static_cast<int>(pts.size()) > terrain_max_points_) {
            const int excess = pts.size() - terrain_max_points_;
            pts.erase(pts.begin(), pts.begin() + excess);
        }

        terrain_cloud_->width = pts.size();
        terrain_cloud_->height = 1;
        terrain_cloud_->is_dense = true;
    }

    void publish_terrain_cloud()
    {
        if (!pub_terrain_ || pub_terrain_->get_subscription_count() == 0)
            return;

        std::lock_guard<std::mutex> lock(terrain_mutex_);
        if (terrain_cloud_->empty())
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

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_gridmap_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_;
    rclcpp::TimerBase::SharedPtr terrain_timer_;

    // Terrain accumulation
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
    std::mutex terrain_mutex_;

    // [v3] Dynamic obstacle tracking (odom frame)
    std::unordered_map<OdomCellKey, ObstacleCell, OdomCellHash> obstacle_grid_;
    mutable std::mutex obstacle_grid_mutex_;

    // Cached yaw (updated each frame)
    double cos_yaw_ = 1.0;
    double sin_yaw_ = 0.0;

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

    // [v3] Dynamic obstacle parameters
    double obstacle_decay_time_;
    bool obstacle_clear_on_ground_;
    double obstacle_grid_radius_;

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