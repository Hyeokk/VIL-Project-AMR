// terrain_costmap_node.cpp
// Generates a 3m×3m robot-centric costmap from GroundGrid output.
//
// Inputs:
//   /groundgrid/grid_map      (grid_map_msgs/GridMap)  — elevation, variance, confidence
//   /groundgrid/filtered_cloud (PointCloud2)            — ground(49) + obstacle(99) labeled
//   TF: odom → base_link chain
//
// Outputs:
//   /terrain_costmap           (nav_msgs/OccupancyGrid) — 3m×3m traversability costmap
//   /terrain_costmap/terrain_cloud (PointCloud2)        — accumulated terrain for debug
//
// Cost factors:
//   1. Obstacle density   — non-ground points in each cell
//   2. Slope              — elevation gradient between neighbor cells
//   3. Roughness          — height variance within cell
//   4. Uncertainty        — low groundpatch confidence = unknown area
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
        // Robot is at center of costmap, so origin is at (-size/2, -size/2)
        costmap->info.origin.position.x = -costmap_size_ / 2.0;
        costmap->info.origin.position.y = -costmap_size_ / 2.0;
        costmap->info.origin.position.z = 0.0;
        costmap->info.origin.orientation.w = 1.0;

        costmap->data.resize(costmap_cells_ * costmap_cells_, -1);  // -1 = unknown

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

                // (a) Obstacle density
                if (has_points) {
                    float pt_count = submap.at("points", idx);
                    obstacle_cost = std::min(
                        static_cast<double>(pt_count) / obstacle_count_max_, 1.0);
                }

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
    // Filtered cloud callback — Persistent terrain accumulation
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

        // Parse intensity to separate ground/obstacle
        // GroundGrid visualize mode: ground=49, obstacle=99
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