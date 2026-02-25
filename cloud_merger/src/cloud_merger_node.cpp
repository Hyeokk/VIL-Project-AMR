// cloud_merger_node.cpp
// Merges FAST-LIO2 registered cloud (M300) + S10 Ultra depth cloud
// for GroundGrid ground segmentation.
//
// M300: 360° but blind below -10° → can't see ground within ~3.6m
// S10 Ultra: 120°×80° forward, fills the near-ground gap
//
// Output: PointXYZI in body frame at 10Hz (driven by M300 rate)
// If S10 Ultra is unavailable, M300 passes through alone.
//
// Height filter: removes points above max_height in body frame
// to prevent ceiling/sky points from becoming false obstacles
// in GroundGrid (indoor, tunnel environments).
// Only upper bound is applied; no lower bound to preserve
// ground points on slopes and downhill terrain.
//
// NOTE: M300 uses sensor-internal clock, S10 Ultra uses system clock.
//       Staleness is checked via wall-clock receive time, not message stamps.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <algorithm>

class CloudMergerNode : public rclcpp::Node
{
public:
    CloudMergerNode() : Node("cloud_merger")
    {
        // Parameters
        this->declare_parameter<std::string>("m300_topic", "/cloud_registered_body");
        this->declare_parameter<std::string>("s10_topic", "/lx_camera_node/LxCamera_Cloud");
        this->declare_parameter<std::string>("merged_topic", "/merged_cloud");
        this->declare_parameter<std::string>("output_frame", "body");
        this->declare_parameter<double>("s10_stale_threshold", 0.5);  // seconds

        // Height crop filter (ground-referenced)
        // Removes ceiling/sky points before GroundGrid to prevent false obstacles
        // sensor_height: body frame (m300_link) height from ground [m]
        // max_height_from_ground: points above this height from ground are removed
        // Internal threshold: max_height_from_ground - sensor_height (body frame z)
        // Only upper bound; no lower bound to preserve ground on slopes
        this->declare_parameter<bool>("enable_height_filter", true);
        this->declare_parameter<double>("max_height_from_ground", 2.0);  // [m] from ground
        this->declare_parameter<double>("sensor_height", 0.63);          // [m] body frame above ground

        auto m300_topic = this->get_parameter("m300_topic").as_string();
        auto s10_topic = this->get_parameter("s10_topic").as_string();
        auto merged_topic = this->get_parameter("merged_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        s10_stale_sec_ = this->get_parameter("s10_stale_threshold").as_double();
        enable_height_filter_ = this->get_parameter("enable_height_filter").as_bool();
        double max_height_from_ground = this->get_parameter("max_height_from_ground").as_double();
        double sensor_height = this->get_parameter("sensor_height").as_double();
        max_height_body_ = max_height_from_ground - sensor_height;  // convert to body frame z

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        sub_m300_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m300_topic, rclcpp::SensorDataQoS(),
            std::bind(&CloudMergerNode::m300_callback, this, std::placeholders::_1));

        sub_s10_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            s10_topic, rclcpp::SensorDataQoS(),
            std::bind(&CloudMergerNode::s10_callback, this, std::placeholders::_1));

        // Publisher
        pub_merged_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            merged_topic, rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(),
            "CloudMerger: M300(%s) + S10(%s) → %s [frame: %s]",
            m300_topic.c_str(), s10_topic.c_str(),
            merged_topic.c_str(), output_frame_.c_str());

        if (enable_height_filter_) {
            RCLCPP_INFO(this->get_logger(),
                "Height filter enabled: max %.2fm from ground (body z <= %.2fm), sensor_height=%.2fm",
                max_height_from_ground, max_height_body_, sensor_height);
        }
    }

private:
    // Store latest S10 Ultra cloud with wall-clock receive time
    void s10_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(s10_mutex_);
        latest_s10_ = msg;
        s10_recv_time_ = std::chrono::steady_clock::now();
        s10_received_ = true;

        // Cache static TF on first receive
        if (!tf_cached_) {
            try {
                auto tf = tf_buffer_->lookupTransform(
                    output_frame_, msg->header.frame_id,
                    tf2::TimePointZero, tf2::durationFromSec(1.0));

                Eigen::Quaternionf q(
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);

                // p_body = R * p_s10 + t
                tf_s10_to_body_ = Eigen::Affine3f::Identity();
                tf_s10_to_body_.translation() = Eigen::Vector3f(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
                tf_s10_to_body_.linear() = q.toRotationMatrix();

                tf_cached_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Cached TF %s → %s: t=[%.3f, %.3f, %.3f]",
                    msg->header.frame_id.c_str(), output_frame_.c_str(),
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Waiting for TF %s → %s: %s",
                    msg->header.frame_id.c_str(), output_frame_.c_str(), ex.what());
            }
        }
    }

    // M300 callback drives the merge pipeline
    void m300_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // Read M300 as PointXYZI (works with both PointXYZI and PointXYZINormal input)
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI> m300_raw;
        pcl::fromROSMsg(*msg, m300_raw);

        merged->points.reserve(m300_raw.points.size() + 40000);
        for (const auto &pt : m300_raw.points) {
            merged->points.push_back(pt);
        }

        // Try to merge S10 Ultra
        bool s10_merged = false;
        size_t s10_count = 0;
        {
            std::lock_guard<std::mutex> lock(s10_mutex_);
            if (s10_received_ && tf_cached_ && latest_s10_) {
                // Check staleness using wall-clock receive time
                // (M300 and S10 use different clock domains)
                auto now = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(now - s10_recv_time_).count();

                if (dt < s10_stale_sec_) {
                    // Convert S10 (PointXYZRGB) → PointXYZI, apply TF
                    pcl::PointCloud<pcl::PointXYZRGB> s10_rgb;
                    pcl::fromROSMsg(*latest_s10_, s10_rgb);

                    for (const auto &pt : s10_rgb.points) {
                        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                            continue;
                        // Skip zero points (invalid depth)
                        if (pt.x == 0.0f && pt.y == 0.0f && pt.z == 0.0f)
                            continue;

                        // Transform to body frame
                        Eigen::Vector3f p_s10(pt.x, pt.y, pt.z);
                        Eigen::Vector3f p_body = tf_s10_to_body_ * p_s10;

                        pcl::PointXYZI p;
                        p.x = p_body.x();
                        p.y = p_body.y();
                        p.z = p_body.z();
                        // Intensity from RGB luminance
                        p.intensity = 0.299f * pt.r + 0.587f * pt.g + 0.114f * pt.b;
                        merged->points.push_back(p);
                    }
                    s10_count = merged->points.size() - m300_raw.points.size();
                    s10_merged = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "S10 cloud stale: %.3fs ago (threshold: %.1fs)", dt, s10_stale_sec_);
                }
            }
        }

        // Height crop filter: remove ceiling/sky points (upper bound only)
        // No lower bound to preserve ground points on slopes/downhill
        if (enable_height_filter_) {
            size_t before = merged->points.size();
            merged->points.erase(
                std::remove_if(merged->points.begin(), merged->points.end(),
                    [this](const pcl::PointXYZI &pt) {
                        return pt.z > max_height_body_;
                    }),
                merged->points.end());

            size_t removed = before - merged->points.size();
            if (removed > 0) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Height filter: removed %zu/%zu pts above body z=%.1fm",
                    removed, before, max_height_body_);
            }
        }

        // Publish
        merged->width = merged->points.size();
        merged->height = 1;
        merged->is_dense = true;

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*merged, out_msg);
        out_msg.header = msg->header;  // Use M300 timestamp, body frame
        out_msg.header.frame_id = output_frame_;
        pub_merged_->publish(out_msg);

        if (s10_merged) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Merged: M300(%zu) + S10(%zu) = %zu pts",
                m300_raw.points.size(), s10_count, merged->points.size());
        }
    }

    // Subscribers & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_m300_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_s10_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Affine3f tf_s10_to_body_ = Eigen::Affine3f::Identity();
    bool tf_cached_ = false;

    // S10 Ultra latest cloud
    std::mutex s10_mutex_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_s10_;
    std::chrono::steady_clock::time_point s10_recv_time_;
    bool s10_received_ = false;

    // Config
    std::string output_frame_;
    double s10_stale_sec_;
    bool enable_height_filter_;
    double max_height_body_;    // computed: max_height_from_ground - sensor_height
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudMergerNode>());
    rclcpp::shutdown();
    return 0;
}