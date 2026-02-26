// path_planner_node.cpp
// A* Global Planner + DWA Local Planner for DONKEYBOTI Caterpillar Robot
//
// Designed for:
//   - 5m x 5m costmap, 0.1m resolution (50x50 cells)
//   - Differential-drive (skid-steer) kinematics
//   - Turn-in-place capability (caterpillar tracks)
//   - Terrain-aware cost traversal
//
// Subscriptions:
//   /terrain_costmap    (nav_msgs/OccupancyGrid)  — 50x50 traversability
//   /goal_pose          (geometry_msgs/PoseStamped) — navigation target
//
// Publications:
//   /cmd_vel            (geometry_msgs/Twist)       — velocity command
//   /path_planner/global_path     (nav_msgs/Path)  — A* path (viz)
//   /path_planner/local_trajectory (nav_msgs/Path) — DWA best trajectory (viz)
//
// TF Required:
//   odom → base_link  (from FAST-LIO2 or wheel odometry)
//
// [v3] Fixes:
//   - Goal frame: transform any frame to odom on receipt
//   - DWA scoring: forward-bias to prevent unnecessary spinning
//   - Robot footprint: oriented rectangular collision check

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <mutex>
#include <thread>

// =========================================================================
// Helper: normalize angle to [-π, π]
// =========================================================================
static inline double normalize_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// =========================================================================
// A* grid cell
// =========================================================================
struct Cell {
    int r, c;
    bool operator==(const Cell &o) const { return r == o.r && c == o.c; }
};

struct CellHash {
    size_t operator()(const Cell &c) const {
        return std::hash<int>()(c.r) ^ (std::hash<int>()(c.c) << 16);
    }
};

struct AStarEntry {
    double f;
    Cell cell;
    bool operator>(const AStarEntry &o) const { return f > o.f; }
};

// =========================================================================
// PathPlannerNode
// =========================================================================
class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        declare_parameters();
        load_parameters();
        setup_tf();
        setup_pub_sub();
        setup_timer();

        RCLCPP_INFO(get_logger(),
            "PathPlanner: costmap %dx%d (%.1fm), v=[%.2f,%.2f], w=[%.2f,%.2f]",
            grid_cells_, grid_cells_, costmap_size_,
            min_lin_vel_, max_lin_vel_, -max_ang_vel_, max_ang_vel_);
        RCLCPP_INFO(get_logger(),
            "DWA: %d v x %d w = %d trajectories, sim=%.2fs, dt=%.3fs",
            n_v_samples_, n_w_samples_, n_v_samples_ * n_w_samples_,
            sim_time_, sim_granularity_);
        RCLCPP_INFO(get_logger(),
            "Robot footprint: %.3f x %.3f m, clearance=%.2fm",
            robot_length_, robot_width_, clearance_margin_);
        RCLCPP_INFO(get_logger(),
            "Path caching: replan=%.1fs, deviation=%.2fm, lookahead=%.2fm",
            replan_interval_, path_deviation_threshold_, pure_pursuit_lookahead_);
        RCLCPP_INFO(get_logger(),
            "DWA weights: heading=%.2f, clearance=%.2f, velocity=%.2f, path=%.2f, spin_thresh=%.1f°",
            w_heading_, w_clearance_, w_velocity_, w_path_dist_,
            spin_heading_threshold_ * 180.0 / M_PI);
    }

    ~PathPlannerNode() override
    {
        send_stop_command();
    }

    void send_stop_command()
    {
        RCLCPP_INFO(get_logger(), "Shutdown: sending stop command to /cmd_vel");
        geometry_msgs::msg::Twist stop;
        for (int i = 0; i < 10; ++i) {
            pub_cmd_->publish(stop);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    // =====================================================================
    // Parameters
    // =====================================================================
    void declare_parameters()
    {
        declare_parameter<double>("costmap_size", 5.0);
        declare_parameter<double>("costmap_resolution", 0.1);

        declare_parameter<double>("max_lin_vel", 0.6);
        declare_parameter<double>("min_lin_vel", -0.2);
        declare_parameter<double>("max_ang_vel", 0.5);
        declare_parameter<double>("max_lin_acc", 1.0);
        declare_parameter<double>("max_ang_acc", 2.0);

        declare_parameter<int>("n_v_samples", 11);
        declare_parameter<int>("n_w_samples", 21);
        declare_parameter<double>("sim_time", 1.5);
        declare_parameter<double>("sim_granularity", 0.1);

        // [v3] Rebalanced defaults
        declare_parameter<double>("weight_heading", 0.6);
        declare_parameter<double>("weight_clearance", 0.5);
        declare_parameter<double>("weight_velocity", 0.8);
        declare_parameter<double>("weight_path_dist", 0.8);

        declare_parameter<double>("goal_xy_tolerance", 0.2);
        declare_parameter<double>("goal_yaw_tolerance", 0.15);

        declare_parameter<double>("astar_lethal_cost", 90.0);
        declare_parameter<double>("astar_cost_weight", 1.0);

        // [v3] Rectangular footprint
        declare_parameter<double>("robot_length", 1.432);
        declare_parameter<double>("robot_width", 0.850);
        declare_parameter<double>("clearance_margin", 0.1);

        declare_parameter<double>("control_rate", 20.0);

        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("robot_frame", "base_link");

        declare_parameter<std::string>("costmap_topic", "/terrain_costmap");
        declare_parameter<std::string>("goal_topic", "/goal_pose");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        declare_parameter<double>("slope_speed_factor", 0.5);
        declare_parameter<double>("slope_threshold_deg", 15.0);

        declare_parameter<double>("replan_interval", 1.0);
        declare_parameter<double>("path_deviation_threshold", 0.5);
        declare_parameter<double>("pure_pursuit_lookahead", 0.4);
        declare_parameter<double>("pure_pursuit_lookahead_min", 0.2);
        declare_parameter<double>("pure_pursuit_lookahead_max", 0.8);
        declare_parameter<double>("velocity_lookahead_gain", 0.5);

        // [v3]
        declare_parameter<double>("spin_heading_threshold", 1.05);
    }

    void load_parameters()
    {
        costmap_size_ = get_parameter("costmap_size").as_double();
        costmap_resolution_ = get_parameter("costmap_resolution").as_double();
        grid_cells_ = static_cast<int>(std::round(costmap_size_ / costmap_resolution_));

        max_lin_vel_ = get_parameter("max_lin_vel").as_double();
        min_lin_vel_ = get_parameter("min_lin_vel").as_double();
        max_ang_vel_ = get_parameter("max_ang_vel").as_double();
        max_lin_acc_ = get_parameter("max_lin_acc").as_double();
        max_ang_acc_ = get_parameter("max_ang_acc").as_double();

        n_v_samples_ = get_parameter("n_v_samples").as_int();
        n_w_samples_ = get_parameter("n_w_samples").as_int();
        sim_time_ = get_parameter("sim_time").as_double();
        sim_granularity_ = get_parameter("sim_granularity").as_double();

        w_heading_ = get_parameter("weight_heading").as_double();
        w_clearance_ = get_parameter("weight_clearance").as_double();
        w_velocity_ = get_parameter("weight_velocity").as_double();
        w_path_dist_ = get_parameter("weight_path_dist").as_double();

        goal_xy_tol_ = get_parameter("goal_xy_tolerance").as_double();
        goal_yaw_tol_ = get_parameter("goal_yaw_tolerance").as_double();

        astar_lethal_ = get_parameter("astar_lethal_cost").as_double();
        astar_cost_weight_ = get_parameter("astar_cost_weight").as_double();

        // [v3] Rectangular footprint
        robot_length_ = get_parameter("robot_length").as_double();
        robot_width_ = get_parameter("robot_width").as_double();
        clearance_margin_ = get_parameter("clearance_margin").as_double();

        control_rate_ = get_parameter("control_rate").as_double();

        odom_frame_ = get_parameter("odom_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();

        slope_speed_factor_ = get_parameter("slope_speed_factor").as_double();
        slope_threshold_deg_ = get_parameter("slope_threshold_deg").as_double();

        replan_interval_ = get_parameter("replan_interval").as_double();
        path_deviation_threshold_ = get_parameter("path_deviation_threshold").as_double();
        pure_pursuit_lookahead_ = get_parameter("pure_pursuit_lookahead").as_double();
        pure_pursuit_lookahead_min_ = get_parameter("pure_pursuit_lookahead_min").as_double();
        pure_pursuit_lookahead_max_ = get_parameter("pure_pursuit_lookahead_max").as_double();
        velocity_lookahead_gain_ = get_parameter("velocity_lookahead_gain").as_double();

        spin_heading_threshold_ = get_parameter("spin_heading_threshold").as_double();
    }

    // =====================================================================
    // Setup
    // =====================================================================
    void setup_tf()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void setup_pub_sub()
    {
        auto costmap_topic = get_parameter("costmap_topic").as_string();
        auto goal_topic = get_parameter("goal_topic").as_string();
        auto cmd_topic = get_parameter("cmd_vel_topic").as_string();

        sub_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&PathPlannerNode::costmap_callback, this, std::placeholders::_1));

        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10,
            std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
        pub_global_path_ = create_publisher<nav_msgs::msg::Path>(
            "/path_planner/global_path", 10);
        pub_local_traj_ = create_publisher<nav_msgs::msg::Path>(
            "/path_planner/local_trajectory", 10);
    }

    void setup_timer()
    {
        double period = 1.0 / control_rate_;
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&PathPlannerNode::control_loop, this));
    }

    // =====================================================================
    // Callbacks
    // =====================================================================
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_ = msg;
    }

    // [v3] Goal: transform any frame → odom
    void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped goal_odom;

        if (msg->header.frame_id == odom_frame_ || msg->header.frame_id.empty()) {
            goal_odom = *msg;
            goal_odom.header.frame_id = odom_frame_;
        } else {
            // Try TF transform first
            try {
                goal_odom = tf_buffer_->transform(
                    *msg, odom_frame_, tf2::durationFromSec(0.2));
            } catch (tf2::TransformException &ex) {
                // Fallback: assume robot-frame goal, manually transform
                double robot_x, robot_y, robot_yaw;
                if (!get_robot_pose(robot_x, robot_y, robot_yaw)) {
                    RCLCPP_ERROR(get_logger(),
                        "Cannot transform goal from '%s' to '%s': %s",
                        msg->header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
                    return;
                }
                double lx = msg->pose.position.x;
                double ly = msg->pose.position.y;
                double cos_y = std::cos(robot_yaw);
                double sin_y = std::sin(robot_yaw);

                goal_odom.header.stamp = msg->header.stamp;
                goal_odom.header.frame_id = odom_frame_;
                goal_odom.pose.position.x = cos_y * lx - sin_y * ly + robot_x;
                goal_odom.pose.position.y = sin_y * lx + cos_y * ly + robot_y;
                goal_odom.pose.position.z = 0.0;

                double goal_yaw_local = get_yaw_from_quat(msg->pose.orientation);
                double goal_yaw_odom = robot_yaw + goal_yaw_local;
                tf2::Quaternion q;
                q.setRPY(0, 0, goal_yaw_odom);
                goal_odom.pose.orientation.x = q.x();
                goal_odom.pose.orientation.y = q.y();
                goal_odom.pose.orientation.z = q.z();
                goal_odom.pose.orientation.w = q.w();

                RCLCPP_WARN(get_logger(),
                    "TF failed, manual transform from '%s' → odom",
                    msg->header.frame_id.c_str());
            }
        }

        goal_ = goal_odom;
        goal_active_ = true;
        need_replan_ = true;

        RCLCPP_INFO(get_logger(),
            "New goal: (%.2f, %.2f) in odom [source: '%s']",
            goal_odom.pose.position.x, goal_odom.pose.position.y,
            msg->header.frame_id.c_str());
    }

    // =====================================================================
    // Replan check
    // =====================================================================
    bool should_replan(double robot_x, double robot_y)
    {
        if (need_replan_) {
            need_replan_ = false;
            return true;
        }
        if (cached_global_path_odom_.empty()) return true;

        double elapsed = (get_clock()->now() - last_replan_time_).seconds();
        if (elapsed >= replan_interval_) return true;

        double min_dist = std::numeric_limits<double>::max();
        for (const auto &wp : cached_global_path_odom_) {
            double d = std::hypot(robot_x - wp.first, robot_y - wp.second);
            min_dist = std::min(min_dist, d);
        }
        if (min_dist > path_deviation_threshold_) {
            RCLCPP_INFO(get_logger(), "Path deviation %.2fm — replanning", min_dist);
            return true;
        }
        return false;
    }

    // =====================================================================
    // Pure Pursuit: find lookahead on cached odom-frame path
    // =====================================================================
    bool find_pure_pursuit_target(
        double robot_x, double robot_y, double robot_yaw,
        double &target_local_x, double &target_local_y)
    {
        if (cached_global_path_odom_.empty()) return false;

        double lookahead = pure_pursuit_lookahead_
                         + velocity_lookahead_gain_ * std::abs(cur_v_);
        lookahead = std::clamp(lookahead,
                              pure_pursuit_lookahead_min_,
                              pure_pursuit_lookahead_max_);

        // Find closest point
        size_t closest_idx = 0;
        double closest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < cached_global_path_odom_.size(); ++i) {
            double d = std::hypot(
                cached_global_path_odom_[i].first - robot_x,
                cached_global_path_odom_[i].second - robot_y);
            if (d < closest_dist) {
                closest_dist = d;
                closest_idx = i;
            }
        }

        // Walk forward along path
        double accum = 0.0;
        size_t target_idx = closest_idx;
        for (size_t i = closest_idx; i + 1 < cached_global_path_odom_.size(); ++i) {
            double ddx = cached_global_path_odom_[i+1].first - cached_global_path_odom_[i].first;
            double ddy = cached_global_path_odom_[i+1].second - cached_global_path_odom_[i].second;
            accum += std::hypot(ddx, ddy);
            target_idx = i + 1;
            if (accum >= lookahead) break;
        }

        // Transform to robot-local
        double ddx = cached_global_path_odom_[target_idx].first - robot_x;
        double ddy = cached_global_path_odom_[target_idx].second - robot_y;
        double cos_y = std::cos(-robot_yaw);
        double sin_y = std::sin(-robot_yaw);
        target_local_x = cos_y * ddx - sin_y * ddy;
        target_local_y = sin_y * ddx + cos_y * ddy;
        return true;
    }

    // =====================================================================
    // Main control loop (20 Hz)
    // =====================================================================
    void control_loop()
    {
        if (!goal_active_) {
            publish_stop();
            return;
        }

        // 1. Costmap
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap = costmap_;
        }
        if (!costmap) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No costmap yet");
            return;
        }

        // 2. Robot pose
        double robot_x, robot_y, robot_yaw;
        if (!get_robot_pose(robot_x, robot_y, robot_yaw)) return;

        // 3. Goal (already in odom frame)
        double goal_x = goal_.pose.position.x;
        double goal_y = goal_.pose.position.y;

        double dx = goal_x - robot_x;
        double dy = goal_y - robot_y;
        double dist = std::hypot(dx, dy);

        // 4. Goal reached? — position only, no yaw alignment
        if (dist < goal_xy_tol_) {
            publish_stop();
            goal_active_ = false;
            cached_global_path_odom_.clear();
            RCLCPP_INFO(get_logger(), "Goal reached! (dist=%.2fm)", dist);
            return;
        }

        // 5. A* replan
        if (should_replan(robot_x, robot_y)) {
            double cos_ny = std::cos(-robot_yaw);
            double sin_ny = std::sin(-robot_yaw);
            double goal_lx = cos_ny * dx - sin_ny * dy;
            double goal_ly = sin_ny * dx + cos_ny * dy;

            double half = costmap_size_ / 2.0;
            double margin = costmap_resolution_ * 2;
            double plan_lx = goal_lx, plan_ly = goal_ly;

            if (std::abs(goal_lx) >= half - margin || std::abs(goal_ly) >= half - margin) {
                double angle = std::atan2(goal_ly, goal_lx);
                double bnd = half - margin;
                plan_lx = bnd * std::cos(angle);
                plan_ly = bnd * std::sin(angle);
            }

            int start_r = grid_cells_ / 2, start_c = grid_cells_ / 2;
            int goal_r = std::clamp(static_cast<int>((plan_ly + half) / costmap_resolution_), 1, grid_cells_ - 2);
            int goal_c = std::clamp(static_cast<int>((plan_lx + half) / costmap_resolution_), 1, grid_cells_ - 2);

            auto grid_path = astar_plan(costmap, {start_r, start_c}, {goal_r, goal_c});
            if (grid_path.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "A* no path — stop");
                publish_stop();
                return;
            }

            // Grid → odom
            double cos_y = std::cos(robot_yaw);
            double sin_y = std::sin(robot_yaw);
            cached_global_path_odom_.clear();
            cached_global_path_odom_.reserve(grid_path.size());
            for (const auto &cell : grid_path) {
                double lx = (cell.c + 0.5) * costmap_resolution_ - half;
                double ly = (cell.r + 0.5) * costmap_resolution_ - half;
                cached_global_path_odom_.emplace_back(
                    cos_y * lx - sin_y * ly + robot_x,
                    sin_y * lx + cos_y * ly + robot_y);
            }
            last_replan_time_ = get_clock()->now();
        }

        // 6. Publish global path
        if (!cached_global_path_odom_.empty())
            publish_global_path_odom(cached_global_path_odom_, now());

        // 7. Pure Pursuit target
        double pursuit_lx, pursuit_ly;
        if (!find_pure_pursuit_target(robot_x, robot_y, robot_yaw, pursuit_lx, pursuit_ly)) {
            publish_stop();
            return;
        }

        // 8. Local path for DWA
        double cos_ny = std::cos(-robot_yaw);
        double sin_ny = std::sin(-robot_yaw);
        std::vector<std::pair<double, double>> local_path;
        local_path.reserve(cached_global_path_odom_.size());
        for (const auto &[ox, oy] : cached_global_path_odom_) {
            local_path.emplace_back(
                cos_ny * (ox - robot_x) - sin_ny * (oy - robot_y),
                sin_ny * (ox - robot_x) + cos_ny * (oy - robot_y));
        }

        // 9. DWA
        auto [best_v, best_w, best_traj] = dwa_plan(costmap, local_path, pursuit_lx, pursuit_ly);

        publish_local_trajectory(best_traj, now(), robot_x, robot_y, robot_yaw);

        // 10. Command
        publish_cmd(best_v, best_w);
    }

    // =====================================================================
    // A* Global Planner
    // =====================================================================
    std::vector<Cell> astar_plan(
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &costmap,
        Cell start, Cell goal)
    {
        const int rows = costmap->info.height;
        const int cols = costmap->info.width;
        const auto &data = costmap->data;

        auto get_cost = [&](int r, int c) -> int {
            if (r < 0 || r >= rows || c < 0 || c >= cols) return 127;
            return data[r * cols + c];
        };

        if (get_cost(goal.r, goal.c) >= static_cast<int>(astar_lethal_)) {
            goal = find_nearest_free(costmap, goal);
            if (goal.r < 0) return {};
        }

        std::priority_queue<AStarEntry, std::vector<AStarEntry>,
                           std::greater<AStarEntry>> open;
        std::unordered_map<Cell, double, CellHash> g_score;
        std::unordered_map<Cell, Cell, CellHash> came_from;

        g_score[start] = 0.0;
        open.push({heuristic(start, goal), start});

        static const int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        static const int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static const double move_cost[] = {
            M_SQRT2, 1.0, M_SQRT2, 1.0, 1.0, M_SQRT2, 1.0, M_SQRT2
        };

        while (!open.empty()) {
            auto [f, current] = open.top();
            open.pop();

            if (current == goal)
                return reconstruct_path(came_from, start, goal);

            auto it = g_score.find(current);
            if (it != g_score.end() && f - heuristic(current, goal) > it->second + 1e-6)
                continue;

            for (int i = 0; i < 8; ++i) {
                Cell nb = {current.r + dr[i], current.c + dc[i]};
                if (nb.r < 0 || nb.r >= rows || nb.c < 0 || nb.c >= cols) continue;

                int cc = get_cost(nb.r, nb.c);
                if (cc < 0 || cc >= static_cast<int>(astar_lethal_)) continue;

                double penalty = static_cast<double>(cc) / 100.0 * astar_cost_weight_;
                double tentative = g_score[current] + move_cost[i] * (1.0 + penalty);

                auto git = g_score.find(nb);
                if (git == g_score.end() || tentative < git->second) {
                    g_score[nb] = tentative;
                    came_from[nb] = current;
                    open.push({tentative + heuristic(nb, goal), nb});
                }
            }
        }
        return {};
    }

    double heuristic(const Cell &a, const Cell &b) const
    {
        int ddx = std::abs(a.c - b.c), ddy = std::abs(a.r - b.r);
        return std::max(ddx, ddy) + (M_SQRT2 - 1.0) * std::min(ddx, ddy);
    }

    std::vector<Cell> reconstruct_path(
        const std::unordered_map<Cell, Cell, CellHash> &came_from,
        const Cell &start, const Cell &goal)
    {
        std::vector<Cell> path;
        Cell cur = goal;
        while (!(cur == start)) {
            path.push_back(cur);
            auto it = came_from.find(cur);
            if (it == came_from.end()) break;
            cur = it->second;
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    Cell find_nearest_free(
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &costmap,
        const Cell &target)
    {
        const int rows = costmap->info.height;
        const int cols = costmap->info.width;
        const auto &data = costmap->data;

        for (int rad = 1; rad < std::max(rows, cols) / 2; ++rad) {
            for (int dr = -rad; dr <= rad; ++dr) {
                for (int dc = -rad; dc <= rad; ++dc) {
                    if (std::abs(dr) != rad && std::abs(dc) != rad) continue;
                    int nr = target.r + dr, nc = target.c + dc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        int c = data[nr * cols + nc];
                        if (c >= 0 && c < static_cast<int>(astar_lethal_))
                            return {nr, nc};
                    }
                }
            }
        }
        return {-1, -1};
    }

    // =====================================================================
    // DWA Local Planner  [v3: anti-spin + rectangular footprint]
    // =====================================================================
    struct DWAResult {
        double v, w;
        std::vector<std::pair<double, double>> trajectory;
    };

    DWAResult dwa_plan(
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &costmap,
        const std::vector<std::pair<double, double>> &global_path,
        double goal_local_x, double goal_local_y)
    {
        double dt = 1.0 / control_rate_;
        double v_min = std::max(min_lin_vel_, cur_v_ - max_lin_acc_ * dt);
        double v_max = std::min(max_lin_vel_, cur_v_ + max_lin_acc_ * dt);
        double w_min = std::max(-max_ang_vel_, cur_w_ - max_ang_acc_ * dt);
        double w_max = std::min(max_ang_vel_, cur_w_ + max_ang_acc_ * dt);

        const int sim_steps = static_cast<int>(std::round(sim_time_ / sim_granularity_));
        const double half = costmap_size_ / 2.0;
        const int lethal = static_cast<int>(astar_lethal_);

        // [v3] Heading error to pure pursuit target (for spin decision)
        double heading_to_goal = std::atan2(goal_local_y, goal_local_x);
        double abs_heading_err = std::abs(heading_to_goal);

        // [v3] Rectangular footprint sample points (robot frame)
        // Pre-compute footprint cells to check at each simulation step
        double half_l = robot_length_ / 2.0 + clearance_margin_;
        double half_w = robot_width_ / 2.0 + clearance_margin_;
        struct FPPoint { double x, y; };
        std::vector<FPPoint> footprint_pts;
        for (double fx = -half_l; fx <= half_l; fx += costmap_resolution_) {
            for (double fy = -half_w; fy <= half_w; fy += costmap_resolution_) {
                footprint_pts.push_back({fx, fy});
            }
        }

        double best_score = -std::numeric_limits<double>::infinity();
        double best_v = 0.0, best_w = 0.0;
        std::vector<std::pair<double, double>> best_traj;

        for (int vi = 0; vi < n_v_samples_; ++vi) {
            double v = v_min + (v_max - v_min) * vi / std::max(n_v_samples_ - 1, 1);

            for (int wi = 0; wi < n_w_samples_; ++wi) {
                double w = w_min + (w_max - w_min) * wi / std::max(n_w_samples_ - 1, 1);

                double x = 0.0, y = 0.0, theta = 0.0;
                bool collision = false;
                double min_clearance = std::numeric_limits<double>::max();
                std::vector<std::pair<double, double>> traj;
                traj.reserve(sim_steps);

                for (int s = 0; s < sim_steps; ++s) {
                    x += v * std::cos(theta) * sim_granularity_;
                    y += v * std::sin(theta) * sim_granularity_;
                    theta += w * sim_granularity_;
                    traj.emplace_back(x, y);

                    // Center cell check
                    int gc = static_cast<int>((x + half) / costmap_resolution_);
                    int gr = static_cast<int>((y + half) / costmap_resolution_);
                    if (gc < 0 || gc >= grid_cells_ || gr < 0 || gr >= grid_cells_) {
                        min_clearance = std::min(min_clearance, 0.1);
                        continue;
                    }
                    int cc_val = costmap->data[gr * grid_cells_ + gc];
                    if (cc_val >= lethal) { collision = true; break; }

                    // [v3] Oriented rectangular footprint check
                    double cos_t = std::cos(theta);
                    double sin_t = std::sin(theta);
                    bool fp_hit = false;
                    for (const auto &fp : footprint_pts) {
                        double wx = x + cos_t * fp.x - sin_t * fp.y;
                        double wy = y + sin_t * fp.x + cos_t * fp.y;
                        int wc = static_cast<int>((wx + half) / costmap_resolution_);
                        int wr = static_cast<int>((wy + half) / costmap_resolution_);
                        if (wc >= 0 && wc < grid_cells_ && wr >= 0 && wr < grid_cells_) {
                            if (costmap->data[wr * grid_cells_ + wc] >= lethal) {
                                fp_hit = true;
                                break;
                            }
                        }
                    }
                    if (fp_hit) { collision = true; break; }

                    // Terrain clearance
                    if (cc_val >= 0) {
                        double cl = 1.0 - static_cast<double>(cc_val) / 100.0;
                        min_clearance = std::min(min_clearance, cl);
                    }
                }

                if (collision) continue;

                // ── Scoring [v3] ──

                // (a) Heading: cosine-based (gentle for small errors)
                double ang = std::atan2(goal_local_y - y, goal_local_x - x);
                double herr = std::abs(normalize_angle(ang - theta));
                double heading_score = (1.0 + std::cos(herr)) / 2.0;  // 1.0 at 0, 0.0 at π

                // (b) Clearance
                double clearance_score = (min_clearance < std::numeric_limits<double>::max())
                    ? min_clearance : 1.0;

                // (c) Velocity: strong forward preference
                double velocity_score = 0.0;
                if (max_lin_vel_ > 0.0) {
                    velocity_score = v / max_lin_vel_;
                    if (v < 0.0) velocity_score *= 0.3;
                }

                // (d) Path distance
                double path_dist_score = 1.0;
                if (!global_path.empty()) {
                    double min_d = std::numeric_limits<double>::max();
                    for (const auto &wp : global_path) {
                        double d = std::hypot(x - wp.first, y - wp.second);
                        min_d = std::min(min_d, d);
                    }
                    path_dist_score = std::max(0.0, 1.0 - min_d);
                }

                // [v3] (e) Spin penalty: discourage v≈0 rotation when heading < 60°
                double spin_penalty = 0.0;
                if (abs_heading_err < spin_heading_threshold_) {
                    if (std::abs(v) < 0.05 && std::abs(w) > 0.1) {
                        spin_penalty = -0.5;
                    }
                }

                double score = w_heading_ * heading_score
                             + w_clearance_ * clearance_score
                             + w_velocity_ * velocity_score
                             + w_path_dist_ * path_dist_score
                             + spin_penalty;

                if (score > best_score) {
                    best_score = score;
                    best_v = v;
                    best_w = w;
                    best_traj = traj;
                }
            }
        }

        if (best_score <= -std::numeric_limits<double>::max() + 1.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "DWA: no valid trajectory — emergency stop");
            return {0.0, 0.0, {}};
        }

        return {best_v, best_w, best_traj};
    }

    // =====================================================================
    // TF helpers
    // =====================================================================
    bool get_robot_pose(double &x, double &y, double &yaw)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(
                odom_frame_, robot_frame_, tf2::TimePointZero,
                tf2::durationFromSec(0.1));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            yaw = get_yaw_from_tf(tf.transform.rotation);
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    static double get_yaw_from_tf(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tq(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(tq).getRPY(r, p, y);
        return y;
    }

    static double get_yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
    {
        return get_yaw_from_tf(q);
    }

    // =====================================================================
    // Publishing
    // =====================================================================
    void publish_cmd(double v, double w)
    {
        const double alpha = 0.4;
        cur_v_ += alpha * (v - cur_v_);
        cur_w_ += alpha * (w - cur_w_);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = cur_v_;
        cmd.angular.z = cur_w_;
        pub_cmd_->publish(cmd);
    }

    void publish_stop()
    {
        cur_v_ = 0.0;
        cur_w_ = 0.0;
        geometry_msgs::msg::Twist cmd;
        pub_cmd_->publish(cmd);
    }

    void publish_global_path_odom(
        const std::vector<std::pair<double, double>> &waypoints,
        const rclcpp::Time &stamp)
    {
        if (pub_global_path_->get_subscription_count() == 0) return;
        nav_msgs::msg::Path msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;
        for (const auto &[ox, oy] : waypoints) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = ox;
            ps.pose.position.y = oy;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        pub_global_path_->publish(msg);
    }

    void publish_local_trajectory(
        const std::vector<std::pair<double, double>> &traj,
        const rclcpp::Time &stamp,
        double rx, double ry, double ryaw)
    {
        if (pub_local_traj_->get_subscription_count() == 0 || traj.empty()) return;
        nav_msgs::msg::Path msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;
        double cy = std::cos(ryaw), sy = std::sin(ryaw);
        for (const auto &[lx, ly] : traj) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = cy * lx - sy * ly + rx;
            ps.pose.position.y = sy * lx + cy * ly + ry;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        pub_local_traj_->publish(msg);
    }

    // =====================================================================
    // Member variables
    // =====================================================================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_traj_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;
    std::mutex costmap_mutex_;
    geometry_msgs::msg::PoseStamped goal_;
    bool goal_active_ = false;
    double cur_v_ = 0.0, cur_w_ = 0.0;

    // Path caching
    std::vector<std::pair<double, double>> cached_global_path_odom_;
    rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
    bool need_replan_ = true;

    // Parameters
    double costmap_size_, costmap_resolution_;
    int grid_cells_;
    double max_lin_vel_, min_lin_vel_, max_ang_vel_;
    double max_lin_acc_, max_ang_acc_;
    int n_v_samples_, n_w_samples_;
    double sim_time_, sim_granularity_;
    double w_heading_, w_clearance_, w_velocity_, w_path_dist_;
    double goal_xy_tol_, goal_yaw_tol_;
    double astar_lethal_, astar_cost_weight_;
    double robot_length_, robot_width_;
    double clearance_margin_;
    double control_rate_;
    std::string odom_frame_, robot_frame_;
    double slope_speed_factor_, slope_threshold_deg_;
    double replan_interval_, path_deviation_threshold_;
    double pure_pursuit_lookahead_, pure_pursuit_lookahead_min_, pure_pursuit_lookahead_max_;
    double velocity_lookahead_gain_;
    double spin_heading_threshold_;
};

// =========================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::on_shutdown([node]() { node->send_stop_command(); });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}