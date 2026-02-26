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
// Behavior:
//   1. Receive goal in odom frame
//   2. A* plans on costmap (robot-centric grid → global waypoints)
//   3. Global path cached in odom frame; replan only on new goal / deviation / periodic
//   4. Pure Pursuit selects lookahead on stable cached path
//   5. DWA selects (v, ω) toward pure pursuit target while avoiding obstacles
//   6. Large heading error → DWA naturally selects turn-in-place (v≈0)
//   7. Goal outside costmap → intermediate waypoint at costmap boundary
//   8. No valid trajectory → emergency stop (v=0, ω=0)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
// DWA trajectory candidate
// =========================================================================
struct Trajectory {
    double v, w;           // linear, angular velocity
    double score;          // evaluation score
    std::vector<std::pair<double, double>> points;  // (x, y) for visualization
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
            "Path caching: replan_interval=%.1fs, deviation_threshold=%.2fm, "
            "pure_pursuit_lookahead=%.2fm",
            replan_interval_, path_deviation_threshold_, pure_pursuit_lookahead_);
    }

    // ── 종료 시 반드시 정지 명령 발행 ──
    ~PathPlannerNode() override
    {
        send_stop_command();
    }

    void send_stop_command()
    {
        RCLCPP_INFO(get_logger(), "Shutdown: sending stop command to /cmd_vel");
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        // 여러 번 발행하여 확실히 수신되도록 함
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
        // Costmap (must match terrain_costmap_node)
        declare_parameter<double>("costmap_size", 5.0);
        declare_parameter<double>("costmap_resolution", 0.1);

        // Robot kinematics (DONKEYBOTI.yaml)
        declare_parameter<double>("max_lin_vel", 0.6);
        declare_parameter<double>("min_lin_vel", -0.2);     // limited reverse
        declare_parameter<double>("max_ang_vel", 0.5);
        declare_parameter<double>("max_lin_acc", 1.0);
        declare_parameter<double>("max_ang_acc", 2.0);

        // DWA parameters
        declare_parameter<int>("n_v_samples", 11);
        declare_parameter<int>("n_w_samples", 21);          // more ω for turn-in-place
        declare_parameter<double>("sim_time", 1.5);         // forward simulation [s]
        declare_parameter<double>("sim_granularity", 0.1);  // simulation step [s]

        // DWA scoring weights
        declare_parameter<double>("weight_heading", 1.0);   // alignment to A* path
        declare_parameter<double>("weight_clearance", 0.5); // obstacle distance
        declare_parameter<double>("weight_velocity", 0.3);  // prefer forward motion
        declare_parameter<double>("weight_path_dist", 0.8); // distance to A* path

        // Goal tolerance
        declare_parameter<double>("goal_xy_tolerance", 0.2);    // [m]
        declare_parameter<double>("goal_yaw_tolerance", 0.15);  // [rad] ~8.6°

        // A* configuration
        declare_parameter<double>("astar_lethal_cost", 80.0);   // cells >= this are blocked
        declare_parameter<double>("astar_cost_weight", 2.0);    // terrain cost penalty

        // Safety
        declare_parameter<double>("robot_radius", 0.35);    // [m] collision check radius
        declare_parameter<double>("clearance_margin", 0.1); // [m] extra safety margin

        // Control rate
        declare_parameter<double>("control_rate", 20.0);    // [Hz]

        // Frames
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("robot_frame", "base_link");

        // Topics
        declare_parameter<std::string>("costmap_topic", "/terrain_costmap");
        declare_parameter<std::string>("goal_topic", "/goal_pose");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        // Slope speed reduction (IMU-based, future extension)
        declare_parameter<double>("slope_speed_factor", 0.5);  // reduce max_v on slope
        declare_parameter<double>("slope_threshold_deg", 15.0);

        // === NEW: Path caching & Pure Pursuit ===
        declare_parameter<double>("replan_interval", 1.0);             // [s] periodic A* replan
        declare_parameter<double>("path_deviation_threshold", 0.5);    // [m] replan if robot deviates
        declare_parameter<double>("pure_pursuit_lookahead", 0.4);      // [m] pure pursuit lookahead distance
        declare_parameter<double>("pure_pursuit_lookahead_min", 0.2);  // [m] minimum lookahead
        declare_parameter<double>("pure_pursuit_lookahead_max", 0.8);  // [m] maximum lookahead
        declare_parameter<double>("velocity_lookahead_gain", 0.5);     // lookahead scales with speed
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

        robot_radius_ = get_parameter("robot_radius").as_double();
        clearance_margin_ = get_parameter("clearance_margin").as_double();

        control_rate_ = get_parameter("control_rate").as_double();

        odom_frame_ = get_parameter("odom_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();

        slope_speed_factor_ = get_parameter("slope_speed_factor").as_double();
        slope_threshold_deg_ = get_parameter("slope_threshold_deg").as_double();

        // NEW
        replan_interval_ = get_parameter("replan_interval").as_double();
        path_deviation_threshold_ = get_parameter("path_deviation_threshold").as_double();
        pure_pursuit_lookahead_ = get_parameter("pure_pursuit_lookahead").as_double();
        pure_pursuit_lookahead_min_ = get_parameter("pure_pursuit_lookahead_min").as_double();
        pure_pursuit_lookahead_max_ = get_parameter("pure_pursuit_lookahead_max").as_double();
        velocity_lookahead_gain_ = get_parameter("velocity_lookahead_gain").as_double();
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

    void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        goal_ = *msg;
        goal_active_ = true;
        need_replan_ = true;  // NEW: force replan on new goal
        RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f) in frame '%s'",
            msg->pose.position.x, msg->pose.position.y,
            msg->header.frame_id.c_str());
    }

    // =====================================================================
    // Check if replanning is needed
    // =====================================================================
    bool should_replan(double robot_x, double robot_y)
    {
        // (1) Forced replan (new goal)
        if (need_replan_) {
            need_replan_ = false;
            return true;
        }

        // (2) No cached path
        if (cached_global_path_odom_.empty()) {
            return true;
        }

        // (3) Periodic replan
        auto now = get_clock()->now();
        double elapsed = (now - last_replan_time_).seconds();
        if (elapsed >= replan_interval_) {
            return true;
        }

        // (4) Robot deviated too far from cached path
        double min_dist = std::numeric_limits<double>::max();
        for (const auto &wp : cached_global_path_odom_) {
            double d = std::hypot(robot_x - wp.first, robot_y - wp.second);
            min_dist = std::min(min_dist, d);
        }
        if (min_dist > path_deviation_threshold_) {
            RCLCPP_INFO(get_logger(),
                "Path deviation %.2fm > threshold %.2fm — replanning",
                min_dist, path_deviation_threshold_);
            return true;
        }

        return false;
    }

    // =====================================================================
    // Pure Pursuit: find lookahead point on cached odom-frame path
    // Returns lookahead in robot-local coordinates
    // =====================================================================
    bool find_pure_pursuit_target(
        double robot_x, double robot_y, double robot_yaw,
        double &target_local_x, double &target_local_y)
    {
        if (cached_global_path_odom_.empty()) return false;

        // Adaptive lookahead: scales with velocity
        double lookahead = pure_pursuit_lookahead_
                         + velocity_lookahead_gain_ * std::abs(cur_v_);
        lookahead = std::clamp(lookahead,
                              pure_pursuit_lookahead_min_,
                              pure_pursuit_lookahead_max_);

        // Find the closest point on path to robot
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

        // Walk forward along path from closest point to find lookahead target
        double accum_dist = 0.0;
        size_t target_idx = closest_idx;
        for (size_t i = closest_idx; i + 1 < cached_global_path_odom_.size(); ++i) {
            double dx = cached_global_path_odom_[i+1].first - cached_global_path_odom_[i].first;
            double dy = cached_global_path_odom_[i+1].second - cached_global_path_odom_[i].second;
            accum_dist += std::hypot(dx, dy);
            target_idx = i + 1;
            if (accum_dist >= lookahead) break;
        }

        // Transform target to robot-local frame
        double tx_odom = cached_global_path_odom_[target_idx].first;
        double ty_odom = cached_global_path_odom_[target_idx].second;
        double dx = tx_odom - robot_x;
        double dy = ty_odom - robot_y;
        double cos_y = std::cos(-robot_yaw);
        double sin_y = std::sin(-robot_yaw);
        target_local_x = cos_y * dx - sin_y * dy;
        target_local_y = sin_y * dx + cos_y * dy;

        return true;
    }

    // =====================================================================
    // Main control loop (20 Hz)
    // =====================================================================
    void control_loop()
    {
        if (!goal_active_) return;

        // 1. Get latest costmap
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap = costmap_;
        }
        if (!costmap) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "No costmap received yet");
            return;
        }

        // 2. Get robot pose in odom frame
        double robot_x, robot_y, robot_yaw;
        if (!get_robot_pose(robot_x, robot_y, robot_yaw))
            return;

        // 3. Compute goal in robot frame
        double goal_x_odom = goal_.pose.position.x;
        double goal_y_odom = goal_.pose.position.y;
        double goal_yaw = get_yaw_from_quat(goal_.pose.orientation);

        double dx = goal_x_odom - robot_x;
        double dy = goal_y_odom - robot_y;
        double dist_to_goal = std::hypot(dx, dy);

        // 4. Check if goal reached
        if (dist_to_goal < goal_xy_tol_) {
            double yaw_err = std::abs(normalize_angle(goal_yaw - robot_yaw));
            if (yaw_err < goal_yaw_tol_) {
                // Goal reached
                publish_stop();
                goal_active_ = false;
                cached_global_path_odom_.clear();
                RCLCPP_INFO(get_logger(), "Goal reached!");
                return;
            } else {
                // Position reached, align heading
                double w = std::clamp(
                    normalize_angle(goal_yaw - robot_yaw) * 1.5,
                    -max_ang_vel_, max_ang_vel_);
                publish_cmd(0.0, w);
                return;
            }
        }

        // ============================================================
        // 5. A* REPLAN ONLY WHEN NEEDED (not every cycle!)
        // ============================================================
        if (should_replan(robot_x, robot_y)) {
            // Transform goal to robot-local costmap coordinates
            double goal_local_x = std::cos(-robot_yaw) * dx - std::sin(-robot_yaw) * dy;
            double goal_local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;

            // Project goal to costmap boundary if outside
            double half = costmap_size_ / 2.0;
            double margin = costmap_resolution_ * 2;
            bool goal_in_costmap = (std::abs(goal_local_x) < half - margin &&
                                    std::abs(goal_local_y) < half - margin);

            double plan_local_x = goal_local_x;
            double plan_local_y = goal_local_y;
            if (!goal_in_costmap) {
                double angle = std::atan2(goal_local_y, goal_local_x);
                double boundary = half - margin;
                plan_local_x = boundary * std::cos(angle);
                plan_local_y = boundary * std::sin(angle);
            }

            // Convert local coords to grid indices
            int start_r = grid_cells_ / 2;
            int start_c = grid_cells_ / 2;
            int goal_r = static_cast<int>((plan_local_y + half) / costmap_resolution_);
            int goal_c = static_cast<int>((plan_local_x + half) / costmap_resolution_);

            goal_r = std::clamp(goal_r, 1, grid_cells_ - 2);
            goal_c = std::clamp(goal_c, 1, grid_cells_ - 2);

            // A* global path
            auto grid_path = astar_plan(costmap, {start_r, start_c}, {goal_r, goal_c});

            if (grid_path.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "A* found no path — stopping");
                publish_stop();
                return;
            }

            // Convert grid path to local coordinates
            std::vector<std::pair<double, double>> waypoints_local;
            waypoints_local.reserve(grid_path.size());
            for (const auto &cell : grid_path) {
                double lx = (cell.c + 0.5) * costmap_resolution_ - half;
                double ly = (cell.r + 0.5) * costmap_resolution_ - half;
                waypoints_local.emplace_back(lx, ly);
            }

            // *** Convert to odom frame and cache ***
            double cos_y = std::cos(robot_yaw);
            double sin_y = std::sin(robot_yaw);
            cached_global_path_odom_.clear();
            cached_global_path_odom_.reserve(waypoints_local.size());
            for (const auto &[lx, ly] : waypoints_local) {
                double ox = cos_y * lx - sin_y * ly + robot_x;
                double oy = sin_y * lx + cos_y * ly + robot_y;
                cached_global_path_odom_.emplace_back(ox, oy);
            }

            last_replan_time_ = get_clock()->now();

            RCLCPP_DEBUG(get_logger(), "A* replanned: %zu waypoints",
                cached_global_path_odom_.size());
        }

        // 6. Publish cached global path for visualization
        if (!cached_global_path_odom_.empty()) {
            publish_global_path_odom(cached_global_path_odom_, now());
        }

        // ============================================================
        // 7. Pure Pursuit: find stable lookahead on cached path
        // ============================================================
        double pursuit_local_x, pursuit_local_y;
        if (!find_pure_pursuit_target(robot_x, robot_y, robot_yaw,
                                       pursuit_local_x, pursuit_local_y))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "No pure pursuit target — stopping");
            publish_stop();
            return;
        }

        // 8. Convert cached path to robot-local for DWA collision checking
        double cos_neg_yaw = std::cos(-robot_yaw);
        double sin_neg_yaw = std::sin(-robot_yaw);
        std::vector<std::pair<double, double>> local_path;
        local_path.reserve(cached_global_path_odom_.size());
        for (const auto &[ox, oy] : cached_global_path_odom_) {
            double lx = cos_neg_yaw * (ox - robot_x) - sin_neg_yaw * (oy - robot_y);
            double ly = sin_neg_yaw * (ox - robot_x) + cos_neg_yaw * (oy - robot_y);
            local_path.emplace_back(lx, ly);
        }

        // 9. DWA local planning (using pure pursuit target instead of raw goal)
        auto [best_v, best_w, best_traj] = dwa_plan(costmap, local_path,
            pursuit_local_x, pursuit_local_y);

        // Publish local trajectory for visualization
        publish_local_trajectory(best_traj, now(),
            robot_x, robot_y, robot_yaw);

        // 10. Publish command
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

        // Validate start/goal
        auto get_cost = [&](int r, int c) -> int {
            if (r < 0 || r >= rows || c < 0 || c >= cols) return 127;
            return data[r * cols + c];
        };

        if (get_cost(goal.r, goal.c) >= static_cast<int>(astar_lethal_)) {
            // Goal cell is blocked — find nearest free cell
            goal = find_nearest_free(costmap, goal);
            if (goal.r < 0) return {};
        }

        // Priority queue
        std::priority_queue<AStarEntry, std::vector<AStarEntry>,
                           std::greater<AStarEntry>> open;
        std::unordered_map<Cell, double, CellHash> g_score;
        std::unordered_map<Cell, Cell, CellHash> came_from;

        g_score[start] = 0.0;
        open.push({heuristic(start, goal), start});

        // 8-connected neighbors
        static const int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        static const int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static const double move_cost[] = {
            M_SQRT2, 1.0, M_SQRT2, 1.0, 1.0, M_SQRT2, 1.0, M_SQRT2
        };

        while (!open.empty()) {
            auto [f, current] = open.top();
            open.pop();

            if (current == goal) {
                return reconstruct_path(came_from, start, goal);
            }

            // Skip if we already found a better path
            auto it = g_score.find(current);
            if (it != g_score.end() && f - heuristic(current, goal) > it->second + 1e-6)
                continue;

            for (int i = 0; i < 8; ++i) {
                Cell nb = {current.r + dr[i], current.c + dc[i]};
                if (nb.r < 0 || nb.r >= rows || nb.c < 0 || nb.c >= cols)
                    continue;

                int cell_cost = get_cost(nb.r, nb.c);
                if (cell_cost < 0 || cell_cost >= static_cast<int>(astar_lethal_))
                    continue;  // unknown(-1) or lethal

                // Terrain-aware cost: higher cost cells are more expensive
                double terrain_penalty = static_cast<double>(cell_cost) / 100.0
                                       * astar_cost_weight_;
                double tentative = g_score[current]
                                 + move_cost[i] * (1.0 + terrain_penalty);

                auto git = g_score.find(nb);
                if (git == g_score.end() || tentative < git->second) {
                    g_score[nb] = tentative;
                    came_from[nb] = current;
                    open.push({tentative + heuristic(nb, goal), nb});
                }
            }
        }

        return {};  // No path found
    }

    double heuristic(const Cell &a, const Cell &b) const
    {
        // Octile distance
        int dx = std::abs(a.c - b.c);
        int dy = std::abs(a.r - b.r);
        return std::max(dx, dy) + (M_SQRT2 - 1.0) * std::min(dx, dy);
    }

    std::vector<Cell> reconstruct_path(
        const std::unordered_map<Cell, Cell, CellHash> &came_from,
        const Cell &start, const Cell &goal)
    {
        std::vector<Cell> path;
        Cell current = goal;
        while (!(current == start)) {
            path.push_back(current);
            auto it = came_from.find(current);
            if (it == came_from.end()) break;
            current = it->second;
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

        // BFS spiral outward from target
        for (int radius = 1; radius < std::max(rows, cols) / 2; ++radius) {
            for (int dr = -radius; dr <= radius; ++dr) {
                for (int dc = -radius; dc <= radius; ++dc) {
                    if (std::abs(dr) != radius && std::abs(dc) != radius)
                        continue;
                    int nr = target.r + dr;
                    int nc = target.c + dc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        int cost = data[nr * cols + nc];
                        if (cost >= 0 && cost < static_cast<int>(astar_lethal_))
                            return {nr, nc};
                    }
                }
            }
        }
        return {-1, -1};
    }

    // =====================================================================
    // DWA Local Planner
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
        // Dynamic window from current velocity
        double dt = 1.0 / control_rate_;
        double v_min = std::max(min_lin_vel_, cur_v_ - max_lin_acc_ * dt);
        double v_max = std::min(max_lin_vel_, cur_v_ + max_lin_acc_ * dt);
        double w_min = std::max(-max_ang_vel_, cur_w_ - max_ang_acc_ * dt);
        double w_max = std::min(max_ang_vel_, cur_w_ + max_ang_acc_ * dt);

        const int sim_steps = static_cast<int>(std::round(sim_time_ / sim_granularity_));
        const double half = costmap_size_ / 2.0;

        double best_score = -std::numeric_limits<double>::infinity();
        double best_v = 0.0, best_w = 0.0;
        std::vector<std::pair<double, double>> best_traj;

        for (int vi = 0; vi < n_v_samples_; ++vi) {
            double v = v_min + (v_max - v_min) * vi / std::max(n_v_samples_ - 1, 1);

            for (int wi = 0; wi < n_w_samples_; ++wi) {
                double w = w_min + (w_max - w_min) * wi / std::max(n_w_samples_ - 1, 1);

                // Forward simulate trajectory
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

                    // Collision check in costmap
                    int gc = static_cast<int>((x + half) / costmap_resolution_);
                    int gr = static_cast<int>((y + half) / costmap_resolution_);

                    if (gc < 0 || gc >= grid_cells_ || gr < 0 || gr >= grid_cells_) {
                        // Outside costmap — penalize but don't block
                        min_clearance = std::min(min_clearance, 0.1);
                        continue;
                    }

                    int cell_cost = costmap->data[gr * grid_cells_ + gc];
                    if (cell_cost >= static_cast<int>(astar_lethal_)) {
                        collision = true;
                        break;
                    }

                    // Robot radius collision check
                    int radius_cells = static_cast<int>(
                        std::ceil((robot_radius_ + clearance_margin_) / costmap_resolution_));
                    bool radius_collision = false;
                    for (int dr = -radius_cells; dr <= radius_cells && !radius_collision; ++dr) {
                        for (int dc = -radius_cells; dc <= radius_cells && !radius_collision; ++dc) {
                            if (dr * dr + dc * dc > radius_cells * radius_cells)
                                continue;
                            int cr = gr + dr;
                            int cc = gc + dc;
                            if (cr >= 0 && cr < grid_cells_ && cc >= 0 && cc < grid_cells_) {
                                if (costmap->data[cr * grid_cells_ + cc] >= static_cast<int>(astar_lethal_)) {
                                    radius_collision = true;
                                }
                            }
                        }
                    }

                    if (radius_collision) {
                        collision = true;
                        break;
                    }

                    // Clearance as inverse of terrain cost
                    if (cell_cost >= 0) {
                        double cl = 1.0 - static_cast<double>(cell_cost) / 100.0;
                        min_clearance = std::min(min_clearance, cl);
                    }
                }

                if (collision) continue;

                // === Scoring ===
                // (a) Heading: alignment to pure pursuit target from trajectory end
                double angle_to_goal = std::atan2(goal_local_y - y, goal_local_x - x);
                double heading_err = std::abs(normalize_angle(angle_to_goal - theta));
                double heading_score = 1.0 - heading_err / M_PI;

                // (b) Clearance
                double clearance_score = (min_clearance < std::numeric_limits<double>::max())
                    ? min_clearance : 1.0;

                // (c) Velocity: prefer forward motion
                double velocity_score = 0.0;
                if (max_lin_vel_ > 0.0) {
                    velocity_score = v / max_lin_vel_;
                    // Slight penalty for pure reverse
                    if (v < 0.0) velocity_score *= 0.5;
                }

                // (d) Path distance: closeness to global A* path
                double path_dist_score = 1.0;
                if (!global_path.empty()) {
                    double min_dist = std::numeric_limits<double>::max();
                    for (const auto &wp : global_path) {
                        double d = std::hypot(x - wp.first, y - wp.second);
                        min_dist = std::min(min_dist, d);
                    }
                    // Normalize: 0 at 1m distance, 1 at 0m
                    path_dist_score = std::max(0.0, 1.0 - min_dist);
                }

                double score = w_heading_ * heading_score
                             + w_clearance_ * clearance_score
                             + w_velocity_ * velocity_score
                             + w_path_dist_ * path_dist_score;

                if (score > best_score) {
                    best_score = score;
                    best_v = v;
                    best_w = w;
                    best_traj = traj;
                }
            }
        }

        // No valid trajectory found → emergency stop
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
        // ── 속도 스무딩 (지수 이동 평균) ──
        // DWA가 매 사이클 다른 (v,w)를 선택해도 급격한 변화를 완화
        const double alpha = 0.4;  // 0.0~1.0, 작을수록 부드러움
        double smoothed_v = cur_v_ + alpha * (v - cur_v_);
        double smoothed_w = cur_w_ + alpha * (w - cur_w_);

        cur_v_ = smoothed_v;
        cur_w_ = smoothed_w;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = smoothed_v;
        cmd.angular.z = smoothed_w;
        pub_cmd_->publish(cmd);
    }

    void publish_stop()
    {
        // 긴급 정지는 스무딩 없이 즉시 정지
        cur_v_ = 0.0;
        cur_w_ = 0.0;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd_->publish(cmd);
    }

    // Publish cached odom-frame path directly (no per-cycle coordinate transform)
    void publish_global_path_odom(
        const std::vector<std::pair<double, double>> &waypoints_odom,
        const rclcpp::Time &stamp)
    {
        if (pub_global_path_->get_subscription_count() == 0) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = odom_frame_;

        for (const auto &[ox, oy] : waypoints_odom) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = ox;
            ps.pose.position.y = oy;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }

        pub_global_path_->publish(path_msg);
    }

    void publish_local_trajectory(
        const std::vector<std::pair<double, double>> &traj,
        const rclcpp::Time &stamp,
        double robot_x, double robot_y, double robot_yaw)
    {
        if (pub_local_traj_->get_subscription_count() == 0) return;
        if (traj.empty()) return;

        nav_msgs::msg::Path traj_msg;
        traj_msg.header.stamp = stamp;
        traj_msg.header.frame_id = odom_frame_;

        double cos_y = std::cos(robot_yaw);
        double sin_y = std::sin(robot_yaw);

        for (const auto &[lx, ly] : traj) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = traj_msg.header;
            ps.pose.position.x = cos_y * lx - sin_y * ly + robot_x;
            ps.pose.position.y = sin_y * lx + cos_y * ly + robot_y;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            traj_msg.poses.push_back(ps);
        }

        pub_local_traj_->publish(traj_msg);
    }

    // =====================================================================
    // Member variables
    // =====================================================================

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_traj_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;
    std::mutex costmap_mutex_;
    geometry_msgs::msg::PoseStamped goal_;
    bool goal_active_ = false;
    double cur_v_ = 0.0;  // current velocity (for dynamic window)
    double cur_w_ = 0.0;

    // === NEW: Path caching state ===
    std::vector<std::pair<double, double>> cached_global_path_odom_;  // odom-frame cached path
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
    double robot_radius_, clearance_margin_;
    double control_rate_;
    std::string odom_frame_, robot_frame_;
    double slope_speed_factor_, slope_threshold_deg_;

    // NEW: Path caching & Pure Pursuit parameters
    double replan_interval_;
    double path_deviation_threshold_;
    double pure_pursuit_lookahead_;
    double pure_pursuit_lookahead_min_;
    double pure_pursuit_lookahead_max_;
    double velocity_lookahead_gain_;
};

// =========================================================================
// Main
// =========================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();

    // SIGINT(Ctrl+C) 시 on_shutdown 콜백으로 정지 명령 발행
    rclcpp::on_shutdown([node]() {
        node->send_stop_command();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}