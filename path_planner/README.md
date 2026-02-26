# path_planner — A* + DWA for DONKEYBOTI Caterpillar Robot

## Overview

Path planning and control node combining an A* Global Planner with a DWA (Dynamic Window Approach) Local Planner. Operates on a 5m × 5m terrain costmap (50×50 cells, 0.1m resolution) and supports **turn-in-place** for caterpillar (skid-steer) kinematics.

Collision checking uses a **rectangular footprint** (1.432m × 0.85m) that rotates with the robot heading, ensuring accurate clearance for the full body at every orientation.

## Node Graph

```
/terrain_costmap (OccupancyGrid, 50x50, 10Hz)
        │
        ▼
┌───────────────────────┐     /goal_pose (PoseStamped)
│  path_planner_node    │◄────────────────────────────
│                       │
│  ┌─────────────────┐  │     TF: odom → base_link
│  │ A* Global Path  │  │◄────────────────────────────
│  └───────┬─────────┘  │
│          ▼             │
│  ┌─────────────────┐  │
│  │ DWA Local Plan  │  │
│  │ (231 candidates)│  │
│  └───────┬─────────┘  │
└──────────┼─────────────┘
           │
     /cmd_vel (Twist)
           │
           ▼
┌───────────────────────┐
│  robot_control        │  ← bringup_launch.py
│  serial: $cVW,v,w     │
│  115200 baud          │
└───────────────────────┘
```

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/terrain_costmap` | `nav_msgs/OccupancyGrid` | 5m × 5m traversability costmap (from `terrain_costmap_node`) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation target (any frame, auto-transformed to odom) |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command → `robot_control` → serial motor |
| `/path_planner/global_path` | `nav_msgs/Path` | A* global path (RViz visualization) |
| `/path_planner/local_trajectory` | `nav_msgs/Path` | DWA best trajectory (RViz visualization) |

### TF Dependencies

| Transform | Source | Purpose |
|-----------|--------|---------|
| `odom → base_link` | FAST-LIO2 (or wheel odometry) | Robot pose for planning |
| `base_footprint → base_link` | `robot_state_publisher` (URDF) | Static offset |
| `base_link → m300_link` | `robot_state_publisher` (URDF) | LiDAR mount position |

## Build

```bash
cd ~/ros2_ws
cp -r path_planner src/
colcon build --packages-select path_planner
source install/setup.bash
```

## Full System Launch Order

```bash
# Terminal 1: Robot motor driver + URDF TF
ros2 launch omorobot_bringup bringup_launch.py

# Terminal 2: Sensors + SLAM (provides TF: odom → base_link)
# (project-specific launch for FAST-LIO2)

# Terminal 3: Terrain costmap
ros2 launch terrain_costmap terrain_costmap.launch.py

# Terminal 4: Path planner
ros2 launch path_planner path_planner.launch.py
```

## Sending Goals

### Option 1: RViz2 (recommended)

Click **"2D Goal Pose"** in the RViz2 toolbar, then click and drag on the map to set the target position and orientation.

### Option 2: CLI

```bash
# Move 2m forward
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}"

# Move to (1.5, 1.0) in odom frame
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 1.5, y: 1.0, z: 0.0}, \
           orientation: {w: 1.0}}}"
```

## Launch Arguments

```bash
# Default (full speed)
ros2 launch path_planner path_planner.launch.py

# Slow mode for slope testing
ros2 launch path_planner path_planner.launch.py max_lin_vel:=0.3

# Ultra-slow precision mode
ros2 launch path_planner path_planner.launch.py max_lin_vel:=0.15 max_ang_vel:=0.3
```

| Argument | Default | Description |
|----------|---------|-------------|
| `max_lin_vel` | `0.6` | Maximum linear velocity [m/s] |
| `max_ang_vel` | `0.5` | Maximum angular velocity [rad/s] |
| `control_rate` | `20.0` | Control loop rate [Hz] |

## Key Parameters

All parameters are in `param/path_planner.yaml`.

### Robot Footprint

The collision check uses a **rectangular footprint** matching the actual DONKEYBOTI dimensions. At each simulated trajectory point, the rectangle is rotated to the robot's heading and checked against the costmap.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_length` | 1.432 | Front-to-back size [m] (x-axis) |
| `robot_width` | 0.850 | Left-to-right size [m] (y-axis) |
| `clearance_margin` | 0.1 | Safety buffer added to all sides [m] |

Effective footprint per collision check: **1.632m × 1.05m** (with margin).

### Robot Kinematics (DONKEYBOTI)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_lin_vel` | 0.6 | Max forward velocity [m/s] |
| `min_lin_vel` | -0.2 | Max reverse velocity [m/s] |
| `max_ang_vel` | 0.5 | Max angular velocity [rad/s] |
| `max_lin_acc` | 1.0 | Linear acceleration limit [m/s²] |
| `max_ang_acc` | 2.0 | Angular acceleration limit [rad/s²] |

### DWA Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `n_v_samples` | 11 | Linear velocity samples |
| `n_w_samples` | 21 | Angular velocity samples (more for turn-in-place) |
| `sim_time` | 1.5 | Forward simulation horizon [s] |
| `sim_granularity` | 0.1 | Simulation time step [s] |
| `weight_heading` | 0.6 | Alignment to A* path direction |
| `weight_clearance` | 0.5 | Terrain cost / obstacle avoidance |
| `weight_velocity` | 0.8 | Forward motion preference |
| `weight_path_dist` | 0.8 | Closeness to A* global path |
| `spin_heading_threshold` | 1.05 | Heading error [rad] below which pure rotation is penalized |

### A* Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `astar_lethal_cost` | 90 | Cells ≥ this value are impassable |
| `astar_cost_weight` | 1.0 | Terrain cost penalty multiplier |

### Goal Tolerance

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_xy_tolerance` | 0.3 | Position tolerance [m] — robot stops on position only, no yaw alignment |

### Path Caching & Pure Pursuit

| Parameter | Default | Description |
|-----------|---------|-------------|
| `replan_interval` | 1.0 | Periodic A* replan interval [s] |
| `path_deviation_threshold` | 0.5 | Replan if robot deviates this far [m] |
| `pure_pursuit_lookahead` | 0.4 | Base lookahead distance [m] |
| `pure_pursuit_lookahead_min` | 0.2 | Minimum lookahead (low speed) [m] |
| `pure_pursuit_lookahead_max` | 0.8 | Maximum lookahead (high speed) [m] |
| `velocity_lookahead_gain` | 0.5 | lookahead = base + gain × |v| |

## Behavior

### Control Loop (20 Hz)

Each cycle executes the following:

1. Receive latest `/terrain_costmap` (50×50 OccupancyGrid)
2. Look up robot pose via TF (`odom → base_link`)
3. If distance to goal < `goal_xy_tolerance` → stop immediately
4. A* replan only when needed (new goal / deviation / periodic interval)
5. Pure Pursuit selects stable lookahead point on cached odom-frame path
6. If goal is outside costmap (5m) → project intermediate waypoint to costmap boundary
7. **DWA** evaluates 231 velocity candidates `(v, ω)` with anti-spin scoring
8. Publish `Twist` on `/cmd_vel` → `robot_control` converts to serial `$cVW,v,w`

### Turn-in-Place (Caterpillar)

When the heading error to the goal exceeds `spin_heading_threshold` (~60°), DWA allows `v ≈ 0, ω ≠ 0` combinations. Below this threshold, pure rotation is penalized to prevent unnecessary spinning.

### Emergency Stop

The planner issues `v = 0, ω = 0` when:

- A* finds no valid path to the goal
- All DWA trajectory candidates result in collision
- TF lookup fails (localization lost)
- No costmap has been received yet

### Goal Outside Costmap

When the goal is beyond the 5m × 5m costmap range, the planner projects an intermediate waypoint to the costmap boundary in the direction of the goal. The robot navigates toward this intermediate point, and replans as new costmap data arrives.

## Monitoring

```bash
# Check velocity commands being sent
ros2 topic echo /cmd_vel

# Check if costmap is being received at expected rate
ros2 topic hz /terrain_costmap

# View A* global path
ros2 topic echo /path_planner/global_path

# View DWA selected trajectory
ros2 topic echo /path_planner/local_trajectory
```

### RViz2 Display Setup

| Display Type | Topic | Color |
|-------------|-------|-------|
| Map | `/terrain_costmap` | — |
| Path | `/path_planner/global_path` | Red or Blue |
| Path | `/path_planner/local_trajectory` | Yellow |
| TF | — | Show `odom → base_link` chain |

## Performance

Benchmark results (50×50 grid, 231 DWA trajectories, rectangular footprint):

| Platform | Mean | P99 (worst) |
|----------|------|-------------|
| Python (CPython) | 3–18 ms | ~39 ms |
| C++ (this package) | ~0.3–1.2 ms | ~2.6 ms |
| C++ ARM64 (IQ-9075, estimated) | ~0.5–2.5 ms | ~5.2 ms |

With the full upstream pipeline (sensors → FAST-LIO2 → GroundGrid → costmap) consuming ~17 ms, the total end-to-end latency is well within the 50 ms budget for 20 Hz operation.

---

## Field Tuning Guide

Config files: **P** = `path_planner.yaml`, **T** = `terrain_costmap.yaml`, **G** = `groundgrid/m300.yaml`, **F** = `fast_lio/m300.yaml`

All YAML changes take effect on node restart (no rebuild). Runtime: `ros2 param set /node_name param value`

| Parameter | File | Increase when | Decrease when |
|-----------|------|--------------|---------------|
| `weight_heading` | P | Robot ignores goal direction | Robot spins in place |
| `weight_velocity` | P | Robot spins instead of driving forward | Robot ignores obstacles to go fast |
| `weight_clearance` | P | Robot too close to obstacles | Robot detours around everything |
| `weight_path_dist` | P | Robot strays from planned path | Robot can't deviate to avoid obstacles |
| `spin_heading_threshold` | P | Still spins at small heading errors | Drives forward when it should turn |
| `astar_cost_weight` | P | Robot cuts through rough terrain | Robot detours on passable ground |
| `astar_lethal_cost` | P | "No valid trajectory" on passable cells | Robot drives into obstacles |
| `clearance_margin` | P | Robot bumps into obstacles | "No valid trajectory" in tight spaces |
| `goal_xy_tolerance` | P | Robot circles around goal | Robot stops too far from target |
| `max_lin_vel` | P | Robot too slow on flat ground | Robot overshoots goal |
| `replan_interval` | P | Robot jitters during straight driving | Robot reacts slowly to new obstacles |
| `pure_pursuit_lookahead` | P | Robot oscillates left-right | Robot cuts corners |
| `sim_time` | P | "No valid trajectory" near obstacles | DWA too slow (CPU) |
| `weight_slope` | T | Robot drives off steep edges | Robot detours on gentle slopes |
| `weight_roughness` | T | Robot on dangerously rough ground | Robot avoids normal grass |
| `roughness_max` | T | Grass flagged as high cost | Robot ignores rough terrain |
| `obstacle_count_max` | T | Grass points create false obstacles | Real small obstacles ignored |
| `lethal_threshold` | T | "No valid trajectory" too often | Robot drives into obstacles |
| `obstacle_decay_time` | T | Moving obstacles clear too fast | Ghost obstacles persist |
| `min_point_height_obstacle_thres` | G | Grass classified as obstacle | Low obstacles (curbs) missed |
| `outlier_tolerance` | G | Grass tips flagged as outliers | Noise accepted as ground |
| `filter_size_surf` | F | CPU too high | Localization drifting |
| `acc_cov` / `gyr_cov` | F | Caterpillar vibration causes drift | Position jumps randomly |

---

## File Structure

```
path_planner/
├── CMakeLists.txt                  # ament_cmake build configuration
├── package.xml                     # ROS2 package metadata
├── README.md                       # This file
├── launch/
│   └── path_planner.launch.py      # Launch file with runtime arguments
├── param/
│   └── path_planner.yaml           # All configurable parameters
└── src/
    └── path_planner_node.cpp       # A* + DWA main node (C++)
```