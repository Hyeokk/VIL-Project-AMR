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
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation target in odom frame |

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
| `control_rate` | `10.0` | Control loop rate [Hz] |

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
| `weight_heading` | 1.0 | Alignment to A* path direction |
| `weight_clearance` | 0.5 | Terrain cost / obstacle avoidance |
| `weight_velocity` | 0.3 | Forward motion preference |
| `weight_path_dist` | 0.8 | Closeness to A* global path |

### A* Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `astar_lethal_cost` | 80 | Cells ≥ this value are impassable |
| `astar_cost_weight` | 2.0 | Terrain cost penalty multiplier |

### Goal Tolerance

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_xy_tolerance` | 0.2 | Position tolerance [m] |
| `goal_yaw_tolerance` | 0.15 | Heading tolerance [rad] (~8.6°) |

## Behavior

### Control Loop (10 Hz)

Each cycle executes the following:

1. Receive latest `/terrain_costmap` (50×50 OccupancyGrid)
2. Look up robot pose via TF (`odom → base_link`)
3. If distance to goal < `goal_xy_tolerance` → align heading, then stop
4. Transform goal to robot-local coordinates
5. If goal is outside costmap (5m) → project intermediate waypoint to costmap boundary
6. **A\*** searches the 50×50 grid for a global path
7. **DWA** evaluates 231 velocity candidates `(v, ω)` and selects the best trajectory
8. Publish `Twist` on `/cmd_vel` → `robot_control` converts to serial `$cVW,v,w`

### Turn-in-Place (Caterpillar)

When the heading error to the goal is large, DWA naturally selects `v ≈ 0, ω ≠ 0` combinations that score highest. This produces turn-in-place behavior without any special-case logic.

### Emergency Stop

The planner issues `v = 0, ω = 0` when:

- A* finds no valid path to the goal
- All DWA trajectory candidates result in collision
- TF lookup fails (localization lost)
- No costmap has been received yet

### Goal Outside Costmap

When the goal is beyond the 5m × 5m costmap range, the planner projects an intermediate waypoint to the costmap boundary in the direction of the goal. The robot navigates toward this intermediate point, and replans each cycle as new costmap data arrives.

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
| Path | `/path_planner/global_path` | Green |
| Path | `/path_planner/local_trajectory` | Yellow |
| TF | — | Show `odom → base_link` chain |

## Performance

Benchmark results (50×50 grid, 231 DWA trajectories, rectangular footprint):

| Platform | Mean | P99 (worst) |
|----------|------|-------------|
| Python (CPython) | 3–18 ms | ~39 ms |
| C++ (this package) | ~0.3–1.2 ms | ~2.6 ms |
| C++ ARM64 (IQ-9075, estimated) | ~0.5–2.5 ms | ~5.2 ms |

With the full upstream pipeline (sensors → FAST-LIO2 → GroundGrid → costmap) consuming ~17 ms, the total end-to-end latency is well within the 100 ms budget for 10 Hz operation.

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