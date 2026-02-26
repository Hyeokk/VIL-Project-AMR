# AMR Outdoor Navigation - Simulation Test Guide

Test the full perception-planning-control pipeline in rviz without any hardware.

---

## Prerequisites

```bash
cd ~/amr_ws
colcon build --symlink-install --packages-select \
  cloud_merger groundgrid terrain_costmap path_planner
source install/setup.bash
```

Place `test_full_pipeline.py` in `~/amr_ws/`.

---

## Launch Order

Open 6 terminals. Run `source ~/amr_ws/install/setup.bash` in each.

```bash
# Terminal 1 - Simulator (must start first)
python3 test_full_pipeline.py

# Terminal 2 - Cloud Merger
ros2 launch cloud_merger cloud_merger.launch.py

# Terminal 3 - GroundGrid
ros2 launch groundgrid amr_groundgrid.launch.py

# Terminal 4 - Terrain Costmap
ros2 launch terrain_costmap terrain_costmap.launch.py

# Terminal 5 - Path Planner
ros2 launch path_planner path_planner.launch.py

# Terminal 6 - rviz
rviz2
```

Do NOT launch `omorobot_r1_bringup` or `robot_control.py`. These control the real motors.

---

## rviz Setup

Set `Fixed Frame` to `odom`.

Add the following displays:

| Type | Topic | Description |
|---|---|---|
| TF | - | Robot pose |
| OccupancyGrid | `/terrain_costmap` | Cost map |
| Path | `/path_planner/global_path` | A* path |
| Path | `/path_planner/local_trajectory` | DWA trajectory |
| PointCloud2 | `/groundgrid/filtered_cloud` | Ground/obstacle points |
| MarkerArray | `/sim/obstacles` | Simulated obstacles |

Use **2D Goal Pose** in the toolbar to set a navigation goal.

---

## Verification

```bash
ros2 topic hz /merged_cloud                 # ~10Hz
ros2 topic hz /terrain_costmap              # ~10Hz
ros2 topic hz /path_planner/global_path     # ~1Hz (after goal set)
ros2 topic hz /cmd_vel                      # ~20Hz (after goal set)
ros2 run tf2_tools view_frames              # Check TF tree
```

Expected TF chain: `odom -> camera_init -> body -> base_footprint -> base_link -> mrdvs_tof`

---

## World Configuration

Edit constants at the top of `test_full_pipeline.py`:

```python
# Static obstacles: (cx, cy, radius, height)
STATIC_OBSTACLES = [...]

# Walls: (x_start, y_start, x_end, y_end, thickness, height)
WALLS = [...]

# Dynamic obstacles: (cx, cy, radius, height, axis, range, speed)
DYNAMIC_OBSTACLES = [...]

# Grass noise standard deviation [m]
GRASS_HEIGHT_STD = 0.04
```