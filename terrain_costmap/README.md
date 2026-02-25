# terrain_costmap

A ROS2 package that generates a robot-centric traversability costmap from GroundGrid output. Designed for caterpillar (skid-steer) robots operating on mountain terrain.

## Overview

This node subscribes to GroundGrid's 120m×120m GridMap, crops a 3m×3m submap around the robot, and computes per-cell traversability cost from four factors:

- **Obstacle density** — non-ground point count from the `points` layer
- **Slope** — elevation gradient computed from the `ground` layer using central differences
- **Roughness** — height variance from the `variance` layer
- **Uncertainty** — low confidence in the `groundpatch` layer penalizes unobserved areas

The output is a standard `nav_msgs/OccupancyGrid` (values 0–100) in the robot frame, ready for path planners such as Nav2, A\*, or DWA.

### Persistent Terrain Accumulation

An optional feature accumulates ground points (from GroundGrid's `filtered_cloud`) in the odom frame over time. This compensates for sensor blind spots — particularly the rear and sides of the robot where neither the M300 LiDAR nor the S10 Ultra camera has coverage. A sliding window (default 10m radius) manages memory.

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/groundgrid/grid_map` | `grid_map_msgs/GridMap` | GroundGrid elevation map with layers |
| Subscribe | `/groundgrid/filtered_cloud` | `sensor_msgs/PointCloud2` | Labeled cloud (ground=49, obstacle=99) |
| Publish | `/terrain_costmap` | `nav_msgs/OccupancyGrid` | 3m×3m traversability costmap |
| Publish | `/terrain_costmap/terrain_cloud` | `sensor_msgs/PointCloud2` | Accumulated terrain (debug/viz) |

## TF Requirements

The node requires the following TF chain to be available:

`odom` → `camera_init` → `body` → `base_footprint` → `base_link`

- `odom → camera_init`: static identity
- `camera_init → body`: provided by FAST-LIO2 (dynamic)
- `body → base_link`: static offset from URDF

## Parameters

All parameters are defined in `param/terrain_costmap.yaml`.

### Costmap Geometry

| Parameter | Default | Description |
|-----------|---------|-------------|
| `costmap_size` | `3.0` | Side length of square costmap [m] |
| `costmap_resolution` | `0.1` | Cell size [m] (3m / 0.1m = 30×30 = 900 cells) |
| `robot_frame` | `base_link` | Robot body frame |
| `odom_frame` | `odom` | Odometry frame |

### Cost Weights

| Parameter | Default | Description |
|-----------|---------|-------------|
| `weight_obstacle` | `0.40` | Weight for obstacle density |
| `weight_slope` | `0.30` | Weight for terrain inclination |
| `weight_roughness` | `0.15` | Weight for surface irregularity |
| `weight_unknown` | `0.15` | Weight for unobserved area penalty |

### Thresholds

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slope_max_deg` | `35.0` | Slopes ≥ this angle → cost 1.0 (caterpillar tolerance) |
| `roughness_max` | `0.05` | Variance ≥ this → cost 1.0 [m²] |
| `obstacle_count_max` | `5.0` | Obstacle points ≥ this → cost 1.0 |
| `confidence_threshold` | `0.1` | Groundpatch confidence below this → unknown |
| `lethal_threshold` | `0.8` | Total cost ≥ this → cell set to 100 (impassable) |

### Terrain Accumulation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_terrain_accumulation` | `true` | Enable ground point accumulation |
| `terrain_max_points` | `200000` | Max accumulated points in memory |
| `terrain_radius` | `10.0` | Sliding window radius around robot [m] |
| `terrain_publish_rate` | `1.0` | Terrain cloud publish rate for debug [Hz] |

## Build

Place the `terrain_costmap` directory inside the `VIL-Project-AMR` repository at the top level, alongside `cloud_merger`, `groundgrid`, etc.

```bash
cd ~/colcon_ws
colcon build --packages-select terrain_costmap
source install/setup.bash
```

### Dependencies

- `grid_map_ros`, `grid_map_msgs`, `grid_map_core`
- `tf2_ros`, `tf2`, `tf2_geometry_msgs`
- `pcl_conversions`, PCL (common, io, filters)
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`

## Launch

```bash
# Default (3m×3m, 0.1m resolution)
ros2 launch terrain_costmap terrain_costmap.launch.py

# Custom size
ros2 launch terrain_costmap terrain_costmap.launch.py costmap_size:=5.0 costmap_resolution:=0.2
```

## Cost Calculation

Each costmap cell value is computed as:

```
total = w_obstacle × obstacle_cost
      + w_slope    × slope_cost
      + w_roughness× roughness_cost
      + w_unknown  × unknown_cost

cell_value = total × 99       (if total < lethal_threshold)
cell_value = 100               (if total ≥ lethal_threshold or definite obstacle)
cell_value = -1                (if cell has no data)
```

Where each individual cost is normalized to [0.0, 1.0]:

- `obstacle_cost` = min(point_count / obstacle_count_max, 1.0)
- `slope_cost` = min(atan(gradient_magnitude) / slope_max_rad, 1.0)
- `roughness_cost` = min(variance / roughness_max, 1.0)
- `unknown_cost` = 1.0 if confidence < threshold, else 0.0

## Notes

- GroundGrid runs its full 120m×120m GridMap computation regardless of this node's crop size. The crop only reduces downstream cost calculation and path planner search space.
- The `visualize` parameter in GroundGrid's launch must be set to `true` for terrain accumulation to work, since ground points are identified by their intensity label (49).
- The terrain accumulation cloud is separate from FAST-LIO2's ikd-Tree map. It must never be fed back into FAST-LIO2.