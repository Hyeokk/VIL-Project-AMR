# GroundGrid (AMR Fork)

GroundGrid ground segmentation modified for Pacecat M300 LiDAR + FAST-LIO2 pipeline.

> Source: [dcmlr/groundgrid (ros2-jazzy)](https://github.com/dcmlr/groundgrid/tree/ros2-jazzy)

---

## Modifications from Original

| Change | Detail |
|--------|--------|
| Fixed hardcoded `velodyne` frame | `points_callback` now uses `cloud_msg->header.frame_id` |
| Removed `velodyne_pointcloud` dep | Header already bundled locally |
| Fixed `rclpy` in CMakeLists | Removed from C++ build |
| Added `param/m300.yaml` | M300 non-repetitive LiDAR config for mountain terrain |
| Added `amr_groundgrid.launch.py` | Launch with proper TF chain for AMR |

---

## TF Chain

```
odom
  └── base_link          ← FAST-LIO2 (dynamic TF)
        └── m300_link    ← static TF (LiDAR mount, from launch)
```

GroundGrid requires:
- `odom` → `base_link` TF (provided by FAST-LIO2)
- `odom` → `<cloud_frame_id>` TF chain (for point cloud transformation)
- Odometry on `/Odometry` (remapped to `/groundgrid/odometry_in`)

---

## Build

```bash
# Prerequisites (Jazzy)
sudo apt install ros-jazzy-grid-map ros-jazzy-grid-map-core ros-jazzy-grid-map-ros \
  ros-jazzy-grid-map-cv ros-jazzy-grid-map-msgs ros-jazzy-grid-map-filters \
  ros-jazzy-grid-map-visualization ros-jazzy-image-transport ros-jazzy-cv-bridge \
  ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-filters

cd ~/amr_ws
colcon build --symlink-install --packages-select groundgrid
```

## Run

With FAST-LIO2 already running:
```bash
ros2 launch groundgrid amr_groundgrid.launch.py
```

Override LiDAR mount position:
```bash
ros2 launch groundgrid amr_groundgrid.launch.py \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.6 \
  pointcloud_topic:=/cloud_registered_body \
  odometry_topic:=/Odometry
```

Use raw M300 cloud (needs base_link→m300_link TF):
```bash
ros2 launch groundgrid amr_groundgrid.launch.py \
  pointcloud_topic:=/m300/pointcloud
```

---

## Topics

| Topic | Type | I/O | Description |
|-------|------|-----|-------------|
| `/cloud_registered_body` | PointCloud2 | Input | FAST-LIO2 body-frame cloud (default) |
| `/Odometry` | Odometry | Input | FAST-LIO2 odometry |
| `/groundgrid/filtered_cloud` | PointCloud2 | Output | Non-ground points (obstacles) |
| `/groundgrid/grid_map` | GridMap | Output | Elevation + segmentation grid |

---

## Config (param/m300.yaml)

Key parameters for mountain terrain tuning:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `min_point_height_thres` | 0.4m | Points below ground+threshold = ground |
| `min_point_height_obstacle_thres` | 0.15m | Obstacle-adjacent ground threshold |
| `outlier_tolerance` | 0.15m | Outlier detection tolerance |
| `patch_size_change_distance` | 15m | Distance for patch size increase |
| `min_dist_squared` | 4.0 | Ignore points < 2m from sensor |
| `max_threads` | 4 | Thread count (IQ-9075 octa-core) |
| `horizontal_point_ang_dist` | 0.0031 | M300 angular resolution (~0.18°) |
