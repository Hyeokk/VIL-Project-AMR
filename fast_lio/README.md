# FAST-LIO2 (Pacecat M300 Fork)

FAST-LIO2 modified for Pacecat LDS-M300-E LiDAR on ROS2 Jazzy.

> Source: [hku-mars/FAST_LIO (ROS2 branch)](https://github.com/hku-mars/FAST_LIO/tree/ROS2)

---

## Modifications from Original

**No algorithm logic was modified.** Changes are for M300 compatibility and livox dependency removal.

| Change | Detail |
|--------|--------|
| Added `PACECAT` LiDAR type (5) | `preprocess.h/cpp` — M300 point cloud handler |
| Added `m300_ros::Point` | PCL type matching M300 PointCloud2 field layout |
| Removed `livox_ros_driver2` dependency | No longer needed — all inputs via standard PointCloud2 |
| Added `config/m300.yaml` | M300-specific config (topics, IMU extrinsics, scan params) |

---

## M300 Point Cloud Format

The M300 driver publishes `sensor_msgs/PointCloud2` with fields:

| Field | Type | Offset | Unit |
|-------|------|--------|------|
| x, y, z | float32 | 0, 4, 8 | meters |
| intensity | float32 | 12 | - |
| tag | uint8 | 16 | - |
| line | uint8 | 17 | - |
| timestamp | float64 | 18 | nanoseconds (offset from first point) |

`m300_handler()` converts `timestamp` (ns) → `curvature` (ms) for FAST-LIO2 motion undistortion.

---

## Build

```bash
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select fast_lio
```

## Run

```bash
source ~/amr_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

Override config:
```bash
ros2 launch fast_lio mapping.launch.py config_file:=m300.yaml rviz:=false
```

---

## Config (config/m300.yaml)

| Parameter | Value | Note |
|-----------|-------|------|
| `lid_topic` | `/m300/pointcloud` | M300 driver PointCloud2 topic |
| `imu_topic` | `/m300/imu` | M300 driver IMU topic |
| `lidar_type` | 5 | PACECAT enum |
| `timestamp_unit` | 3 | NS (nanoseconds) |
| `scan_line` | 1 | Non-repetitive (no fixed scan lines) |
| `scan_rate` | 10 | 10 Hz |
| `blind` | 0.5 | Min range (m) |
| `fov_degree` | 360 | Full horizontal FOV |
| `det_range` | 50 | Max range (m) |
| `extrinsic_T` | [0.0194, -0.0287, -0.0452] | IMU→LiDAR offset from M300 manual |
| `extrinsic_est_en` | true | Online extrinsic estimation |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cloud_registered` | `PointCloud2` | Registered cloud (world frame) |
| `/cloud_registered_body` | `PointCloud2` | Registered cloud (body frame) |
| `/Odometry` | `Odometry` | 6-DOF pose |
| `/path` | `Path` | Trajectory |
| `/Laser_map` | `PointCloud2` | Accumulated map |
