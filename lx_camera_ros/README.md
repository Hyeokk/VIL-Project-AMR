# MRDVS S10 Ultra ROS2 Driver

ROS2 Jazzy driver for MRDVS S10 Ultra dToF camera.

> Source: [Lanxin-MRDVS/CameraSDK](https://github.com/Lanxin-MRDVS/CameraSDK) — modified for ROS2 Jazzy

---

## Package Structure

```
lx_camera_ros/
├── src/lx_camera/          Camera driver node
├── src/utils/              SDK dynamic loader (dlopen)
├── sdk/                    Bundled SDK (headers, libs, install script)
├── msg/                    Custom messages (Obstacle, Pallet, FrameRate, Result)
├── srv/                    Custom services (LxBool, LxCmd, LxFloat, LxInt, LxString)
└── launch/lx_camera_ros.launch.py
```

---

## Setup & Build

```bash
# 1. Install SDK (once)
cd ~/amr_ws/src/lx_camera_ros/sdk
sudo bash install.sh

# 2. Build
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lx_camera_ros
```

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `LxCamera_Cloud` | `PointCloud2` | 3D point cloud (XYZRGB or XYZIRT) |
| `LxCamera_LidarCloud` | `PointCloud2` | Per-point timestamped cloud (is_xyz=2 only) |
| `LxCamera_Depth` | `Image` | Depth image (240×160, mono16) |
| `LxCamera_Rgb` | `Image` | RGB image (1280×1080) |
| `LxCamera_Imu` | `Imu` | Built-in IMU (accel + gyro, ~200 Hz) |
| `LxCamera_TofInfo` | `CameraInfo` | ToF intrinsics + distortion |

---

## Run

```bash
source ~/amr_ws/install/setup.bash
ros2 launch lx_camera_ros lx_camera_ros.launch.py
```

Key parameters in `launch/lx_camera_ros.launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ip` | `""` | Camera IP (empty = auto-discover) |
| `is_xyz` | `1` | 0: off, 1: XYZRGB cloud, **2: XYZIRT with per-point timestamp** |
| `is_depth` | `1` | Publish depth image |
| `LX_BOOL_ENABLE_2D_STREAM` | `1` | Publish RGB image |
| `LX_BOOL_ENABLE_IMU` | `0` | Enable IMU stream |
| `LX_INT_XYZ_UNIT` | `0` | 0: mm, 1: m |

---

## Modifications from Original

**No runtime logic was modified.** All changes are for Jazzy build compatibility.

| File | Change | Reason |
|------|--------|--------|
| `lx_camera.h/cpp` | Removed deprecated `point_cloud.hpp`, `point_cloud_conversion.hpp` | Deleted in Jazzy |
| `CMakeLists.txt` | Removed localization/sensor_sim targets, `-march=native` | Unused nodes, cross-compile compat |
| `package.xml` | Unified `<depend>` tags | Proper dependency declaration |
| `sdk/` | Bundled SDK headers + libs | Self-contained package |
