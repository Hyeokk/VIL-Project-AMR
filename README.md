# VIL-Project-AMR with Segmentation

Autonomous navigation on unstructured mountain terrain using camera semantic segmentation + LiDAR fusion — without GPS or pre-built maps.

> Base project information (hardware, sensor specs, docs) is on the [main branch](https://github.com/Hyeokk/VIL-Project-AMR/tree/main).

---

## Branch Overview

This branch replaces the GroundGrid-based ground segmentation with DDRNet23-Slim (7-class) semantic segmentation running on the IQ-9075 NPU. The segmentation mask is projected onto M300 LiDAR points (PointPainting) and fused with S10 Ultra depth to generate a 6m×6m semantic costmap.

| | main (GroundGrid) | amr-segmentation |
|---|---|---|
| Terrain Classification | 2-class (ground / non-ground) | 7-class semantic segmentation |
| Inference | CPU (GroundGrid) | **NPU** (DDRNet23-Slim FP16, ~10ms) |
| Costmap | Height-based | Semantic-aware |
| Packages | cloud_merger, groundgrid, terrain_costmap | **amr_segmentation** (single package) |

---

## Packages

### Sensor Drivers

| Package | Description | Source |
|---------|-------------|--------|
| pacecat_m300 | M300-E 3D LiDAR + IMU driver | [BlueSeaLidar/m300](https://github.com/BlueSeaLidar/m300) |
| lx_camera_ros | S10 Ultra dToF camera driver | [Lanxin-MRDVS/CameraSDK](https://github.com/Lanxin-MRDVS/CameraSDK) |

### Perception

| Package | Description | Source |
|---------|-------------|--------|
| fast_lio | FAST-LIO2 with M300 support (LiDAR type 5) | [hku-mars/FAST_LIO (ROS2)](https://github.com/hku-mars/FAST_LIO/tree/ROS2) |
| **amr_segmentation** | DDRNet23-Slim segmentation + PointPainting + semantic costmap | Original |

### Navigation

| Package | Description | Source |
|---------|-------------|--------|
| path_planner | A* global + DWA local planner for skid-steer | Original |

### Robot Platform

| Package | Description | Source |
|---------|-------------|--------|
| omorobot | DONKEYBOTI tracked robot: URDF, motor control, bringup | [omorobot/omorobot_ros2](https://github.com/omorobot/omorobot_ros2) |

> See each package's `README.md` for detailed parameters and configurations.

---

## Build

```bash
mkdir -p ~/amr_ws
cd ~/amr_ws
git clone -b amr-segmentation https://github.com/Hyeokk/VIL-Project-AMR.git src

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Launch

Each node group runs in a separate terminal.

```bash
# 1. Robot platform (motor + URDF TF)
export ROBOT_MODEL=DONKEYBOTI
ros2 launch omorobot_bringup bringup_launch.py publish_tf:=false

# 2. M300 LiDAR driver
ros2 launch pacecat_m300_driver LDS-M300-E.launch.py

# 3. FAST-LIO2 (localization + TF)
ros2 launch fast_lio mapping.launch.py

# 4. S10 Ultra camera driver
ros2 launch lx_camera_ros lx_camera_ros.launch.py

# 5. Segmentation pipeline (static TF + segmentation + painting + costmap)
ros2 launch amr_segmentation pipeline.launch.py

# 6. Path planner (A* + DWA)
ros2 launch path_planner path_planner.launch.py
```

---

## Docs

| Document | Description |
|----------|-------------|
| [docs/Network.md](docs/Network.md) | IQ-9075 network setup: WiFi + LiDAR + Camera routing |
| [docs/Monitoring.md](docs/Monitoring.md) | Remote rviz2 visualization via Docker (Jazzy + CycloneDDS) |

---

## License

Each package retains its original license. See individual package directories.