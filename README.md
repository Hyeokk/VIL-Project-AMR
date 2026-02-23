# VIL-Project-AMR

Vision-Inertial-LiDAR autonomous mobile robot for outdoor mountain terrain navigation.

- Platform: OMOROBOT DONKEYBOTI (tracked, skid-steer)
- LiDAR: Pacecat LDS-M300-E (3D, 200kHz, 360 x 70 deg FOV, built-in 6-axis IMU)
- Camera: MRDVS S10 Ultra (dToF, 120 x 80 deg FOV, XYZRGB point cloud)
- Computer: Dragonwing IQ-9075 (Qualcomm QCS8550, ARM64)
- OS: Ubuntu 24.04, ROS2 Jazzy
- Environment: Outdoor mountain terrain

---

## Packages

### Sensor Drivers

| Package | Description | Source | License |
|---------|-------------|--------|---------|
| pacecat_m300 | M300-E 3D LiDAR + IMU driver | [BlueSeaLidar/m300](https://github.com/BlueSeaLidar/m300) | MIT |
| lx_camera_ros | S10 Ultra dToF camera driver | [Lanxin-MRDVS/CameraSDK](https://github.com/Lanxin-MRDVS/CameraSDK) | Proprietary (SDK) |

### Perception

| Package | Description | Source | License |
|---------|-------------|--------|---------|
| fast_lio | FAST-LIO2 with M300 support (LiDAR type 5), livox dependency removed | [hku-mars/FAST_LIO (ROS2)](https://github.com/hku-mars/FAST_LIO/tree/ROS2) | GPL-2.0 |
| cloud_merger | Merges M300 + S10 Ultra point clouds in body frame | Original | BSD-3 |
| groundgrid | Grid-based ground segmentation and elevation mapping | [dcmlr/groundgrid (ros2-jazzy)](https://github.com/dcmlr/groundgrid/tree/ros2-jazzy) | BSD-3 |

### Robot Platform

| Package | Description | Source | License |
|---------|-------------|--------|---------|
| omorobot | DONKEYBOTI tracked robot: URDF, motor control, bringup | [omorobot/omorobot_ros2](https://github.com/omorobot/omorobot_ros2) | Apache-2.0 |

---

## Build

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
git clone https://github.com/Hyeokk/VIL-Project-AMR.git .

cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Sequence

Each node group runs in a separate terminal.

```bash
# 1. Robot platform (motor + URDF TF)
export ROBOT_MODEL=DONKEYBOTI
ros2 launch omorobot_bringup bringup_launch.py publish_tf:=false

# 2. M300 LiDAR driver
ros2 launch pacecat_m300_driver LDS-M300-E.launch.py

# 3. FAST-LIO2 + static TF bridges
ros2 launch fast_lio mapping.launch.py

# 4. S10 Ultra camera driver
ros2 launch lx_camera_ros lx_camera_ros.launch.py

# 5. Point cloud merger
ros2 launch cloud_merger cloud_merger.launch.py

# 6. Ground segmentation
ros2 launch groundgrid amr_groundgrid.launch.py
```

---

## Testing Without Sensors

```bash
# Terminal 1: Simulated sensor data + TF
python3 test/test_pipeline.py

# Terminal 2-3: cloud_merger and GroundGrid (same as above)

# Terminal 4: Real-time Hz and latency monitor
python3 test/test_monitor.py
```

### Expected test_monitor.py Output

```
[  10.0s]
  cloud_registered_body    |  10.0 Hz |     1.2 ms |  2772 pts
  LxCamera_Cloud           |  10.0 Hz |     1.5 ms |  1634 pts
  merged_cloud             |  10.0 Hz |     2.8 ms |  4406 pts
  filtered_cloud           |  10.0 Hz |    15.3 ms |  1205 pts
  Odometry                 |  10.0 Hz |     0.3 ms |        -
```

---

## Current Status

- [x] Sensor drivers (M300 LiDAR, S10 Ultra camera)
- [x] FAST-LIO2 localization
- [x] Point cloud merger (M300 + S10 Ultra)
- [x] GroundGrid ground segmentation
- [x] TF tree and URDF integration
- [x] Sensor-less test pipeline
- [ ] Terrain-aware costmap generation
- [ ] Nav2 path planning integration
- [ ] Safety systems (rollover detection, emergency stop)
- [ ] S10 Ultra extrinsic calibration (current values are estimates)

---

## License

Each package retains its original license. See individual package directories.