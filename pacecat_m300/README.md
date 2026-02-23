# PACECAT M300 LiDAR ROS2 Driver

ROS2 Jazzy driver for PACECAT LDS-M300-E 3D LiDAR.
Publishes point cloud with per-point timestamps, IMU data, and custom messages.

> Source: [BlueSeaLidar/m300](https://github.com/BlueSeaLidar/m300) — modified for ROS2 Jazzy compatibility

---

## Package Structure

```
pacecat_m300/
├── pacecat_m300_inter/        Message / service definitions
│   ├── msg/CustomMsg.msg      Per-frame point cloud
│   ├── msg/CustomPoint.msg    Per-point data (includes offset_time)
│   └── srv/Control.srv        LiDAR start / stop control
│
└── pacecat_m300_driver/       Driver node
    ├── src/driver.cpp         ROS2 node (publishers + SDK callbacks)
    ├── sdk/                   PACECAT SDK (UDP recv, frame assembly, timestamp interpolation)
    ├── params/LDS-M300-E.yaml Parameter config
    └── launch/LDS-M300-E.launch.py
```

---

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/pointcloud` | `sensor_msgs/msg/PointCloud2` | 10 Hz | Point cloud (x, y, z, intensity, tag, line, **timestamp**) |
| `/custommsg` | `pacecat_m300_inter/msg/CustomMsg` | 10 Hz | Custom format with **offset_time** per point (for FAST-LIO2) |
| `/imu` | `sensor_msgs/msg/Imu` | ~200 Hz | Built-in IIM-42652 IMU (same clock as LiDAR) |

---

## Per-Point Timestamp

The M300 SDK internally interpolates the measurement time of each point within a UDP packet:

```
offset_time[i] = packet.timestamp
               + i × packet.time_interval × 100 / (dot_num - 1)
               - frame_start_timestamp
```

This produces a **nanosecond offset from frame start** for every point. At 10 Hz, offset values range from 0 to ~100 ms across one frame.

**PointCloud2** stores this as `timestamp` field (float64, ns, offset 18).
**CustomMsg** stores this as `offset_time` field (uint32, ns).

For **FAST-LIO2** integration, convert in `preprocess.cpp`:
```
curvature = offset_time / 1,000,000.0    // ns → ms
```

---

## Modifications from Original

All changes are build-level only. **No runtime logic was modified.**

| File | Change | Reason |
|------|--------|--------|
| `driver.cpp` | Removed 3 deprecated includes (`PointCloud`, `point_cloud_conversion`, `String`) | Deleted from `sensor_msgs` in Jazzy |
| `sdk/global.h` | Added `#ifndef` guard around `M_PI` | Prevent redefinition warning |
| `CMakeLists.txt` | Removed `client` target, separated SDK warning flags | Unused executable, suppress vendor `-Wpedantic` warnings |
| `launch/` | Renamed `.launch` → `.launch.py` | ROS2 Jazzy requires Python extension |

---

## Configuration

Edit `params/LDS-M300-E.yaml` before running:

```yaml
adapter: eth0              # Network interface (check: ip link show)
lidar_ip: 192.168.0.198    # M300 IP address
timemode: 0                # 0 = LiDAR hardware clock (recommended for FAST-LIO2)
                           # 1 = Host system clock
frame_package_num: 150     # UDP packets per frame (valid: 120–200)
```

---

## Run

```bash
source ~/amr_ws/install/setup.bash

# Launch
ros2 launch pacecat_m300_driver LDS-M300-E.launch.py

# Or run directly with parameter override
ros2 run pacecat_m300_driver driver --ros-args \
  --params-file src/pacecat_m300/pacecat_m300_driver/params/LDS-M300-E.yaml \
  -p adapter:=eth0
```