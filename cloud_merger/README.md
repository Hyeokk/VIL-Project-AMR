# cloud_merger

ROS 2 node that merges point clouds from **LDS-M300 LiDAR** and **MRDVS S10 Ultra ToF camera** into a single `PointXYZI` cloud for GroundGrid ground segmentation.

## Why Merge?

The M300 LiDAR has a vertical angle range of **-10° to +60°**. At the sensor height of ~0.63m, the minimum downward angle of -10° creates a **blind zone within ~3.6m** where the LiDAR cannot see the ground. The S10 Ultra (120°×80° FoV, forward-mounted) fills this near-ground gap.

```
         M300 LiDAR (-10°~+60°, 360°)
              ●
             /|
    blind   / |  can see ground
    zone   /  |  beyond ~3.6m
          /   |
  -------/----+-----------------------  ground
  <3.6m>

  S10 Ultra (120°×80°, forward)
  fills this gap ↑
```

If the S10 Ultra is unavailable, the M300 cloud passes through alone (degraded but functional).

## Sensor Specifications

| Sensor | LDS-M300 LiDAR | MRDVS S10 Ultra |
|--------|-----------------|-----------------|
| Type | 3D LiDAR (non-repetitive) | ToF depth camera |
| FoV | 360° × 70° (-10°~+60°) | 120° × 80° |
| Range | 0.05~50m | 0.2~45m |
| Rate | 10 Hz | 10 Hz |
| Output | PointXYZINormal (body frame) | PointXYZRGB (mrdvs_tof frame) |
| Clock | Sensor-internal | System clock |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cloud_registered_body` | sensor_msgs/PointCloud2 | Subscribe | M300 registered cloud (body frame, from FAST-LIO2) |
| `/lx_camera_node/LxCamera_Cloud` | sensor_msgs/PointCloud2 | Subscribe | S10 Ultra depth cloud (mrdvs_tof frame) |
| `/merged_cloud` | sensor_msgs/PointCloud2 | Publish | Merged PointXYZI cloud (body frame) |

## TF Dependencies

```
odom → base_link        (FAST-LIO2, dynamic)
base_link → body        (URDF, static)
base_link → mrdvs_tof   (this launch, static: x=0.35 z=0.35)
```

The `base_link → mrdvs_tof` static TF is published by this launch file. The S10 Ultra is mounted 0.35m forward and 0.35m up from base_link.

## Height Filter

In indoor/tunnel environments, ceiling points are classified as non-ground by GroundGrid and appear as false obstacles in the costmap. The height filter removes points above a configurable threshold **before** GroundGrid processing.

### How It Works

The filter uses **ground-referenced** parameters for intuitive configuration:

```
max_height_from_ground: 2.0   ← user sets this (meters from ground)
sensor_height: 0.63           ← body frame height above ground

Internal threshold = max_height_from_ground - sensor_height
                   = 2.0 - 0.63 = 1.37m (body frame z)

Points with body_z > 1.37m are removed
```

### Design Decisions

- **No lower bound**: Slopes and downhill terrain can produce points well below the body frame. A lower bound would incorrectly remove ground points on steep terrain.
- **Default ON** (`enable_height_filter: true`): The robot primarily operates in indoor/tunnel environments where ceiling filtering is needed.
- **Ground-referenced**: Users set `max_height_from_ground` in intuitive ground-relative meters instead of body-frame-relative values.

## Parameters

### cloud_merger.yaml

| Parameter | Default | Description |
|-----------|---------|-------------|
| `m300_topic` | `/cloud_registered_body` | M300 LiDAR input topic |
| `s10_topic` | `/lx_camera_node/LxCamera_Cloud` | S10 Ultra input topic |
| `merged_topic` | `/merged_cloud` | Merged output topic |
| `output_frame` | `body` | Output frame ID |
| `s10_stale_threshold` | 0.5 | S10 data staleness limit [s] |
| `enable_height_filter` | true | Enable ceiling point removal |
| `max_height_from_ground` | 2.0 | Max height from ground [m] |
| `sensor_height` | 0.63 | Body frame height above ground [m] |

### Parameter Modification

```bash
# 1. YAML file (permanent) — edit param/cloud_merger.yaml
max_height_from_ground: 2.5

# 2. Launch argument (per-run override)
ros2 launch cloud_merger cloud_merger.launch.py max_height_from_ground:=2.5

# 3. Runtime (dynamic, no restart)
ros2 param set /cloud_merger max_height_from_ground 2.5
```

### Environment Presets

```bash
# Indoor / Tunnel (default) — ceiling filter ON
ros2 launch cloud_merger cloud_merger.launch.py

# Low tunnel — lower threshold
ros2 launch cloud_merger cloud_merger.launch.py max_height_from_ground:=1.8

# Outdoor — ceiling filter OFF
ros2 launch cloud_merger cloud_merger.launch.py enable_height_filter:=false
```

## Build & Run

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select cloud_merger

# Run
source install/setup.bash
ros2 launch cloud_merger cloud_merger.launch.py
```

## Monitoring

```bash
# Check merged output rate (expect ~10Hz)
ros2 topic hz /merged_cloud

# Check merged point count
ros2 topic echo /merged_cloud --field width --once

# Check height filter status
ros2 param get /cloud_merger enable_height_filter
ros2 param get /cloud_merger max_height_from_ground

# View startup log for filter config
# Expected: "Height filter enabled: max 2.00m from ground (body z <= 1.37m), sensor_height=0.63m"
```

## File Structure

```
cloud_merger/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── cloud_merger.launch.py    # Node + static TF publisher
├── param/
│   └── cloud_merger.yaml         # Default parameters
└── src/
    └── cloud_merger_node.cpp     # Merge + height filter logic
```

## Processing Pipeline

```
M300 (/cloud_registered_body)  ──→ ┐
    PointXYZINormal, body frame    │
                                   ├──→ Merge ──→ Height Filter ──→ /merged_cloud
S10 (/lx_camera_node/...)     ──→ ┘         (PointXYZI, body frame)
    PointXYZRGB, mrdvs_tof frame
    → TF transform to body
    → RGB to intensity (luminance)
    → staleness check (wall-clock)
```

## Clock Domain Note

The M300 uses a **sensor-internal clock** (via FAST-LIO2), while the S10 Ultra uses the **system clock**. Direct timestamp comparison is unreliable, so staleness is checked using **wall-clock receive time** (`steady_clock`) rather than message header stamps.