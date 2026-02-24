# Remote Monitoring Guide

> Visualize IQ-9075 (ROS2 Jazzy) sensor data with rviz2 on a laptop

## Problem

- IQ-9075 runs **ROS2 Jazzy**
- Laptop runs **Ubuntu 22.04 / ROS2 Humble**
- Humble and Jazzy are **NOT cross-compatible** over DDS
- SSH alone cannot display rviz2 efficiently

## Solution

Run a **ROS2 Jazzy Docker container** with **CycloneDDS** on the laptop. The container shares the host network (`--net=host`), discovers IQ-9075 topics over WiFi, and renders rviz2 on the laptop display via X11.

```
 Laptop (Ubuntu 22.04)                    IQ-9075
+-----------------------------+          +--------------+
|  Docker (ROS2 Jazzy)        |   WiFi   |  ROS2 Jazzy  |
|  CycloneDDS, DOMAIN_ID=7   | <------> |  CycloneDDS  |
|  rviz2 / rqt / topic echo  |   DDS    |  DOMAIN_ID=7 |
+-------------+---------------+          +--------------+
              | X11
              v
        Laptop Display
```

## Files

```
docs/docker/
  Dockerfile.jazzy-cyclone   # Jazzy + CycloneDDS image
  run_jazzy.sh               # Container launch script
```

---

## Laptop Setup

### 1. Build Docker image

```bash
cd docs/docker
docker build -f Dockerfile.jazzy-cyclone -t jazzy-desktop-cyclone .
```

`Dockerfile.jazzy-cyclone`:
```dockerfile
FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds \
 && rm -rf /var/lib/apt/lists/*
```

### 2. Launch container

```bash
./run_jazzy.sh
```

`run_jazzy.sh`:
```bash
#!/bin/bash
xhost +local:root

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=7 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  jazzy-desktop-cyclone
```

### 3. Inside the container

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list       # verify IQ-9075 topics appear
rviz2                 # launch visualization
```

---

## IQ-9075 Setup

Both machines must match **RMW** and **DOMAIN_ID**.

### 1. Install CycloneDDS

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### 2. Set environment variables

Add to `~/.bashrc`:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
```

Then reload:

```bash
source ~/.bashrc
```

### 3. Verify DDS uses WiFi interface

If topics are not discovered, restrict CycloneDDS to the WiFi interface:

```bash
cat << 'EOF' > ~/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlp1s0" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
```

Add to `~/.bashrc`:

```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

This ensures DDS discovery traffic goes through WiFi (`wlp1s0`), not the sensor Ethernet (`end0`).

---

## Usage

### View point cloud in rviz2

Inside the Docker container:

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

In rviz2:
1. Add > By topic > select PointCloud2 topic
2. Set Fixed Frame to the appropriate frame (e.g. `odom` or `body`)

### Monitor from terminal

```bash
ros2 topic list
ros2 topic hz /cloud_registered_body
ros2 topic echo /imu/data --once
rqt
```

---

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| `ros2 topic list` empty | Same WiFi? Same DOMAIN_ID? Same RMW? | Verify all three match |
| rviz2 crash / black screen | GPU driver | Remove `-v /dev/dri:/dev/dri` or add `-e QT_QUICK_BACKEND=software` |
| "cannot open display" | X11 not forwarded | Run `xhost +local:root` on host |
| Topics visible but no data | DDS on wrong interface | Apply `cyclonedds.xml` (IQ-9075 Setup Step 3) |
| High latency on point cloud | WiFi bandwidth | Reduce publish rate or downsample |