# Remote Monitoring Guide

> Visualize IQ-9075 (ROS2 Jazzy) sensor data with rviz2 on Host PC

## Background

- IQ-9075: **ROS2 Jazzy** / Host PC: **ROS2 Humble**
- Jazzy and Humble are **NOT DDS-compatible** -- direct communication fails
- Solution: Run a **ROS2 Jazzy Docker container** on the Host PC
- rviz2 runs inside Docker, displayed on Host PC screen via X11

## Requirements

Three settings **must match** on both sides for communication:

| Setting | Value |
|---------|-------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `ROS_DOMAIN_ID` | `64` |
| `CYCLONEDDS_URI` | Path to `cyclonedds.xml` containing each device's WiFi IP |

- Without `cyclonedds.xml`, DDS may use the wrong interface (e.g. sensor Ethernet) and discovery fails

## Files

```
docs/docker/
  Dockerfile.jazzy-cyclone    # Docker image: ROS2 Jazzy + CycloneDDS
  cyclonedds.xml              # DDS config template (change IP only)
  monitor.rviz                # Saved rviz2 layout (auto-loaded on launch)
  run_jazzy.sh                # Container launch script
```

---

## Host PC Setup

### 1. Install Docker (once)

```bash
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
# Log out and back in
```

### 2. Build Docker image (once)

```bash
cd <repo>/docs/docker/
docker build -f Dockerfile.jazzy-cyclone -t jazzy-desktop-cyclone .
```

### 3. Edit cyclonedds.xml

Find your Host PC WiFi IP:
```bash
ip addr show | grep "inet 192"
```

Edit `<repo>/docs/docker/cyclonedds.xml` -- replace `NetworkInterfaceAddress` with your WiFi IP:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>HOST_PC_WIFI_IP</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

- Example: `192.168.0.34`

### 4. Launch

```bash
cd <repo>/docs/docker/
chmod +x run_jazzy.sh    # once
./run_jazzy.sh
```

`run_jazzy.sh` launches rviz2 automatically with the saved `monitor.rviz` config.

---

## IQ-9075 Setup

### 1. Install CycloneDDS (once)

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### 2. Create cyclonedds.xml

Find IQ-9075 WiFi IP:
```bash
ip addr show wlp1s0 | grep "inet "
```

Copy `<repo>/docs/docker/cyclonedds.xml` to `~/cyclonedds.xml` and change the IP to IQ-9075's WiFi IP:

```bash
cp <repo>/docs/docker/cyclonedds.xml ~/cyclonedds.xml
nano ~/cyclonedds.xml
```

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>IQ9075_WIFI_IP</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

- Example: `192.168.0.78`

### 3. Set environment variables (persistent)

```bash
sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc
sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc
sed -i '/CYCLONEDDS_URI/d' ~/.bashrc

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=64' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc

source ~/.bashrc
```

### 4. Verify

```bash
echo $RMW_IMPLEMENTATION    # rmw_cyclonedds_cpp
echo $ROS_DOMAIN_ID         # 64
echo $CYCLONEDDS_URI        # file:///home/ubuntu/cyclonedds.xml
```

- All three must print values
- If any is blank, repeat Step 3

---

## Verification

IQ-9075:
```bash
ros2 topic pub /test std_msgs/String '{data: "hello"}' --rate 1
```

Host PC (inside Docker):
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list        # /test should appear
ros2 topic echo /test  # should print: data: "hello"
```

---

## rviz2 Config

### Save

`run_jazzy.sh` includes `--rm` flag, so the container is deleted on exit. Save the config **before exiting**:

1. In rviz2: File > Save Config As > `/tmp/monitor.rviz`
2. Open a **new terminal on Host PC** (do not exit the container):

```bash
docker cp $(docker ps -q):/tmp/monitor.rviz <repo>/docs/docker/monitor.rviz
```

### Update

To update the saved config, repeat the same steps. The new `monitor.rviz` will overwrite the old one.

### Launch behavior

- `run_jazzy.sh` mounts `<repo>/docs/docker/monitor.rviz` into the container
- rviz2 launches automatically with this config
- If `monitor.rviz` does not exist, rviz2 opens with default settings