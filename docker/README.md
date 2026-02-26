# Remote Monitoring Guide

> Visualize IQ-9075 (ROS2 Jazzy) sensor data with rviz2 on Host PC

## Background

- IQ-9075: **ROS2 Jazzy** / Host PC: **ROS2 Humble**
- Jazzy and Humble are **NOT DDS-compatible** -- direct communication fails
- Solution: Run a **ROS2 Jazzy Docker container** on the Host PC
- rviz2 runs inside Docker, displayed on Host PC screen via X11

## Connection Methods

| Method | Host PC ↔ IQ-9075 | DDS Interface | Use Case |
|--------|-------------------|---------------|----------|
| **WiFi** | Same WiFi network | WiFi IP (e.g. `192.168.0.x`) | Quick setup, no cable needed |
| **Wired** | Ethernet via switch | `10.6.4.x` subnet | Stable, low-latency, no WiFi dependency |

## Requirements

Three settings **must match** on both sides for communication:

| Setting | Value |
|---------|-------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `ROS_DOMAIN_ID` | `64` |
| `CYCLONEDDS_URI` | Path to `cyclonedds.xml` containing the interface IP used for DDS |

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

Choose the IP that matches your connection method.

#### Option A: WiFi

Find your Host PC WiFi IP:
```bash
ip addr show | grep "inet 192"
```

Set `NetworkInterfaceAddress` to your WiFi IP (e.g. `192.168.0.34`):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>192.168.0.34</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

#### Option B: Wired (Ethernet)

Host PC must have a static IP on the `10.6.4.0/24` subnet. Set via GUI or CLI:

**GUI**: Settings → Network → Wired → IPv4 → Manual
- Address: `10.6.4.2`, Netmask: `255.255.255.0`, Gateway: *leave empty*

**CLI** (find your interface name with `ip link show`):
```bash
sudo nano /etc/netplan/99-wired-board.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enx00e04c680017:         # ← replace with your interface name
      dhcp4: false
      dhcp6: false
      addresses:
        - 10.6.4.2/24
      # No gateway -- internet stays on WiFi
```

```bash
sudo chmod 600 /etc/netplan/99-wired-board.yaml
sudo netplan apply
```

Set `NetworkInterfaceAddress` to `10.6.4.2`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>10.6.4.2</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

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

### 2. Configure netplan (once)

IQ-9075 has a single Ethernet port (`end0`) shared by LiDAR, Camera, and (optionally) Host PC via a gigabit switch. Each device lives on its own subnet:

| Subnet | IP on end0 | Target Device |
|--------|-----------|---------------|
| `192.168.1.0/24` | `192.168.1.5` | LiDAR (M300-E default upload target) |
| `192.168.30.0/24` | `192.168.30.100` | Camera (S10 Ultra) |
| `10.6.4.0/24` | `10.6.4.1` | Host PC (SSH + DDS, wired only) |

#### Check current config

```bash
sudo nano /etc/netplan/60-sensors.yaml
```

Review the current settings, then exit without changes: `Ctrl+X`

#### WiFi only (no wired Host PC)

```bash
sudo tee /etc/netplan/60-sensors.yaml << 'EOF'
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    end0:
      addresses:
      - "192.168.1.5/24"
      - "192.168.30.100/24"
      dhcp4: false
      dhcp6: false
      networkmanager:
        passthrough:
          ipv4.never-default: "true"
EOF
sudo chmod 600 /etc/netplan/60-sensors.yaml
sudo netplan generate
sudo netplan apply
```

#### Wired Host PC (add `10.6.4.1`)

```bash
sudo tee /etc/netplan/60-sensors.yaml << 'EOF'
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    end0:
      addresses:
      - "192.168.1.5/24"
      - "192.168.30.100/24"
      - "10.6.4.1/24"
      dhcp4: false
      dhcp6: false
      networkmanager:
        passthrough:
          ipv4.never-default: "true"
EOF
sudo chmod 600 /etc/netplan/60-sensors.yaml
sudo netplan generate
sudo netplan apply
```

#### Verify

```bash
ip addr show end0 | grep inet
```

WiFi only (2 IPs):
```
inet 192.168.1.5/24 ...       ← LiDAR
inet 192.168.30.100/24 ...    ← Camera
```

Wired Host PC (3 IPs):
```
inet 192.168.1.5/24 ...       ← LiDAR
inet 192.168.30.100/24 ...    ← Camera
inet 10.6.4.1/24 ...          ← Host PC
```

> `ipv4.never-default: "true"` ensures no default gateway is added on `end0`. All internet/SSH traffic uses WiFi (`wlp1s0`) when WiFi is connected.

### 3. Create cyclonedds.xml

```bash
cp <repo>/docs/docker/cyclonedds.xml ~/cyclonedds.xml
nano ~/cyclonedds.xml
```

#### Option A: WiFi

Find IQ-9075 WiFi IP:
```bash
ip addr show wlp1s0 | grep "inet "
```

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>192.168.0.78</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

#### Option B: Wired (Ethernet)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>10.6.4.1</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

### 4. Set environment variables (persistent)

```bash
sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc
sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc
sed -i '/CYCLONEDDS_URI/d' ~/.bashrc

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=64' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc

source ~/.bashrc
```

### 5. Verify

```bash
echo $RMW_IMPLEMENTATION    # rmw_cyclonedds_cpp
echo $ROS_DOMAIN_ID         # 64
echo $CYCLONEDDS_URI        # file:///home/ubuntu/cyclonedds.xml
```

- All three must print values
- If any is blank, repeat Step 4

---

## Switching Between WiFi and Wired

Only `cyclonedds.xml` needs to change on **both** sides:

| Mode | IQ-9075 `cyclonedds.xml` | Host PC `cyclonedds.xml` |
|------|--------------------------|--------------------------|
| WiFi | IQ-9075 WiFi IP (e.g. `192.168.0.78`) | Host PC WiFi IP (e.g. `192.168.0.34`) |
| Wired | `10.6.4.1` | `10.6.4.2` |

After editing, restart ROS nodes on IQ-9075 and relaunch `run_jazzy.sh` on Host PC.

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

For wired connection, also verify basic connectivity first:
```bash
# Host PC
ping -c 3 10.6.4.1

# IQ-9075
ping -c 3 10.6.4.2
```

---

## rviz2 Config

### Save

`run_jazzy.sh` includes `--rm` flag, so the container is deleted on exit. Save the config **before exiting**:

1. In rviz2: File > Save Config As > `/monitor.rviz` (root path)
2. Open a **new terminal on Host PC** (do not exit the container):

```bash
docker cp $(docker ps -q):/monitor.rviz <repo>/docs/docker/monitor.rviz
```

### Re-save (overwrite existing config)

`/tmp/monitor.rviz` is mounted from the host and cannot be deleted inside the container. To re-save:

1. Delete old rviz files inside the container:
```bash
docker exec $(docker ps -q) find / -name "*.rviz" 2>/dev/null
docker exec $(docker ps -q) rm /monitor.rviz    # delete any non-mounted copies
```

2. In rviz2: File > Save Config As > `/monitor.rviz`

3. Copy to host:
```bash
docker cp $(docker ps -q):/monitor.rviz <repo>/docs/docker/monitor.rviz
```

### Launch behavior

- `run_jazzy.sh` mounts `<repo>/docs/docker/monitor.rviz` into the container as `/tmp/monitor.rviz`
- rviz2 launches automatically with this config
- If `monitor.rviz` does not exist, rviz2 opens with default settings

---

## Troubleshooting

### Ethernet link reset

If the Ethernet connection stops working after a cable or switch hot-unplug (e.g. switching hub disconnected during operation), the network interface may enter a stale state. Symptoms include `Destination Host Unreachable` even though both sides show correct IPs.

**Fix** -- reset the interface on both sides:

IQ-9075:
```bash
sudo ip link set end0 down && sudo ip link set end0 up
```

Host PC:
```bash
sudo ip link set <interface> down && sudo ip link set <interface> up
# e.g. sudo ip link set enx00e04c680017 down && sudo ip link set enx00e04c680017 up
```

If still not working, re-apply netplan on IQ-9075:
```bash
sudo netplan apply
```