# Network Configuration Guide

> IQ-9075 EVK: WiFi (SSH) + LiDAR + Camera simultaneous operation

## Physical Setup

```
IQ-9075 (end0) ──RJ45──> Gigabit Switch ──> LiDAR  (192.168.158.98)
                                          ──> Camera (192.168.30.82)
IQ-9075 (wlp1s0) ──WiFi──> Router (SSH / Internet)
```

The board has a single Ethernet port (`end0`), so a **gigabit switch** is required to connect both sensors.

## IP Assignment

| Interface | IP Address | Purpose |
|-----------|-----------|---------|
| `wlp1s0` (WiFi) | DHCP (e.g. 192.168.0.78) | SSH, Internet -- **highest priority** |
| `end0` (Ethernet) | `192.168.158.15/24` | LiDAR communication (M300-E default upload target) |
| `end0` (Ethernet) | `192.168.30.100/24` | Camera communication |

## Configuration

### 1. Create netplan file

```bash
sudo nano /etc/netplan/60-sensors.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    end0:
      dhcp4: false
      dhcp6: false
      addresses:
        - 192.168.158.15/24
        - 192.168.30.100/24
```

### 2. Apply

```bash
sudo chmod 600 /etc/netplan/60-sensors.yaml
sudo netplan generate
sudo netplan apply
```

Existing WiFi config files (`90-NM-*.yaml`) are untouched.

### 3. UDP buffer for LiDAR (200K pts/sec)

```bash
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.d/99-lidar-udp.conf
echo "net.core.rmem_default=26214400" | sudo tee -a /etc/sysctl.d/99-lidar-udp.conf
sudo sysctl -p /etc/sysctl.d/99-lidar-udp.conf
```

## Routing Priority

The key design: **no default route on `end0`**. Only WiFi has a default gateway, so all internet/SSH traffic always goes through WiFi.

```bash
ip route show
```

Expected:
```
default via 192.168.0.1 dev wlp1s0 proto dhcp src 192.168.0.78 metric 600   <-- WiFi (default)
192.168.30.0/24 dev end0 proto kernel scope link src 192.168.30.100          <-- Camera subnet
192.168.158.0/24 dev end0 proto kernel scope link src 192.168.158.15         <-- LiDAR subnet
```

Verify which interface handles each destination:

```bash
ip route get 8.8.8.8           # --> dev wlp1s0 (internet via WiFi)
ip route get 192.168.158.98    # --> dev end0 (LiDAR via Ethernet)
ip route get 192.168.30.82     # --> dev end0 (Camera via Ethernet)
```

| Scenario | SSH (WiFi) | Sensors (Ethernet) |
|----------|-----------|-------------------|
| Ethernet cable unplugged | OK | Down |
| WiFi disconnected | Down | OK |
| Both connected | OK | OK |

## Connection Test

```bash
ip addr show end0                          # Should show 2 IPs
ping -c 2 192.168.158.98                   # LiDAR
ping -c 2 192.168.30.82                    # Camera
ping -c 2 8.8.8.8                          # Internet via WiFi
sudo tcpdump -i end0 -c 5 udp port 6668   # LiDAR UDP data
sudo tcpdump -i end0 -c 5 host 192.168.30.82  # Camera packets
```

`end0` shows `NO-CARRIER / state DOWN` when no Ethernet cable is connected. This is normal -- it comes up automatically when plugged into the switch.

## Troubleshooting

| Issue | Command | Fix |
|-------|---------|-----|
| No IP on end0 | `ip addr show end0` | Check cable connection to switch |
| SSH lost | `ip route \| grep default` | Ensure default route is on `wlp1s0` |
| No LiDAR data | `sudo tcpdump -i end0 udp port 6668 -c 5` | Verify board IP is `192.168.158.15` (must match M300-E upload target) |
| No Camera data | `sudo tcpdump -i end0 host 192.168.30.82 -c 5` | Check subnet match |
| Firewall blocking | `sudo ufw status` | `sudo ufw disable` for testing |