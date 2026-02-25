# terrain_costmap

Robot-centric traversability costmap from GroundGrid output for caterpillar (skid-steer) robots on mountain terrain.

## Pipeline Position

```
GroundGrid ──→ terrain_costmap ──→ Path Planner
  │                │
  ├─ grid_map      ├─ OccupancyGrid (0~100)
  └─ filtered_cloud└─ terrain_cloud (debug)
```

---

## Cost Calculation

### Cell Values

| Value | Meaning |
|-------|---------|
| `-1` | Unknown (no data) |
| `0` | Free (flat, smooth, high confidence) |
| `1 ~ 99` | Increasing traversal difficulty |
| `100` | Lethal (impassable) |

### Cost Factors

| Factor | Source Layer | Weight | Normalization | Saturation |
|--------|------------|--------|---------------|------------|
| Obstacle | `points` | 0.40 | point_count / 5 | ≥ 5 points → 1.0 |
| Slope | `ground` | 0.30 | atan(‖∇z‖) / 35° | ≥ 35° → 1.0 |
| Roughness | `variance` | 0.15 | σ² / 0.05 | ≥ 0.05 m² → 1.0 |
| Unknown | `groundpatch` | 0.15 | confidence < 0.1 ? 1 : 0 | binary |

### Formula

```
total = 0.40 × obstacle + 0.30 × slope + 0.15 × roughness + 0.15 × unknown
```

```
             ┌ 100              if obstacle ≥ 0.99
cell_value = ├ 100              if total ≥ 0.80  (lethal_threshold)
             ├ round(total × 99) if total < 0.80
             └ -1               if no data
```

**Slope** is computed via central differences on the `ground` elevation layer:

```
dz/dx = (z[i+1][j] - z[i-1][j]) / (2 × resolution)
dz/dy = (z[i][j+1] - z[i][j-1]) / (2 × resolution)
slope  = atan(√((dz/dx)² + (dz/dy)²))
```

### Examples

| Scenario | Obstacle | Slope | Roughness | Confidence | Total | Cell |
|----------|----------|-------|-----------|------------|-------|------|
| Flat dirt path | 0 pts | 5° | 0.01 m² | 0.8 | 0.073 | **7** |
| Moderate slope | 0 pts | 25° | 0.04 m² | 0.5 | 0.334 | **33** |
| Rock | 8 pts | 40° | 0.10 m² | 0.9 | — | **100** (instant lethal) |
| Accumulated fallback | — | 3° | 0.008 m² | — | 0.050 | **5** |

---

## Terrain Accumulation

### Why

M300 LiDAR and S10 Ultra cover primarily the front. When the robot moves forward, the area behind exits sensor view and becomes unknown (-1). Accumulation preserves previously observed ground.

### How

```
filtered_cloud (body frame)
  → extract ground points (intensity ≈ 49)
  → transform to odom frame (world-fixed)
  → append to terrain_cloud_ buffer
  → prune: remove points > 10m from robot
  → prune: remove oldest if > 200,000 points
```

During costmap generation, unknown cells query this buffer:

```
For each cell where cell_value == -1:
  bin terrain_cloud_ points into cell
  if count ≥ 2:
    cost = 0.30 × slope + 0.15 × roughness   (obstacle=0, unknown=0)
  else:
    remain -1
```

---

## Parameters

### Costmap Geometry

| Parameter | Default | Description |
|-----------|---------|-------------|
| `costmap_size` | 3.0 | Side length [m] |
| `costmap_resolution` | 0.1 | Cell size [m] |
| `robot_frame` | base_link | Output frame |
| `odom_frame` | odom | World-fixed frame |

---

## Build & Launch

```bash
colcon build --packages-select terrain_costmap
source install/setup.bash

# default
ros2 launch terrain_costmap terrain_costmap.launch.py

# custom
ros2 launch terrain_costmap terrain_costmap.launch.py costmap_size:=5.0 costmap_resolution:=0.2
```
