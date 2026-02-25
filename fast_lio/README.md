# Fast Lio VIL (Elevation Mapping & Navigation)

This package integrates Fast-LIO2 with an elevation mapping node to generate 2.5D maps for navigation. It accumulates historical map data to create a persistent "trace" of the robot's path.

## Dependencies

- ROS 2 Humble (or compatible)
- PCL (Point Cloud Library)
- Eigen3
- **Livox ROS Driver 2**: Included in `src/` (ensure it is built).

## Build Instructions

1.  **Clone the repository**:
    ```bash
    mkdir -p dev_ws/src
    cd dev_ws/src
    git clone <your-repo-url>
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/dev_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build**:
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

4.  **Source**:
    ```bash
    source install/setup.bash
    ```

## Usage

### 1. Launch Robot & Mapping
```bash
ros2 launch lio_nav_bringup bringup.launch.py
```

### 2. Run with Bag File (Simulation)
```bash
ros2 bag play path/to/your/bag --clock
```

## Parameters

Key parameters in `launch/elevation_mapping.launch.py`:
- `obstacle_height_thresh`: **0.3** (Default). Height difference to consider as an obstacle.
- `grid_resolution`: **0.2**. Size of each grid cell (meters).
- `local_map_size`: **30.0**. Size of the sliding window for active mapping.
