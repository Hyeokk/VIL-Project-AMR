# AMR Segmentation

Camera-based semantic segmentation pipeline for autonomous mountain terrain navigation.

DDRNet23-Slim (FP16) runs on Qualcomm Dragonwing IQ-9075 NPU via ONNX Runtime QNN EP.
RGB segmentation mask is fused with M300 LiDAR and S10 Ultra depth to produce a 6m×6m semantic costmap.

Model trained in [TerrainSegmentation](https://github.com/Hyeokk/VIL-Project-TerrainSegmentation) (datasets, hyperparameters, model selection).

---

## 7-Class Semantic Ontology

| ID | Class | Costmap Cost | Traversability |
|----|-------|:------------:|----------------|
| 0 | Smooth Ground | 0 | Free |
| 1 | Rough Ground | 40 | Mild penalty |
| 2 | Bumpy Ground | 60 | Moderate penalty |
| 3 | Obstacle | 100 | Lethal |
| 4 | Vegetation | 30 | Low penalty |
| 5 | Sky | 0 | Ignored |
| 6 | Other | 80 | Conservative |

Points outside camera FOV are assigned classes via geometric fallback (height-based rules).

---

## Deployment Summary

| | INT8 QDQ | **FP16 (Deployed)** |
|---|---|---|
| Model | ddrnet23_slim_int8_qdq_nearest.onnx | ddrnet23_slim_unified7class_544x640_nearest.onnx |
| NPU graph | 8 subgraphs (7 NPU-CPU transfers) | Single graph (fully on NPU) |
| Latency | ~62ms (15.9 FPS) | ~10ms (100+ FPS) |

FP16 is 4.3x faster because it runs as a single NPU graph. INT8 QDQ is split into 8 subgraphs by ONNX Runtime, causing repeated NPU-CPU data transfers that dominate latency.

---

## Quick Start

### IQ-9075 Setup (one-time)

```bash
sudo apt-add-repository -y ppa:ubuntu-qcom-iot/qcom-ppa
sudo apt update && sudo apt install -y libqnn1 libsnpe1 libqnn-dev libsnpe-dev
sudo apt install -y clinfo qcom-adreno1
sudo ln -sf /lib/aarch64-linux-gnu/libOpenCL.so.1.0.0 /usr/lib/libOpenCL.so

python3 -m venv ~/ort_env && source ~/ort_env/bin/activate
pip install onnxruntime-qnn numpy opencv-python-headless
```

### Model

The deployed model is `ddrnet23_slim_unified7class_544x640_nearest.onnx` (FP16). All Resize ops in the original ONNX graph use bilinear interpolation, but QAIRT 2.40 HTP does not support bilinear Resize. This causes the graph to be split into multiple subgraphs with costly NPU-CPU transfers. The nearest-mode variant patches all Resize ops to nearest-neighbor, allowing the entire model to run on the NPU as a single graph (~10ms vs ~62ms).

### Launch

```bash
# Default launch (NPU enabled, default camera mount)
ros2 launch amr_segmentation pipeline.launch.py

# Custom camera mount position
ros2 launch amr_segmentation pipeline.launch.py cam_x:=0.40 cam_z:=0.30

# CPU-only inference (no NPU)
ros2 launch amr_segmentation pipeline.launch.py use_qnn:=false
```

---

## Segmentation Test (infer_test.py)

Standalone test script for the segmentation model without the full ROS2 pipeline.
Model path is fixed to `models/ddrnet23_slim_unified7class_544x640_nearest.onnx`.

### Video Mode (no ROS2 required)

Test with a recorded video file.

```bash
# Display overlay on screen
python3 scripts/infer_test.py video --input test.mp4

# Save result to file
python3 scripts/infer_test.py video --input test.mp4 --output result.mp4

# CPU-only inference (no NPU)
python3 scripts/infer_test.py video --input test.mp4 --no-qnn
```

### Camera Mode (ROS2 + S10 Ultra required)

Test with live MRDVS S10 Ultra camera feed (`/lx_camera_node/LxCamera_Rgb`).

```bash
# Display overlay on screen
python3 scripts/infer_test.py camera

# Save result to file
python3 scripts/infer_test.py camera --output result.mp4

# CPU-only inference (no NPU)
python3 scripts/infer_test.py camera --no-qnn
```

Both modes display segmentation overlay, inference time (ms), FPS, and class legend. Press `q` to quit.

---

## Inference Benchmark (benchmark_infer.py)

Per-stage latency profiling with statistics. Separates **preprocess / inference / postprocess** timings and reports min, max, mean, p50, p95, p99, std. Warmup frames are excluded from statistics to avoid NPU first-compile skew.

### Video Mode (no ROS2 required)

```bash
# Basic benchmark (300 frames, 20 warmup)
python3 scripts/benchmark_infer.py video --input test.mp4

# Custom frame count and warmup
python3 scripts/benchmark_infer.py video --input test.mp4 --frames 500 --warmup 30

# Save overlay video (segmentation mask + per-stage latency + FPS)
python3 scripts/benchmark_infer.py video --input test.mp4 --output result.mp4

# QNN (NPU) vs CPU automatic comparison
python3 scripts/benchmark_infer.py video --input test.mp4 --compare

# Save statistics to JSON
python3 scripts/benchmark_infer.py video --input test.mp4 --save results.json
```

### Camera Mode (ROS2 + S10 Ultra required)

```bash
# Live benchmark from S10 Ultra feed
python3 scripts/benchmark_infer.py camera

# Save overlay video + JSON stats
python3 scripts/benchmark_infer.py camera --output result.mp4 --save results.json

# CPU-only
python3 scripts/benchmark_infer.py camera --no-qnn
```

### Output

Terminal prints a summary table after measurement completes:

```
  ┌── QNN (NPU)
  │  Measured frames : 280
  │  Effective FPS   : 98.3
  │
  │  Stage            Mean     Std     Min     P50     P95     P99     Max
  │  ────────────────────────────────────────────────────────────────────
  │  Preprocess       1.23    0.15    1.02    1.21    1.48    1.62    1.85
  │  Inference        8.94    0.43    8.21    8.87    9.68   10.12   11.34
  │  Postprocess      0.01    0.00    0.01    0.01    0.02    0.02    0.03
  │  ────────────────────────────────────────────────────────────────────
  │  ▶ TOTAL         10.18    0.52    9.31   10.12   11.05   11.68   13.02
  └──────────────────────────────────────────────────────────────────────
```

`--output result.mp4` overlays segmentation mask, per-stage latency (pre/infer/post), total latency, and FPS on each frame. Warmup frames are not written to the video.

---

## Preprocessing

The ONNX model expects ImageNet-normalized input, not raw camera pixels. All inference scripts handle this automatically via `segmentation_common.py`. For custom inference code, apply:

| Step | Operation | Result |
|------|-----------|--------|
| 1 | BGR to RGB | RGB uint8 |
| 2 | Resize to 640 × 544 (W × H), nearest | RGB uint8 |
| 3 | Divide by 255.0 | float32, 0.0–1.0 |
| 4 | Subtract mean [0.485, 0.456, 0.406] | float32 |
| 5 | Divide by std [0.229, 0.224, 0.225] | float32, normalized |
| 6 | HWC to NCHW | shape (1, 3, 544, 640) |

Feeding raw pixels (0–255) directly produces meaningless output.

---

## Calibration

No hardcoded calibration files. All obtained from existing infrastructure.

| Parameter | Source | Method |
|-----------|--------|--------|
| RGB Intrinsics (K) | S10 Ultra SDK | `/lx_camera_node/LxCamera_RgbInfo` topic |
| ToF-RGB Extrinsic | S10 Ultra SDK | TF: `mrdvs_tof` → `mrdvs_rgb` |
| Camera-Robot | pipeline.launch.py | TF: `base_link` → `mrdvs_tof` (static, configurable) |
| LiDAR-Robot | FAST-LIO2 | TF: `odom` → `base_link` → `body` |

---

## Configuration

All pipeline parameters are defined in `config/pipeline_params.yaml`.

```yaml
segmentation_node:
  ros__parameters:
    model_path: "~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx"
    use_qnn: true
    image_topic: "/lx_camera_node/LxCamera_Rgb"
    publish_visualization: true

pointcloud_painter_node:
  ros__parameters:
    lidar_topic: "/cloud_registered_body"
    seg_topic: "/segmentation_node/segmentation"
    caminfo_topic: "/lx_camera_node/LxCamera_RgbInfo"
    camera_frame: "mrdvs_rgb"
    enable_geometric_fallback: true
    tf_timeout_sec: 1.0

semantic_merger_node:
  ros__parameters:
    painted_topic: "/pointcloud_painter_node/painted_pointcloud"
    s10_cloud_topic: "/lx_camera_node/LxCamera_Cloud"
    seg_topic: "/segmentation_node/segmentation"
    caminfo_topic: "/lx_camera_node/LxCamera_RgbInfo"
    robot_frame: "body"
    grid_size_m: 6.0
    resolution: 0.1
    z_min: -1.0
    z_max: 2.0
```

Camera mount position is adjustable via launch arguments.

```bash
ros2 launch amr_segmentation pipeline.launch.py cam_x:=0.40 cam_z:=0.30
```

---

## Structure

```
amr_segmentation/
├── amr_segmentation/               # Python modules
│   ├── __init__.py
│   ├── segmentation_common.py      # 7-class constants, preprocessing, ORT session
│   ├── segmentation_node.py        # RGB → segmentation mask (NPU)
│   ├── pointcloud_painter_node.py  # M300 LiDAR + mask → painted pointcloud
│   └── semantic_merger_node.py     # Painted cloud + S10 depth → costmap
├── config/
│   └── pipeline_params.yaml        # All node parameters
├── launch/
│   └── pipeline.launch.py          # Static TF + 3 nodes
├── models/
│   └── *.onnx                      # ONNX models (gitignored)
├── scripts/
│   ├── infer_test.py               # Offline test (video / camera)
│   └── benchmark_infer.py          # Per-stage latency profiling
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Migration from Previous Packages

This package replaces three previous packages: `groundgrid`, `terrain_costmap`, and `cloud_merger`.

| Before | After | Notes |
|--------|-------|-------|
| `groundgrid` | `segmentation_node` | 2-class → 7-class segmentation |
| `terrain_costmap` | `semantic_merger_node` | Height-only → semantic costmap |
| `cloud_merger` (static TF) | `pipeline.launch.py` | TF `base_link→mrdvs_tof` migrated |

Update the path planner costmap topic accordingly.

```yaml
# path_planner/param/path_planner.yaml
costmap_topic: "/semantic_merger_node/semantic_costmap"  # was /terrain_costmap
```