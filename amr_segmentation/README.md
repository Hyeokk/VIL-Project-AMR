# AMR Segmentation

DDRNet23-Slim semantic segmentation deployed to Qualcomm Dragonwing IQ-9075 NPU as FP16.

Model trained in [TerrainSegmentation](https://github.com/Hyeokk/VIL-Project-TerrainSegmentation) (datasets, hyperparameters, model selection).

---

## Deployment Summary

| | INT8 QDQ | **FP16 (Deployed)** |
|---|---|---|
| Model | ddrnet23_slim_int8_qdq_nearest.onnx | ddrnet23_slim_unified7class_544x640_nearest.onnx |
| NPU graph | 8 subgraphs (7 NPU-CPU transfers) | Single graph (fully on NPU) |
| Latency | ~62ms (15.9 FPS) | ~10ms (100+ FPS) |

FP16 is 4.3x faster because it runs as a single NPU graph. INT8 QDQ is split into 8 subgraphs by ONNX Runtime, causing repeated NPU-CPU data transfers that dominate latency.

---

## Pipeline

| Step | Script | Output |
|------|--------|--------|
| 1. ONNX Export | export_onnx.py | FP32 ONNX (bilinear) |
| 2. Patch Resize | patch_resize_nearest.py | FP32 ONNX (nearest) |
| 3. Deploy | scp to IQ-9075 | FP16 on NPU |

Step 2 is required because QAIRT 2.40 HTP does not support bilinear Resize. The patch converts all Resize ops to nearest mode, enabling the entire model to run on the NPU as a single graph.

INT8 quantization (quantize_onnx.py) is available but not recommended for this model. See [docs/deployment_details.md](docs/deployment_details.md) for the FP16 vs INT8 comparison.

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

### Export and Deploy

```bash
# 1. ONNX export (Host PC)
python scripts/export_onnx.py \
    --checkpoint ./checkpoints/ddrnet23-slim/best_model.pth

# 2. Patch resize to nearest (Host PC)
python scripts/patch_resize_nearest.py \
    --input deploy/ddrnet23_slim_unified7class_544x640.onnx \
    --output deploy/ddrnet23_slim_unified7class_544x640_nearest.onnx

# 3. Transfer to device
scp deploy/ddrnet23_slim_unified7class_544x640_nearest.onnx user@iq9075:~/models/

# 4. Run inference (IQ-9075)
python3 scripts/infer_qnn_ros2.py \
    --model ~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx \
    --topic /camera/s10_ultra/color/image_raw
```

---

## Preprocessing

The ONNX model expects ImageNet-normalized input, not raw camera pixels. All inference scripts handle this automatically. If writing custom inference code, apply:

| Step | Operation | Result |
|------|-----------|--------|
| 1 | BGR to RGB | RGB uint8 |
| 2 | Resize to 640 x 544 (W x H) | RGB uint8 |
| 3 | Divide by 255.0 | float32, 0.0 - 1.0 |
| 4 | Subtract mean [0.485, 0.456, 0.406] | float32 |
| 5 | Divide by std [0.229, 0.224, 0.225] | float32, normalized |
| 6 | HWC to NCHW | shape (1, 3, 544, 640) |

Feeding raw pixels (0-255) directly produces meaningless output. See [docs/deployment_details.md](docs/deployment_details.md) for code examples and explanation.

---

## Device Inference

### Video

```bash
# Host PC test
python scripts/infer_qnn_video.py \
    --model deploy/ddrnet23_slim_unified7class_544x640_nearest.onnx \
    --input video.mp4 --backend cpu

# IQ-9075
python scripts/infer_qnn_video.py \
    --model ~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx \
    --input video.mp4 --output result.mp4
```

### ROS2

```bash
python3 scripts/infer_qnn_ros2.py \
    --model ~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx \
    --topic /camera/s10_ultra/color/image_raw
```

| Published Topic | Type | Content |
|-----------------|------|---------|
| ~/segmentation | sensor_msgs/Image (mono8) | 7-class mask (0-6) |
| ~/costmap | nav_msgs/OccupancyGrid | 0=free, 100=lethal |
| ~/overlay | sensor_msgs/Image (bgr8) | Segmentation blend |

---

## Structure

```
amr_segmentation/
├── scripts/
│   ├── export_onnx.py              # ONNX export
│   ├── patch_resize_nearest.py     # bilinear -> nearest conversion
│   ├── quantize_onnx.py            # INT8 QDQ quantization (optional)
│   ├── infer_qnn_video.py          # Video inference
│   └── infer_qnn_ros2.py           # ROS2 live inference
├── src/
│   ├── models.py                   # Model factory
│   └── models_ddrnet.py            # DDRNet builder
├── deploy/                         # Export artifacts
├── docs/
│   ├── deployment_details.md       # FP16 vs INT8, resize, preprocessing
│   └── troubleshooting.md          # All known issues and fixes
└── README.md
```