# Models

Place model files here. These are not tracked in git (large binary files).

## Required File

| File | Source | Size | Usage |
|------|--------|------|-------|
| `ddrnet23_slim_unified7class_544x640.onnx` | `export_qnn.py --method onnx` | 21.8 MB | All inference (NPU + CPU) |

## Auto-Generated (do not transfer manually)

| File | Created By | Usage |
|------|-----------|-------|
| `ddrnet23_slim_unified7class_544x640_ctx.onnx` | QNN EP (first run) | Cached NPU context |

On first inference with QNN EP, the ONNX model is compiled for the Hexagon NPU.
The compiled context is automatically saved as `*_ctx.onnx`. Subsequent runs
load this cache instantly (no recompilation).

## How to Obtain

```bash
# On host PC: export ONNX from checkpoint
python scripts/export_qnn.py \
    --checkpoint ./checkpoints/ddrnet23-slim/best_model.pth \
    --method onnx

# Transfer to IQ-9075
scp deploy/ddrnet23_slim_unified7class_544x640.onnx \
    ubuntu@<IQ9075_IP>:~/VIL-Project-AMR/amr_segmentation/models/
```

## Model Specification

- Architecture: DDRNet23-Slim (5.7M params)
- Input: (1, 3, 544, 640) float32, ImageNet-normalized
- Output: (1, 7, 68, 80) float32 logits (1/8 resolution)
- NPU: 157/157 ops on Hexagon NPU, 19.1 ms median