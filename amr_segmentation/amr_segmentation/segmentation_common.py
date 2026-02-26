#!/usr/bin/env python3
"""
Shared constants and utilities for the AMR segmentation pipeline.

7-class offroad semantic segmentation ontology:
  0: Smooth Ground   - paved road, flat dirt
  1: Rough Ground    - gravel, uneven dirt
  2: Bumpy Ground    - rocky terrain, roots
  3: Obstacle        - walls, large rocks, structures
  4: Vegetation      - bushes, grass, low plants
  5: Sky             - sky, open air
  6: Other           - unclassified

DDRNet23-Slim model inference setup and preprocessing.
"""

import os
import numpy as np

# ===================================================================
# 7-Class Ontology
# ===================================================================

NUM_CLASSES = 7

CLASS_NAMES = [
    "Smooth Ground",   # 0
    "Rough Ground",    # 1
    "Bumpy Ground",    # 2
    "Obstacle",        # 3
    "Vegetation",      # 4
    "Sky",             # 5
    "Other",           # 6
]

# OccupancyGrid cost values [0..100] for each class
# Higher = more dangerous for traversal
COSTMAP_COST = [
    0,     # 0: Smooth Ground  - free to traverse
    40,    # 1: Rough Ground   - mild penalty
    60,    # 2: Bumpy Ground   - moderate penalty
    100,   # 3: Obstacle       - lethal
    30,    # 4: Vegetation     - low penalty (may be traversable)
    0,     # 5: Sky            - irrelevant (above ground)
    80,    # 6: Other          - conservative high penalty
]

# Class ID used for points that could not be labeled
CLASS_UNLABELED = -1

# Geometric fallback class assignments (for points outside camera FOV)
CLASS_SMOOTH_GROUND = 0
CLASS_ROUGH_GROUND = 1
CLASS_OBSTACLE = 3
CLASS_VEGETATION = 4

# ===================================================================
# Color palette for visualization (BGR format for OpenCV)
# ===================================================================

CLASS_COLORS_BGR = [
    (0, 200, 0),      # 0: Smooth Ground  - green
    (0, 150, 200),    # 1: Rough Ground   - orange
    (0, 100, 255),    # 2: Bumpy Ground   - dark orange
    (0, 0, 255),      # 3: Obstacle       - red
    (0, 255, 0),      # 4: Vegetation     - bright green
    (255, 200, 100),  # 5: Sky            - light blue
    (128, 128, 128),  # 6: Other          - gray
]

# ===================================================================
# Model preprocessing constants
# ===================================================================

# DDRNet23-Slim input size (height, width)
MODEL_INPUT_H = 544
MODEL_INPUT_W = 640

# ImageNet normalization (used during training)
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocess_image(rgb_img, target_h=MODEL_INPUT_H, target_w=MODEL_INPUT_W):
    """
    Preprocess an RGB image for DDRNet23-Slim inference.

    Steps:
      1. Resize to (target_h, target_w) using nearest-neighbor
      2. Convert to float32 [0, 1]
      3. Normalize with ImageNet mean/std
      4. Transpose to NCHW format

    Args:
        rgb_img: np.ndarray (H, W, 3) uint8 RGB image
        target_h: Model input height
        target_w: Model input width

    Returns:
        np.ndarray (1, 3, target_h, target_w) float32
    """
    import cv2

    # Resize (nearest-neighbor to avoid interpolation artifacts on labels)
    resized = cv2.resize(rgb_img, (target_w, target_h),
                         interpolation=cv2.INTER_NEAREST)

    # Normalize
    img = resized.astype(np.float32) / 255.0
    img = (img - IMAGENET_MEAN) / IMAGENET_STD

    # HWC -> NCHW
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)

    return img.astype(np.float32)


def create_ort_session(model_path, use_qnn=True):
    """
    Create an ONNX Runtime inference session.

    Attempts QNN (Qualcomm NPU) first, falls back to CPU.

    Args:
        model_path: Path to ONNX model file
        use_qnn: If True, attempt QNN EP first

    Returns:
        onnxruntime.InferenceSession
    """
    import onnxruntime as ort

    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    providers_tried = []

    if use_qnn:
        try:
            sess = ort.InferenceSession(
                model_path,
                providers=['QNNExecutionProvider'],
                provider_options=[{
                    'backend_path': 'libQnnHtp.so',
                    'htp_performance_mode': 'burst',
                }]
            )
            print(f"[segmentation_common] QNN (NPU) session created: {model_path}")
            return sess
        except Exception as e:
            providers_tried.append(f"QNN failed: {e}")

    # CPU fallback
    sess = ort.InferenceSession(
        model_path,
        providers=['CPUExecutionProvider']
    )
    if providers_tried:
        print(f"[segmentation_common] {'; '.join(providers_tried)}")
    print(f"[segmentation_common] CPU session created: {model_path}")
    return sess


def postprocess_mask(raw_output, orig_h, orig_w):
    """
    Postprocess model output to segmentation mask at original resolution.

    Args:
        raw_output: np.ndarray (1, NUM_CLASSES, H, W) logits
        orig_h: Original image height
        orig_w: Original image width

    Returns:
        np.ndarray (orig_h, orig_w) uint8 class indices [0..6]
    """
    import cv2

    # argmax over class dimension
    mask = np.argmax(raw_output[0], axis=0).astype(np.uint8)

    # Resize back to original resolution
    if mask.shape[0] != orig_h or mask.shape[1] != orig_w:
        mask = cv2.resize(mask, (orig_w, orig_h),
                          interpolation=cv2.INTER_NEAREST)

    return mask
