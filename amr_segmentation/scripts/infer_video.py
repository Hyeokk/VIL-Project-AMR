#!/usr/bin/env python3
"""
Standalone video inference script for DDRNet23-Slim segmentation.
Runs without ROS2 for quick offline testing.

Usage:
    python3 scripts/infer_video.py --input video.mp4 --model ~/models/ddrnet23.onnx
    python3 scripts/infer_video.py --input 0  # webcam
"""

import argparse
import time
import sys
import os

import cv2
import numpy as np

# Add parent package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from amr_segmentation.segmentation_common import (
    CLASS_NAMES, CLASS_COLORS_BGR, NUM_CLASSES,
    preprocess_image, create_ort_session, postprocess_mask,
)


def main():
    parser = argparse.ArgumentParser(description="Offline segmentation inference")
    parser.add_argument('--input', type=str, required=True,
                        help="Video file path or camera index (0, 1, ...)")
    parser.add_argument('--model', type=str, required=True,
                        help="Path to ONNX model")
    parser.add_argument('--no-qnn', action='store_true',
                        help="Force CPU execution (skip QNN)")
    parser.add_argument('--output', type=str, default=None,
                        help="Output video path (optional)")
    args = parser.parse_args()

    # Open video source
    try:
        src = int(args.input)
    except ValueError:
        src = args.input

    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video source: {args.input}")
        sys.exit(1)

    # Create session
    session = create_ort_session(args.model, use_qnn=not args.no_qnn)
    input_name = session.get_inputs()[0].name

    # Output video writer
    writer = None
    if args.output:
        fps = cap.get(cv2.CAP_PROP_FPS) or 10
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.output, fourcc, fps, (w, h))

    frame_count = 0
    total_ms = 0.0

    print(f"Running segmentation on: {args.input}")
    print(f"Press 'q' to quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        orig_h, orig_w = frame.shape[:2]
        rgb = frame[:, :, ::-1]

        t0 = time.perf_counter()
        tensor = preprocess_image(rgb)
        outputs = session.run(None, {input_name: tensor})
        mask = postprocess_mask(outputs[0], orig_h, orig_w)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        frame_count += 1
        total_ms += dt_ms

        # Colorize overlay
        color_mask = np.zeros_like(frame)
        for cid in range(NUM_CLASSES):
            color_mask[mask == cid] = CLASS_COLORS_BGR[cid]
        overlay = cv2.addWeighted(frame, 0.5, color_mask, 0.5, 0)

        # Add FPS text
        avg_ms = total_ms / frame_count
        cv2.putText(overlay, f"{dt_ms:.1f}ms (avg {avg_ms:.1f}ms)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if writer:
            writer.write(overlay)

        cv2.imshow("Segmentation", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()

    print(f"\nProcessed {frame_count} frames, avg {total_ms/max(frame_count,1):.1f}ms/frame")


if __name__ == '__main__':
    main()
