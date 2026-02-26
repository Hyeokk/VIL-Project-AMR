#!/usr/bin/env python3
"""
Segmentation test script for DDRNet23-Slim.

Two modes:
  video  -- Test with mp4 file (no ROS2 required)
  camera -- Test with MRDVS S10 Ultra live feed (ROS2 required)

Model is fixed to: models/ddrnet23_slim_unified7class_544x640_nearest.onnx

Usage:
    # Video mode (no ROS2)
    python3 scripts/infer_test.py video --input test.mp4
    python3 scripts/infer_test.py video --input test.mp4 --output result.mp4

    # Camera mode (ROS2 + S10 Ultra required)
    python3 scripts/infer_test.py camera
    python3 scripts/infer_test.py camera --output result.mp4

    # Common options
    python3 scripts/infer_test.py video --input test.mp4 --no-qnn   # CPU fallback
"""

import argparse
import time
import sys
import os

import cv2
import numpy as np

# Add parent package to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(SCRIPT_DIR, '..')
sys.path.insert(0, PKG_DIR)

from amr_segmentation.segmentation_common import (
    CLASS_NAMES, CLASS_COLORS_BGR, NUM_CLASSES,
    preprocess_image, create_ort_session, postprocess_mask,
)

# Fixed model path (relative to package root)
MODEL_PATH = os.path.join(
    PKG_DIR, 'models',
    'ddrnet23_slim_unified7class_544x640_nearest.onnx'
)

# MRDVS S10 Ultra RGB topic
CAMERA_TOPIC = '/lx_camera_node/LxCamera_Rgb'


def draw_overlay(frame, mask, dt_ms, avg_ms, frame_count):
    """Draw segmentation overlay with stats on frame."""
    color_mask = np.zeros_like(frame)
    for cid in range(NUM_CLASSES):
        color_mask[mask == cid] = CLASS_COLORS_BGR[cid]
    overlay = cv2.addWeighted(frame, 0.5, color_mask, 0.5, 0)

    # Stats text
    fps = 1000.0 / avg_ms if avg_ms > 0 else 0
    cv2.putText(overlay, f"{dt_ms:.1f}ms (avg {avg_ms:.1f}ms, {fps:.1f}FPS)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(overlay, f"Frame {frame_count}",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    # Legend
    y_start = 90
    for cid in range(NUM_CLASSES):
        color = CLASS_COLORS_BGR[cid]
        cv2.rectangle(overlay, (10, y_start + cid * 22),
                      (25, y_start + cid * 22 + 15), color, -1)
        cv2.putText(overlay, f"{cid}: {CLASS_NAMES[cid]}",
                    (32, y_start + cid * 22 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    return overlay


def run_inference(session, input_name, frame):
    """Run segmentation on a single BGR frame."""
    orig_h, orig_w = frame.shape[:2]
    rgb = frame[:, :, ::-1]

    t0 = time.perf_counter()
    tensor = preprocess_image(rgb)
    outputs = session.run(None, {input_name: tensor})
    mask = postprocess_mask(outputs[0], orig_h, orig_w)
    dt_ms = (time.perf_counter() - t0) * 1000.0

    return mask, dt_ms


# ===================================================================
# Video Mode
# ===================================================================

def run_video_mode(args, session, input_name):
    """Test segmentation with mp4 video file."""
    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video: {args.input}")
        sys.exit(1)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 10

    # Output writer
    writer = None
    if args.output:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.output, fourcc, src_fps, (w, h))
        print(f"Saving output to: {args.output}")

    print(f"Source: {args.input} ({total_frames} frames, {src_fps:.1f}fps)")
    print(f"Press 'q' to quit\n")

    frame_count = 0
    total_ms = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        mask, dt_ms = run_inference(session, input_name, frame)
        frame_count += 1
        total_ms += dt_ms
        avg_ms = total_ms / frame_count

        overlay = draw_overlay(frame, mask, dt_ms, avg_ms, frame_count)

        if writer:
            writer.write(overlay)

        cv2.imshow("Segmentation Test - Video", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()

    print(f"\nDone: {frame_count} frames, avg {total_ms/max(frame_count,1):.1f}ms/frame")


# ===================================================================
# Camera Mode
# ===================================================================

def run_camera_mode(args, session, input_name):
    """Test segmentation with MRDVS S10 Ultra camera via ROS2."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
    except ImportError:
        print("ERROR: ROS2 (rclpy) not found.")
        print("Camera mode requires ROS2. Use 'video' mode for testing without ROS2.")
        sys.exit(1)

    class CameraTestNode(Node):
        def __init__(self, session, input_name, output_path):
            super().__init__('segmentation_test_node')
            self.session = session
            self.input_name = input_name
            self.frame_count = 0
            self.total_ms = 0.0
            self.writer = None
            self.output_path = output_path
            self.running = True

            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            self.sub = self.create_subscription(
                Image, CAMERA_TOPIC, self.callback, sensor_qos)

            self.get_logger().info(f"Subscribing to: {CAMERA_TOPIC}")
            self.get_logger().info("Waiting for camera image... (press 'q' in window to quit)")

        def callback(self, msg):
            if not self.running:
                return

            # Convert ROS Image to numpy BGR
            if msg.encoding == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                frame = rgb[:, :, ::-1].copy()
            else:
                self.get_logger().warn(
                    f"Unsupported encoding: {msg.encoding}, trying bgr8 conversion")
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)

            # Init writer on first frame
            if self.output_path and self.writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.output_path, fourcc, 10, (w, h))
                self.get_logger().info(f"Saving output to: {self.output_path}")

            # Inference
            mask, dt_ms = run_inference(self.session, self.input_name, frame)
            self.frame_count += 1
            self.total_ms += dt_ms
            avg_ms = self.total_ms / self.frame_count

            overlay = draw_overlay(frame, mask, dt_ms, avg_ms, self.frame_count)

            if self.writer:
                self.writer.write(overlay)

            cv2.imshow("Segmentation Test - Camera", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False

            if self.frame_count % 100 == 0:
                fps = 1000.0 / avg_ms if avg_ms > 0 else 0
                self.get_logger().info(
                    f"Frame {self.frame_count}: {dt_ms:.1f}ms "
                    f"(avg {avg_ms:.1f}ms, {fps:.1f}FPS)")

        def shutdown(self):
            if self.writer:
                self.writer.release()
            cv2.destroyAllWindows()
            avg = self.total_ms / max(self.frame_count, 1)
            print(f"\nDone: {self.frame_count} frames, avg {avg:.1f}ms/frame")

    rclpy.init()
    node = CameraTestNode(session, input_name, args.output)

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


# ===================================================================
# Main
# ===================================================================

def main():
    parser = argparse.ArgumentParser(
        description="DDRNet23-Slim segmentation test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scripts/infer_test.py video --input test.mp4
  python3 scripts/infer_test.py video --input test.mp4 --output result.mp4
  python3 scripts/infer_test.py camera
  python3 scripts/infer_test.py camera --output result.mp4
        """)

    subparsers = parser.add_subparsers(dest='mode', required=True)

    # Video subcommand
    video_parser = subparsers.add_parser('video', help='Test with mp4 video file')
    video_parser.add_argument('--input', type=str, required=True,
                              help='Video file path')
    video_parser.add_argument('--output', type=str, default=None,
                              help='Output video path (optional)')
    video_parser.add_argument('--no-qnn', action='store_true',
                              help='Force CPU execution')

    # Camera subcommand
    camera_parser = subparsers.add_parser('camera',
                                          help='Test with MRDVS S10 Ultra (ROS2)')
    camera_parser.add_argument('--output', type=str, default=None,
                               help='Output video path (optional)')
    camera_parser.add_argument('--no-qnn', action='store_true',
                               help='Force CPU execution')

    args = parser.parse_args()

    # Validate model exists
    if not os.path.isfile(MODEL_PATH):
        print(f"ERROR: Model not found: {MODEL_PATH}")
        print(f"Expected location: amr_segmentation/models/")
        print(f"  ddrnet23_slim_unified7class_544x640_nearest.onnx")
        sys.exit(1)

    # Create session
    print(f"{'='*60}")
    print(f"  DDRNet23-Slim Segmentation Test")
    print(f"{'='*60}")
    print(f"  Mode:  {args.mode}")
    print(f"  Model: {os.path.basename(MODEL_PATH)}")
    print(f"  QNN:   {'disabled' if args.no_qnn else 'enabled'}")
    print(f"{'='*60}\n")

    session = create_ort_session(MODEL_PATH, use_qnn=not args.no_qnn)
    input_name = session.get_inputs()[0].name

    if args.mode == 'video':
        run_video_mode(args, session, input_name)
    elif args.mode == 'camera':
        run_camera_mode(args, session, input_name)


if __name__ == '__main__':
    main()