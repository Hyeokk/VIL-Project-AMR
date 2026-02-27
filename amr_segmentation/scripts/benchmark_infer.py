#!/usr/bin/env python3
"""
Segmentation Inference Benchmark.

Per-stage profiling (preprocess / inference / postprocess) with latency
statistics (min, max, mean, p50, p95, p99, std).

Two modes:
  video  -- Benchmark with mp4 file (no ROS2 required)
  camera -- Benchmark with MRDVS S10 Ultra live feed (ROS2 required)

Usage:
    # Video mode
    python3 scripts/benchmark_infer.py video --input test.mp4
    python3 scripts/benchmark_infer.py video --input test.mp4 --frames 500 --warmup 30
    python3 scripts/benchmark_infer.py video --input test.mp4 --compare  # QNN vs CPU

    # Video mode with overlay mp4 output
    python3 scripts/benchmark_infer.py video --input test.mp4 --output result.mp4

    # Camera mode (ROS2 + S10 Ultra)
    python3 scripts/benchmark_infer.py camera
    python3 scripts/benchmark_infer.py camera --frames 300 --output result.mp4

    # Save stats to JSON
    python3 scripts/benchmark_infer.py video --input test.mp4 --save results.json
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(SCRIPT_DIR, '..')
sys.path.insert(0, PKG_DIR)

from amr_segmentation.segmentation_common import (
    NUM_CLASSES, CLASS_NAMES, CLASS_COLORS_BGR,
    MODEL_INPUT_H, MODEL_INPUT_W,
    preprocess_image, create_ort_session, postprocess_mask,
)

MODEL_PATH = os.path.join(
    PKG_DIR, 'models',
    'ddrnet23_slim_unified7class_544x640_nearest.onnx',
)

CAMERA_TOPIC = '/lx_camera_node/LxCamera_Rgb'


# ===================================================================
# Profiling utilities
# ===================================================================

class StageProfiler:
    """Collects per-stage latency samples and computes statistics."""

    def __init__(self):
        self.preprocess_ms = []
        self.inference_ms = []
        self.postprocess_ms = []
        self.total_ms = []

    def record(self, pre, infer, post):
        self.preprocess_ms.append(pre)
        self.inference_ms.append(infer)
        self.postprocess_ms.append(post)
        self.total_ms.append(pre + infer + post)

    def count(self):
        return len(self.total_ms)

    @staticmethod
    def _stats(arr):
        if not arr:
            return {}
        a = np.array(arr)
        return {
            'count': len(a),
            'mean': float(np.mean(a)),
            'std': float(np.std(a)),
            'min': float(np.min(a)),
            'p50': float(np.percentile(a, 50)),
            'p95': float(np.percentile(a, 95)),
            'p99': float(np.percentile(a, 99)),
            'max': float(np.max(a)),
        }

    def summary(self):
        return {
            'preprocess': self._stats(self.preprocess_ms),
            'inference': self._stats(self.inference_ms),
            'postprocess': self._stats(self.postprocess_ms),
            'total': self._stats(self.total_ms),
        }


def profiled_inference(session, input_name, bgr_frame):
    """Run one frame through the pipeline, returning per-stage timings."""
    orig_h, orig_w = bgr_frame.shape[:2]
    rgb = bgr_frame[:, :, ::-1]

    # --- Preprocess ---
    t0 = time.perf_counter()
    tensor = preprocess_image(rgb, MODEL_INPUT_H, MODEL_INPUT_W)
    t1 = time.perf_counter()

    # --- Inference ---
    outputs = session.run(None, {input_name: tensor})
    t2 = time.perf_counter()

    # --- Postprocess ---
    mask = postprocess_mask(outputs[0], orig_h, orig_w)
    t3 = time.perf_counter()

    pre_ms = (t1 - t0) * 1000.0
    inf_ms = (t2 - t1) * 1000.0
    post_ms = (t3 - t2) * 1000.0

    return mask, pre_ms, inf_ms, post_ms


def draw_benchmark_overlay(frame, mask, pre_ms, inf_ms, post_ms, avg_total, frame_count):
    """Draw segmentation mask + per-stage latency + FPS overlay."""
    total_ms = pre_ms + inf_ms + post_ms
    fps = 1000.0 / avg_total if avg_total > 0 else 0

    # Segmentation color overlay
    color_mask = np.zeros_like(frame)
    for cid in range(NUM_CLASSES):
        color_mask[mask == cid] = CLASS_COLORS_BGR[cid]
    overlay = cv2.addWeighted(frame, 0.5, color_mask, 0.5, 0)

    # Semi-transparent black bar for text readability
    bar_h = 120
    bar = overlay[:bar_h, :, :].copy()
    overlay[:bar_h, :, :] = cv2.addWeighted(
        bar, 0.4, np.zeros_like(bar), 0.6, 0)

    # Per-stage latency
    y = 24
    cv2.putText(overlay, f'Pre: {pre_ms:.1f}ms  |  Infer: {inf_ms:.1f}ms  |  Post: {post_ms:.1f}ms',
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y += 28
    cv2.putText(overlay, f'Total: {total_ms:.1f}ms  (avg {avg_total:.1f}ms)   {fps:.1f} FPS',
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y += 28
    cv2.putText(overlay, f'Frame {frame_count}',
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # Class legend (right side)
    lx = overlay.shape[1] - 180
    ly = 10
    for cid in range(NUM_CLASSES):
        color = CLASS_COLORS_BGR[cid]
        cv2.rectangle(overlay, (lx, ly + cid * 20),
                      (lx + 14, ly + cid * 20 + 14), color, -1)
        cv2.putText(overlay, f'{cid}: {CLASS_NAMES[cid]}',
                    (lx + 20, ly + cid * 20 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return overlay


# ===================================================================
# Display
# ===================================================================

DIVIDER = '─' * 68
HEADER_FMT = '{:<14s} {:>7s} {:>7s} {:>7s} {:>7s} {:>7s} {:>7s} {:>7s}'
ROW_FMT = '{:<14s} {:>7.2f} {:>7.2f} {:>7.2f} {:>7.2f} {:>7.2f} {:>7.2f} {:>7.2f}'


def print_summary(profiler: StageProfiler, label: str = ''):
    s = profiler.summary()
    n = s['total']['count']
    fps = 1000.0 / s['total']['mean'] if s['total']['mean'] > 0 else 0

    if label:
        print(f'\n  ┌── {label}')
    print(f'  │  Measured frames : {n}')
    print(f'  │  Effective FPS   : {fps:.1f}')
    print(f'  │')
    print(f'  │  ' + HEADER_FMT.format(
        'Stage', 'Mean', 'Std', 'Min', 'P50', 'P95', 'P99', 'Max'))
    print(f'  │  {DIVIDER}')

    for stage_name in ('preprocess', 'inference', 'postprocess', 'total'):
        d = s[stage_name]
        label_str = stage_name.capitalize()
        if stage_name == 'total':
            print(f'  │  {DIVIDER}')
            label_str = '▶ TOTAL'
        print(f'  │  ' + ROW_FMT.format(
            label_str,
            d['mean'], d['std'], d['min'], d['p50'], d['p95'], d['p99'], d['max']))

    print(f'  └{"─" * 70}')
    print()


# ===================================================================
# Video mode
# ===================================================================

def run_video_benchmark(args, session, input_name):
    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        print(f'ERROR: Cannot open video: {args.input}')
        sys.exit(1)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 10

    max_frames = args.frames if args.frames else total_frames
    warmup = min(args.warmup, max_frames // 2)
    output_path = getattr(args, 'output', None)

    print(f'  Source     : {args.input}')
    print(f'  Resolution : {src_w}x{src_h} @ {src_fps:.1f}fps')
    print(f'  Total      : {total_frames} frames in file')
    print(f'  Warmup     : {warmup} frames (excluded from stats)')
    print(f'  Measure    : up to {max_frames - warmup} frames')
    if output_path:
        print(f'  Output     : {output_path}')
    print()

    # Video writer (writes measured frames only, skips warmup)
    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, src_fps, (src_w, src_h))

    profiler = StageProfiler()
    frame_idx = 0

    while frame_idx < max_frames:
        ret, frame = cap.read()
        if not ret:
            # Loop video if not enough frames
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                break

        mask, pre_ms, inf_ms, post_ms = profiled_inference(
            session, input_name, frame)
        frame_idx += 1

        if frame_idx <= warmup:
            if frame_idx == 1:
                print(f'  Warming up ...', end='', flush=True)
            if frame_idx % 10 == 0:
                print('.', end='', flush=True)
            if frame_idx == warmup:
                print(f' done ({warmup} frames)\n  Measuring ...', end='', flush=True)
            continue

        profiler.record(pre_ms, inf_ms, post_ms)

        if writer:
            avg_total = np.mean(profiler.total_ms)
            overlay = draw_benchmark_overlay(
                frame, mask, pre_ms, inf_ms, post_ms,
                avg_total, profiler.count())
            writer.write(overlay)

        if profiler.count() % 50 == 0:
            print('.', end='', flush=True)

    cap.release()
    if writer:
        writer.release()
        print(f'\n  Video saved: {output_path}')
    print(' done\n')
    return profiler


# ===================================================================
# Camera mode
# ===================================================================

def run_camera_benchmark(args, session, input_name):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
    except ImportError:
        print('ERROR: ROS2 (rclpy) not available. Use video mode instead.')
        sys.exit(1)

    max_frames = args.frames
    warmup = min(args.warmup, max_frames // 2)
    output_path = getattr(args, 'output', None)

    print(f'  Topic      : {CAMERA_TOPIC}')
    print(f'  Warmup     : {warmup} frames (excluded from stats)')
    print(f'  Measure    : {max_frames - warmup} frames')
    if output_path:
        print(f'  Output     : {output_path}')
    print()

    profiler = StageProfiler()

    class BenchNode(Node):
        def __init__(self):
            super().__init__('segmentation_benchmark_node')
            self.frame_idx = 0
            self.done = False
            self.writer = None
            self.output_path = output_path

            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sub = self.create_subscription(
                Image, CAMERA_TOPIC, self.callback, sensor_qos)
            self.get_logger().info(f'Subscribing to {CAMERA_TOPIC} ...')

        def callback(self, msg):
            if self.done:
                return

            # Decode ROS Image
            if msg.encoding in ('bgr8', 'rgb8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    frame = frame[:, :, ::-1].copy()
            else:
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)

            mask, pre_ms, inf_ms, post_ms = profiled_inference(
                session, input_name, frame)
            self.frame_idx += 1

            if self.frame_idx <= warmup:
                if self.frame_idx == 1:
                    self.get_logger().info(
                        f'First frame received ({msg.width}x{msg.height}). Warming up ...')
                return

            if self.frame_idx == warmup + 1:
                self.get_logger().info('Warmup complete. Measuring ...')
                # Init writer on first measured frame
                if self.output_path and self.writer is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    self.writer = cv2.VideoWriter(
                        self.output_path, fourcc, 10, (w, h))

            profiler.record(pre_ms, inf_ms, post_ms)

            if self.writer:
                avg_total = np.mean(profiler.total_ms)
                overlay = draw_benchmark_overlay(
                    frame, mask, pre_ms, inf_ms, post_ms,
                    avg_total, profiler.count())
                self.writer.write(overlay)

            if profiler.count() % 50 == 0:
                avg = np.mean(profiler.total_ms[-50:])
                self.get_logger().info(
                    f'  [{profiler.count()}/{max_frames - warmup}] '
                    f'last-50 avg: {avg:.2f} ms')

            if profiler.count() >= (max_frames - warmup):
                self.done = True

        def shutdown(self):
            if self.writer:
                self.writer.release()
                print(f'  Video saved: {self.output_path}')

    rclpy.init()
    node = BenchNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n  Interrupted by user.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    print()
    return profiler


# ===================================================================
# Compare mode (QNN vs CPU, video only)
# ===================================================================

def run_compare(args):
    print(f'\n{"=" * 68}')
    print(f'  ▶ QNN (NPU) Session')
    print(f'{"=" * 68}\n')

    session_qnn = create_ort_session(MODEL_PATH, use_qnn=True)
    input_name = session_qnn.get_inputs()[0].name
    profiler_qnn = run_video_benchmark(args, session_qnn, input_name)
    del session_qnn

    print(f'\n{"=" * 68}')
    print(f'  ▶ CPU Session')
    print(f'{"=" * 68}\n')

    session_cpu = create_ort_session(MODEL_PATH, use_qnn=False)
    input_name = session_cpu.get_inputs()[0].name
    profiler_cpu = run_video_benchmark(args, session_cpu, input_name)
    del session_cpu

    # Print both summaries
    print_summary(profiler_qnn, 'QNN (NPU)')
    print_summary(profiler_cpu, 'CPU')

    # Speedup summary
    sq = profiler_qnn.summary()
    sc = profiler_cpu.summary()
    if sq['inference']['mean'] > 0:
        speedup = sc['inference']['mean'] / sq['inference']['mean']
        print(f'  NPU inference speedup vs CPU : {speedup:.1f}x')
        print(f'  NPU inference mean           : {sq["inference"]["mean"]:.2f} ms')
        print(f'  CPU inference mean           : {sc["inference"]["mean"]:.2f} ms')
        print()

    return {'qnn': profiler_qnn.summary(), 'cpu': profiler_cpu.summary()}


# ===================================================================
# Main
# ===================================================================

def main():
    parser = argparse.ArgumentParser(
        description='DDRNet23-Slim Segmentation Benchmark',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scripts/benchmark_infer.py video --input test.mp4
  python3 scripts/benchmark_infer.py video --input test.mp4 --output result.mp4
  python3 scripts/benchmark_infer.py video --input test.mp4 --frames 500 --warmup 30
  python3 scripts/benchmark_infer.py video --input test.mp4 --compare
  python3 scripts/benchmark_infer.py camera --frames 300
  python3 scripts/benchmark_infer.py camera --output result.mp4 --save results.json
        """)

    subparsers = parser.add_subparsers(dest='mode', required=True)

    # --- Video ---
    vp = subparsers.add_parser('video', help='Benchmark with mp4 file')
    vp.add_argument('--input', type=str, required=True, help='Video file path')
    vp.add_argument('--output', type=str, default=None,
                    help='Save overlay video (mp4) with segmentation + latency')
    vp.add_argument('--frames', type=int, default=300,
                    help='Total frames to process (warmup + measure, default: 300)')
    vp.add_argument('--warmup', type=int, default=20,
                    help='Warmup frames excluded from stats (default: 20)')
    vp.add_argument('--no-qnn', action='store_true', help='Force CPU execution')
    vp.add_argument('--compare', action='store_true',
                    help='Run both QNN and CPU, print comparison')
    vp.add_argument('--save', type=str, default=None,
                    help='Save results to JSON file')

    # --- Camera ---
    cp = subparsers.add_parser('camera', help='Benchmark with S10 Ultra (ROS2)')
    cp.add_argument('--output', type=str, default=None,
                    help='Save overlay video (mp4) with segmentation + latency')
    cp.add_argument('--frames', type=int, default=300,
                    help='Total frames to process (warmup + measure, default: 300)')
    cp.add_argument('--warmup', type=int, default=20,
                    help='Warmup frames excluded from stats (default: 20)')
    cp.add_argument('--no-qnn', action='store_true', help='Force CPU execution')
    cp.add_argument('--save', type=str, default=None,
                    help='Save results to JSON file')

    args = parser.parse_args()

    # Validate model
    if not os.path.isfile(MODEL_PATH):
        print(f'ERROR: Model not found: {MODEL_PATH}')
        sys.exit(1)

    print(f'\n{"=" * 68}')
    print(f'  DDRNet23-Slim Segmentation Benchmark')
    print(f'{"=" * 68}')
    print(f'  Mode   : {args.mode}')
    print(f'  Model  : {os.path.basename(MODEL_PATH)}')
    print(f'  Input  : {MODEL_INPUT_W}x{MODEL_INPUT_H} (W x H)')
    if args.mode == 'video' and args.compare:
        print(f'  Compare: QNN (NPU) vs CPU')
    else:
        print(f'  Backend: {"CPU" if args.no_qnn else "QNN (NPU)"}')
    print(f'{"=" * 68}\n')

    # --- Compare mode ---
    if args.mode == 'video' and getattr(args, 'compare', False):
        results = run_compare(args)
        if args.save:
            with open(args.save, 'w') as f:
                json.dump(results, f, indent=2)
            print(f'  Results saved to: {args.save}')
        return

    # --- Single-backend benchmark ---
    use_qnn = not args.no_qnn
    session = create_ort_session(MODEL_PATH, use_qnn=use_qnn)
    input_name = session.get_inputs()[0].name

    if args.mode == 'video':
        profiler = run_video_benchmark(args, session, input_name)
    else:
        profiler = run_camera_benchmark(args, session, input_name)

    if profiler.count() == 0:
        print('  No frames measured. Check input source or increase --frames.')
        return

    backend_label = 'CPU' if args.no_qnn else 'QNN (NPU)'
    print_summary(profiler, backend_label)

    # Save
    if args.save:
        result = {
            'backend': backend_label,
            'mode': args.mode,
            'model': os.path.basename(MODEL_PATH),
            'input_size': f'{MODEL_INPUT_W}x{MODEL_INPUT_H}',
            'warmup_frames': args.warmup,
            **profiler.summary(),
        }
        with open(args.save, 'w') as f:
            json.dump(result, f, indent=2)
        print(f'  Results saved to: {args.save}\n')


if __name__ == '__main__':
    main()