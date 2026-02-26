#!/usr/bin/env python3
"""
Segmentation Node (Node 1 of 3).

Subscribes to S10 Ultra RGB image, runs DDRNet23-Slim semantic segmentation
on Qualcomm NPU, and publishes a 7-class segmentation mask.

Subscribes:
    /lx_camera_node/LxCamera_Rgb  (sensor_msgs/Image, bgr8)

Publishes:
    ~/segmentation      (sensor_msgs/Image, mono8, class IDs 0~6)
    ~/segmentation_vis  (sensor_msgs/Image, bgr8, colorized overlay)

The segmentation mask (mono8) is consumed by pointcloud_painter_node
for 3D->2D projection label lookup.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from amr_segmentation.segmentation_common import (
    NUM_CLASSES, CLASS_NAMES, CLASS_COLORS_BGR,
    MODEL_INPUT_H, MODEL_INPUT_W,
    preprocess_image, create_ort_session, postprocess_mask,
)


class SegmentationNode(Node):
    """DDRNet23-Slim semantic segmentation on S10 Ultra RGB."""

    def __init__(self):
        super().__init__('segmentation_node')

        # --- Parameters ---
        self.declare_parameter('model_path',
            os.path.expanduser('~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx'))
        self.declare_parameter('use_qnn', True)
        self.declare_parameter('image_topic', '/lx_camera_node/LxCamera_Rgb')
        self.declare_parameter('publish_visualization', True)

        model_path = self.get_parameter('model_path').value
        use_qnn = self.get_parameter('use_qnn').value
        image_topic = self.get_parameter('image_topic').value
        self.pub_vis = self.get_parameter('publish_visualization').value

        # --- ONNX Session ---
        self.get_logger().info(f"Loading model: {model_path}")
        self.session = create_ort_session(model_path, use_qnn=use_qnn)
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info("Model loaded successfully")

        # --- ROS2 I/O ---
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_image = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos)

        self.pub_mask = self.create_publisher(Image, '~/segmentation', 10)

        if self.pub_vis:
            self.pub_overlay = self.create_publisher(Image, '~/segmentation_vis', 10)

        # --- Stats ---
        self.frame_count = 0
        self.total_infer_ms = 0.0

        self.get_logger().info(
            f"Segmentation node ready -- subscribing to: {image_topic}")

    def image_callback(self, msg: Image):
        """Process incoming RGB image: preprocess -> infer -> publish mask."""
        t0 = time.perf_counter()

        # Convert ROS Image to numpy (BGR from SDK)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        orig_h, orig_w = bgr.shape[:2]

        # BGR -> RGB for model input
        rgb = bgr[:, :, ::-1]

        # Preprocess
        input_tensor = preprocess_image(rgb, MODEL_INPUT_H, MODEL_INPUT_W)

        # Inference
        outputs = self.session.run(None, {self.input_name: input_tensor})

        # Postprocess to original resolution
        mask = postprocess_mask(outputs[0], orig_h, orig_w)

        # Publish segmentation mask (mono8, values 0~6)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

        # Publish visualization overlay
        if self.pub_vis:
            overlay = self._create_overlay(bgr, mask)
            vis_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            vis_msg.header = msg.header
            self.pub_overlay.publish(vis_msg)

        # Stats
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.total_infer_ms += dt_ms

        if self.frame_count % 100 == 0:
            avg = self.total_infer_ms / self.frame_count
            self.get_logger().info(
                f"Frame {self.frame_count}: {dt_ms:.1f}ms (avg {avg:.1f}ms)")

    def _create_overlay(self, bgr, mask, alpha=0.5):
        """Create colorized segmentation overlay on original image."""
        import cv2

        color_mask = np.zeros_like(bgr)
        for cid in range(NUM_CLASSES):
            color_mask[mask == cid] = CLASS_COLORS_BGR[cid]

        overlay = cv2.addWeighted(bgr, 1.0 - alpha, color_mask, alpha, 0)
        return overlay


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
