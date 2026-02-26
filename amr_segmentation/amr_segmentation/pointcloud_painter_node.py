#!/usr/bin/env python3
"""
PointCloud Painter Node (Node 2 of 3).

Projects M300 LiDAR 3D points onto the S10 Ultra RGB segmentation mask
to assign semantic labels to each point. Uses the PointPainting approach.

Calibration is obtained automatically from the existing ROS2 infrastructure:
  - Intrinsics: /lx_camera_node/LxCamera_RgbInfo (CameraInfo topic, factory calibration)
  - Extrinsics: TF tree lookup (cloud_frame -> mrdvs_rgb)

No hardcoded calibration.yaml is needed.

TF chain used:
  body <- (FAST-LIO2) <- m300_lidar
  body -> (static TF, cloud_merger.launch.py) -> mrdvs_tof -> (SDK) -> mrdvs_rgb

Subscribes:
    /cloud_registered_body       (sensor_msgs/PointCloud2)  -- M300 LiDAR (body frame)
    ~/segmentation               (sensor_msgs/Image, mono8) -- Segmentation mask
    /lx_camera_node/LxCamera_RgbInfo (sensor_msgs/CameraInfo) -- RGB intrinsics

Publishes:
    ~/painted_pointcloud  (sensor_msgs/PointCloud2)  -- PointCloud with semantic fields

Output PointCloud2 fields:
    x, y, z        (float32)  -- 3D position in body frame
    class_id       (uint8)    -- Semantic class [0..6], 255 = unlabeled
    cost           (uint8)    -- Costmap cost [0..100]
"""

import time
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
import tf2_ros

from amr_segmentation.segmentation_common import (
    NUM_CLASSES, COSTMAP_COST, CLASS_UNLABELED,
    CLASS_SMOOTH_GROUND, CLASS_ROUGH_GROUND, CLASS_OBSTACLE,
)


# ===================================================================
# Helper: TF geometry_msgs Transform -> 4x4 matrix
# ===================================================================

def transform_to_4x4(tf_stamped):
    """Convert geometry_msgs/TransformStamped to 4x4 numpy matrix."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation

    # Quaternion to rotation matrix
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


# ===================================================================
# Core painting algorithm (pure NumPy, no ROS dependency)
# ===================================================================

def paint_pointcloud(points_3d, seg_mask, K, T_cam_lidar, img_w, img_h):
    """
    Project 3D LiDAR points onto 2D segmentation mask and assign labels.

    Args:
        points_3d:    (N, 3) float64 -- points in LiDAR/body frame
        seg_mask:     (H, W) uint8   -- segmentation mask (class IDs 0~6)
        K:            (3, 3) float64 -- camera intrinsic matrix
        T_cam_lidar:  (4, 4) float64 -- transform from LiDAR frame to camera frame
        img_w:        int            -- image width
        img_h:        int            -- image height

    Returns:
        class_ids: (N,) int8   -- class for each point (-1 = unlabeled)
        uv_coords: (N, 2) int  -- projected pixel coordinates (for debug)
    """
    N = points_3d.shape[0]
    class_ids = np.full(N, CLASS_UNLABELED, dtype=np.int8)
    uv_coords = np.zeros((N, 2), dtype=np.int32)

    if N == 0:
        return class_ids, uv_coords

    # Transform points to camera frame: P_cam = T_cam_lidar @ P_lidar
    ones = np.ones((N, 1), dtype=np.float64)
    pts_homo = np.hstack([points_3d, ones])  # (N, 4)
    pts_cam = (T_cam_lidar @ pts_homo.T).T   # (N, 4)

    # Camera frame: Z = forward (depth), X = right, Y = down
    X_cam = pts_cam[:, 0]
    Y_cam = pts_cam[:, 1]
    Z_cam = pts_cam[:, 2]

    # Filter: only points in front of camera (Z > 0)
    valid_depth = Z_cam > 0.1  # 10cm minimum depth

    # Project to image plane (only for valid-depth points to avoid div-by-zero)
    u = np.full(N, -1.0, dtype=np.float64)
    v = np.full(N, -1.0, dtype=np.float64)
    u[valid_depth] = (K[0, 0] * X_cam[valid_depth] / Z_cam[valid_depth] + K[0, 2])
    v[valid_depth] = (K[1, 1] * Y_cam[valid_depth] / Z_cam[valid_depth] + K[1, 2])

    u_int = np.round(u).astype(np.int32)
    v_int = np.round(v).astype(np.int32)

    uv_coords[:, 0] = u_int
    uv_coords[:, 1] = v_int

    # Filter: within image bounds
    valid_bounds = (
        valid_depth &
        (u_int >= 0) & (u_int < img_w) &
        (v_int >= 0) & (v_int < img_h)
    )

    # Lookup class from mask
    valid_idx = np.where(valid_bounds)[0]
    class_ids[valid_idx] = seg_mask[v_int[valid_idx], u_int[valid_idx]].astype(np.int8)

    return class_ids, uv_coords


def geometric_fallback(points_3d, class_ids, ground_z_threshold=-0.3,
                       obstacle_z_threshold=0.5):
    """
    Assign classes to unlabeled points using geometric rules.

    For points outside camera FOV, use height-based heuristics:
      - Below ground level -> Smooth/Rough Ground (conservative)
      - Near ground level  -> Rough Ground
      - Above ground level -> Obstacle

    Args:
        points_3d:   (N, 3) float64  -- points in body frame (z = up)
        class_ids:   (N,) int8       -- current class assignments (-1 = unlabeled)
        ground_z_threshold: z below which is considered ground
        obstacle_z_threshold: z above which is considered obstacle

    Returns:
        class_ids: (N,) int8 -- updated class assignments
    """
    unlabeled = class_ids == CLASS_UNLABELED
    z_vals = points_3d[unlabeled, 2]

    # Height-based rules (z in body frame, 0 = sensor height)
    fallback = np.full(z_vals.shape, CLASS_ROUGH_GROUND, dtype=np.int8)
    fallback[z_vals < ground_z_threshold] = CLASS_SMOOTH_GROUND
    fallback[z_vals > obstacle_z_threshold] = CLASS_OBSTACLE

    class_ids[unlabeled] = fallback
    return class_ids


def class_ids_to_costs(class_ids):
    """Convert class IDs to costmap cost values."""
    costs = np.zeros(class_ids.shape, dtype=np.uint8)
    for cid in range(NUM_CLASSES):
        costs[class_ids == cid] = COSTMAP_COST[cid]
    # Unlabeled points get conservative cost
    costs[class_ids == CLASS_UNLABELED] = 80
    return costs


# ===================================================================
# ROS2 Node
# ===================================================================

class PointCloudPainterNode(Node):
    """Paint LiDAR pointcloud with camera semantic segmentation."""

    def __init__(self):
        super().__init__('pointcloud_painter_node')

        # --- Parameters ---
        self.declare_parameter('lidar_topic', '/cloud_registered_body')
        self.declare_parameter('seg_topic', '/segmentation_node/segmentation')
        self.declare_parameter('caminfo_topic', '/lx_camera_node/LxCamera_RgbInfo')
        self.declare_parameter('camera_frame', 'mrdvs_rgb')
        self.declare_parameter('enable_geometric_fallback', True)
        self.declare_parameter('tf_timeout_sec', 1.0)

        lidar_topic = self.get_parameter('lidar_topic').value
        seg_topic = self.get_parameter('seg_topic').value
        caminfo_topic = self.get_parameter('caminfo_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.enable_fallback = self.get_parameter('enable_geometric_fallback').value
        self.tf_timeout = self.get_parameter('tf_timeout_sec').value

        # --- State ---
        self.K = None              # (3,3) intrinsic matrix
        self.img_w = 0
        self.img_h = 0
        self.seg_mask = None       # Latest segmentation mask
        self.seg_lock = threading.Lock()
        self.K_lock = threading.Lock()

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        self.sub_caminfo = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_callback, 10)

        self.sub_seg = self.create_subscription(
            Image, seg_topic, self.seg_callback, sensor_qos)

        self.sub_lidar = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, sensor_qos)

        # --- Publisher ---
        self.pub_painted = self.create_publisher(
            PointCloud2, '~/painted_pointcloud', 10)

        # --- Stats ---
        self.frame_count = 0
        self.total_paint_ms = 0.0

        self.get_logger().info(
            f"PointCloud Painter ready\n"
            f"  LiDAR:     {lidar_topic}\n"
            f"  Seg mask:  {seg_topic}\n"
            f"  CamInfo:   {caminfo_topic}\n"
            f"  Cam frame: {self.camera_frame}")

    # ----- CameraInfo callback -----
    def caminfo_callback(self, msg: CameraInfo):
        """Receive camera intrinsics from SDK (factory calibration)."""
        with self.K_lock:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.img_w = msg.width
            self.img_h = msg.height

        if self.frame_count == 0:
            self.get_logger().info(
                f"Camera intrinsics received: "
                f"fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
                f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f} "
                f"({self.img_w}x{self.img_h})")

    # ----- Segmentation mask callback -----
    def seg_callback(self, msg: Image):
        """Cache the latest segmentation mask."""
        # mono8: each pixel is a class ID [0..6]
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width)
        with self.seg_lock:
            self.seg_mask = mask.copy()

    # ----- LiDAR callback (main pipeline trigger) -----
    def lidar_callback(self, msg: PointCloud2):
        """
        Main painting pipeline, triggered by each LiDAR frame.

        Steps:
          1. Check prerequisites (K, seg_mask, TF)
          2. Extract XYZ from PointCloud2
          3. Lookup TF: cloud_frame -> camera_frame
          4. Project 3D -> 2D, lookup class from seg mask
          5. Apply geometric fallback for unlabeled points
          6. Build and publish painted PointCloud2
        """
        # --- Prerequisites check ---
        with self.K_lock:
            if self.K is None:
                return
            K = self.K.copy()
            img_w = self.img_w
            img_h = self.img_h

        with self.seg_lock:
            if self.seg_mask is None:
                return
            seg_mask = self.seg_mask.copy()

        t0 = time.perf_counter()

        # --- Extract XYZ from PointCloud2 ---
        points_3d = self._extract_xyz(msg)
        if points_3d.shape[0] == 0:
            return

        # --- TF lookup: cloud frame -> camera frame ---
        cloud_frame = msg.header.frame_id
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.camera_frame,
                cloud_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
            T_cam_lidar = transform_to_4x4(tf_stamped)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if self.frame_count % 50 == 0:
                self.get_logger().warn(
                    f"TF {cloud_frame} -> {self.camera_frame}: {e}")
            return

        # --- Project and paint ---
        class_ids, _ = paint_pointcloud(
            points_3d, seg_mask, K, T_cam_lidar, img_w, img_h)

        # --- Geometric fallback for unlabeled points ---
        if self.enable_fallback:
            class_ids = geometric_fallback(points_3d, class_ids)

        # --- Compute costs ---
        costs = class_ids_to_costs(class_ids)

        # --- Build and publish painted PointCloud2 ---
        painted_msg = self._build_painted_cloud(
            msg.header, points_3d, class_ids, costs)
        self.pub_painted.publish(painted_msg)

        # --- Stats ---
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.total_paint_ms += dt_ms

        n_labeled = np.sum(class_ids >= 0)
        ratio = n_labeled / max(len(class_ids), 1)

        if self.frame_count % 100 == 0:
            avg = self.total_paint_ms / self.frame_count
            self.get_logger().info(
                f"Frame {self.frame_count}: {dt_ms:.1f}ms "
                f"(avg {avg:.1f}ms), "
                f"{n_labeled}/{len(class_ids)} labeled ({ratio*100:.0f}%)")

    # ----- PointCloud2 field extraction -----
    def _extract_xyz(self, msg: PointCloud2):
        """Extract XYZ coordinates from a PointCloud2 message."""
        # Find field offsets
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == 'x':
                x_off = f.offset
            elif f.name == 'y':
                y_off = f.offset
            elif f.name == 'z':
                z_off = f.offset

        if x_off is None or y_off is None or z_off is None:
            self.get_logger().error("PointCloud2 missing x/y/z fields")
            return np.zeros((0, 3), dtype=np.float64)

        data = np.frombuffer(msg.data, dtype=np.uint8)
        n_points = msg.width * msg.height
        step = msg.point_step

        if len(data) < n_points * step:
            return np.zeros((0, 3), dtype=np.float64)

        # Vectorized extraction
        data_reshaped = data.reshape(n_points, step)
        x = np.frombuffer(data_reshaped[:, x_off:x_off+4].tobytes(),
                          dtype=np.float32)
        y = np.frombuffer(data_reshaped[:, y_off:y_off+4].tobytes(),
                          dtype=np.float32)
        z = np.frombuffer(data_reshaped[:, z_off:z_off+4].tobytes(),
                          dtype=np.float32)

        points = np.column_stack([x, y, z]).astype(np.float64)

        # Filter NaN/Inf
        valid = np.all(np.isfinite(points), axis=1)
        return points[valid]

    # ----- Build output PointCloud2 -----
    def _build_painted_cloud(self, header, points_3d, class_ids, costs):
        """Build a PointCloud2 with x, y, z, class_id, cost fields."""
        N = points_3d.shape[0]

        # Define fields
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='class_id', offset=12,
                       datatype=PointField.UINT8, count=1),
            PointField(name='cost', offset=13,
                       datatype=PointField.UINT8, count=1),
        ]

        point_step = 14  # 3*4 + 1 + 1
        row_step = N * point_step

        # Pack data
        buf = bytearray(row_step)
        pts_f32 = points_3d.astype(np.float32)
        # Map unlabeled (-1) to 255 during uint8 conversion
        cls_u8 = np.where(class_ids < 0, np.uint8(255),
                          class_ids.astype(np.uint8))

        for i in range(N):
            offset = i * point_step
            struct.pack_into('<fffBB', buf, offset,
                             pts_f32[i, 0], pts_f32[i, 1], pts_f32[i, 2],
                             cls_u8[i], costs[i])

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = N
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = row_step
        msg.data = bytes(buf)
        msg.is_dense = True

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPainterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
