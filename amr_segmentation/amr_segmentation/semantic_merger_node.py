#!/usr/bin/env python3
"""
Semantic Merger Node (Node 3 of 3).

Merges painted M300 pointcloud with S10 Ultra depth pointcloud (labeled
using the same segmentation mask) and generates a 2D OccupancyGrid costmap.

Two sources of labeled points:
  A) M300 painted pointcloud (from pointcloud_painter_node)
     - 360 deg coverage, sparse ground points near robot
  B) S10 Ultra depth pointcloud (self-labeled via seg mask)
     - 120x80 deg forward, dense near-ground coverage

Points are merged in a voxel grid. When both sources contribute to the
same voxel, the MORE DANGEROUS (higher cost) label wins (conservative).

Subscribes:
    ~/painted_pointcloud         (PointCloud2)  -- from pointcloud_painter_node
    /lx_camera_node/LxCamera_Cloud (PointCloud2) -- S10 Ultra depth cloud
    ~/segmentation               (Image, mono8) -- for labeling S10 depth points

Publishes:
    ~/semantic_costmap  (nav_msgs/OccupancyGrid) -- 2D costmap for Nav2
    ~/merged_cloud      (PointCloud2)            -- merged labeled cloud (debug)
"""

import time
import struct
import threading
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import tf2_ros

from amr_segmentation.segmentation_common import (
    NUM_CLASSES, COSTMAP_COST,
)


@dataclass
class CostmapConfig:
    """Costmap generation parameters."""
    # Grid dimensions
    grid_size_m: float = 6.0         # 6m x 6m around robot
    resolution: float = 0.1          # 10cm per cell
    # Height filter (body frame z)
    z_min: float = -1.0              # include downhill/slopes
    z_max: float = 2.0               # exclude sky/ceiling
    # S10 depth stale threshold
    s10_stale_sec: float = 0.5


class SemanticMergerNode(Node):
    """Merge labeled pointclouds and produce semantic costmap."""

    def __init__(self):
        super().__init__('semantic_merger_node')

        # --- Parameters ---
        self.declare_parameter('painted_topic',
            '/pointcloud_painter_node/painted_pointcloud')
        self.declare_parameter('s10_cloud_topic',
            '/lx_camera_node/LxCamera_Cloud')
        self.declare_parameter('seg_topic',
            '/segmentation_node/segmentation')
        self.declare_parameter('caminfo_topic',
            '/lx_camera_node/LxCamera_RgbInfo')
        self.declare_parameter('robot_frame', 'body')
        self.declare_parameter('grid_size_m', 6.0)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('z_min', -1.0)
        self.declare_parameter('z_max', 2.0)

        painted_topic = self.get_parameter('painted_topic').value
        s10_cloud_topic = self.get_parameter('s10_cloud_topic').value
        seg_topic = self.get_parameter('seg_topic').value
        caminfo_topic = self.get_parameter('caminfo_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.cfg = CostmapConfig(
            grid_size_m=self.get_parameter('grid_size_m').value,
            resolution=self.get_parameter('resolution').value,
            z_min=self.get_parameter('z_min').value,
            z_max=self.get_parameter('z_max').value,
        )

        # --- State ---
        self.seg_mask = None
        self.seg_lock = threading.Lock()
        self.K = None
        self.img_w = 0
        self.img_h = 0
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
        self.sub_painted = self.create_subscription(
            PointCloud2, painted_topic, self.painted_callback, sensor_qos)
        self.sub_s10 = self.create_subscription(
            PointCloud2, s10_cloud_topic, self.s10_callback, sensor_qos)
        self.sub_seg = self.create_subscription(
            Image, seg_topic, self.seg_callback, sensor_qos)
        self.sub_caminfo = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_callback, 10)

        # --- Publishers ---
        self.pub_costmap = self.create_publisher(
            OccupancyGrid, '~/semantic_costmap', 10)
        self.pub_merged = self.create_publisher(
            PointCloud2, '~/merged_cloud', 10)

        # S10 cache
        self.s10_cloud = None
        self.s10_time = None
        self.s10_lock = threading.Lock()

        # Stats
        self.frame_count = 0

        self.get_logger().info(
            f"Semantic Merger ready\n"
            f"  Painted:   {painted_topic}\n"
            f"  S10 cloud: {s10_cloud_topic}\n"
            f"  Grid:      {self.cfg.grid_size_m}m, "
            f"res={self.cfg.resolution}m")

    def caminfo_callback(self, msg: CameraInfo):
        """Cache camera intrinsics for S10 depth labeling."""
        with self.K_lock:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.img_w = msg.width
            self.img_h = msg.height

    def seg_callback(self, msg: Image):
        """Cache latest segmentation mask."""
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width)
        with self.seg_lock:
            self.seg_mask = mask.copy()

    def s10_callback(self, msg: PointCloud2):
        """Cache latest S10 depth pointcloud."""
        with self.s10_lock:
            self.s10_cloud = msg
            self.s10_time = time.time()

    def painted_callback(self, msg: PointCloud2):
        """
        Main pipeline trigger: M300 painted cloud arrives.

        Steps:
          1. Extract painted M300 points (x,y,z,class_id,cost)
          2. Optionally add labeled S10 depth points
          3. Voxel merge (higher cost wins)
          4. Generate and publish OccupancyGrid
        """
        t0 = time.perf_counter()

        # --- 1. Extract painted M300 points ---
        m300_pts, m300_cls, m300_costs = self._extract_painted_cloud(msg)

        # --- 2. Optionally add S10 depth points ---
        s10_pts = np.zeros((0, 3), dtype=np.float32)
        s10_cls = np.zeros(0, dtype=np.uint8)
        s10_costs = np.zeros(0, dtype=np.uint8)

        with self.s10_lock:
            s10_msg = self.s10_cloud
            s10_t = self.s10_time

        if s10_msg is not None and s10_t is not None:
            dt = time.time() - s10_t
            if dt < self.cfg.s10_stale_sec:
                s10_pts, s10_cls, s10_costs = self._label_s10_cloud(s10_msg)

        # --- 3. Merge ---
        if s10_pts.shape[0] > 0:
            all_pts = np.vstack([m300_pts, s10_pts])
            all_cls = np.concatenate([m300_cls, s10_cls])
            all_costs = np.concatenate([m300_costs, s10_costs])
        else:
            all_pts = m300_pts
            all_cls = m300_cls
            all_costs = m300_costs

        if all_pts.shape[0] == 0:
            return

        # --- Height filter ---
        z = all_pts[:, 2]
        valid = (z >= self.cfg.z_min) & (z <= self.cfg.z_max)
        all_pts = all_pts[valid]
        all_cls = all_cls[valid]
        all_costs = all_costs[valid]

        # --- 4. Generate costmap ---
        costmap_msg = self._generate_costmap(
            all_pts, all_costs, msg.header)
        self.pub_costmap.publish(costmap_msg)

        # --- Publish merged cloud (debug) ---
        merged_msg = self._build_merged_cloud(
            msg.header, all_pts, all_cls, all_costs)
        self.pub_merged.publish(merged_msg)

        # Stats
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Frame {self.frame_count}: {dt_ms:.1f}ms, "
                f"{all_pts.shape[0]} pts "
                f"(M300={m300_pts.shape[0]}, S10={s10_pts.shape[0]})")

    def _extract_painted_cloud(self, msg: PointCloud2):
        """Extract x,y,z,class_id,cost from painted PointCloud2."""
        n_points = msg.width * msg.height
        step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if len(data) < n_points * step or n_points == 0:
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros(0, dtype=np.uint8),
                    np.zeros(0, dtype=np.uint8))

        # Find field offsets
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset

        data_2d = data.reshape(n_points, step)

        x = np.frombuffer(
            data_2d[:, offsets['x']:offsets['x']+4].tobytes(),
            dtype=np.float32)
        y = np.frombuffer(
            data_2d[:, offsets['y']:offsets['y']+4].tobytes(),
            dtype=np.float32)
        z = np.frombuffer(
            data_2d[:, offsets['z']:offsets['z']+4].tobytes(),
            dtype=np.float32)

        pts = np.column_stack([x, y, z])

        # class_id and cost are uint8
        cls = data_2d[:, offsets['class_id']].copy()
        costs = data_2d[:, offsets['cost']].copy()

        # Filter NaN
        valid = np.all(np.isfinite(pts), axis=1)
        return pts[valid], cls[valid], costs[valid]

    def _label_s10_cloud(self, msg: PointCloud2):
        """
        Label S10 Ultra depth pointcloud using segmentation mask.

        S10 outputs XYZRGB in mrdvs_tof frame. We need to:
          1. Transform to mrdvs_rgb frame (via TF)
          2. Project to image plane (using intrinsics)
          3. Lookup class from seg mask
          4. Transform to body frame for merging
        """
        with self.seg_lock:
            if self.seg_mask is None:
                return (np.zeros((0, 3), dtype=np.float32),
                        np.zeros(0, dtype=np.uint8),
                        np.zeros(0, dtype=np.uint8))
            seg_mask = self.seg_mask.copy()

        with self.K_lock:
            if self.K is None:
                return (np.zeros((0, 3), dtype=np.float32),
                        np.zeros(0, dtype=np.uint8),
                        np.zeros(0, dtype=np.uint8))
            K = self.K.copy()
            img_w = self.img_w
            img_h = self.img_h

        # Extract XYZ from S10 cloud
        n_points = msg.width * msg.height
        step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if len(data) < n_points * step or n_points == 0:
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros(0, dtype=np.uint8),
                    np.zeros(0, dtype=np.uint8))

        # Find x/y/z offsets
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == 'x':
                x_off = f.offset
            elif f.name == 'y':
                y_off = f.offset
            elif f.name == 'z':
                z_off = f.offset

        if x_off is None:
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros(0, dtype=np.uint8),
                    np.zeros(0, dtype=np.uint8))

        data_2d = data.reshape(n_points, step)
        x = np.frombuffer(data_2d[:, x_off:x_off+4].tobytes(),
                          dtype=np.float32)
        y = np.frombuffer(data_2d[:, y_off:y_off+4].tobytes(),
                          dtype=np.float32)
        z = np.frombuffer(data_2d[:, z_off:z_off+4].tobytes(),
                          dtype=np.float32)

        pts_tof = np.column_stack([x, y, z])

        # Filter invalid points
        valid = (np.all(np.isfinite(pts_tof), axis=1) &
                 ~np.all(pts_tof == 0, axis=1))
        pts_tof = pts_tof[valid]

        if pts_tof.shape[0] == 0:
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros(0, dtype=np.uint8),
                    np.zeros(0, dtype=np.uint8))

        # TF: mrdvs_tof -> mrdvs_rgb (for projection)
        try:
            from amr_segmentation.pointcloud_painter_node import (
                transform_to_4x4, paint_pointcloud, class_ids_to_costs)

            tf_tof_rgb = self.tf_buffer.lookup_transform(
                'mrdvs_rgb', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            T_rgb_tof = transform_to_4x4(tf_tof_rgb)

            # Project S10 points to RGB image for labeling
            class_ids, _ = paint_pointcloud(
                pts_tof.astype(np.float64), seg_mask,
                K, T_rgb_tof, img_w, img_h)

            costs = class_ids_to_costs(class_ids)

            # TF: mrdvs_tof -> body (for spatial merging)
            tf_tof_body = self.tf_buffer.lookup_transform(
                self.robot_frame, msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            T_body_tof = transform_to_4x4(tf_tof_body)

            ones = np.ones((pts_tof.shape[0], 1), dtype=np.float64)
            pts_homo = np.hstack([pts_tof.astype(np.float64), ones])
            pts_body = (T_body_tof @ pts_homo.T).T[:, :3].astype(np.float32)

            # Map unlabeled (-1) to 255 during uint8 conversion
            cls_out = np.where(class_ids < 0, np.uint8(255),
                               class_ids.astype(np.uint8))
            return pts_body, cls_out, costs

        except Exception as e:
            if self.frame_count % 50 == 0:
                self.get_logger().warn(f"S10 labeling failed: {e}")
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros(0, dtype=np.uint8),
                    np.zeros(0, dtype=np.uint8))

    def _generate_costmap(self, all_pts, all_costs, header):
        """
        Generate 2D OccupancyGrid from labeled 3D points.

        Voxel grid projection: for each (x,y) cell, take the MAX cost
        across all points in that cell (conservative strategy).
        """
        half = self.cfg.grid_size_m / 2.0
        res = self.cfg.resolution
        n_cells = int(self.cfg.grid_size_m / res)

        # Initialize unknown (-1)
        grid = np.full((n_cells, n_cells), -1, dtype=np.int8)

        if all_pts.shape[0] > 0:
            # Map points to grid indices
            # Robot is at center of grid
            gx = ((all_pts[:, 0] + half) / res).astype(np.int32)
            gy = ((all_pts[:, 1] + half) / res).astype(np.int32)

            # Filter within grid bounds
            valid = (
                (gx >= 0) & (gx < n_cells) &
                (gy >= 0) & (gy < n_cells)
            )
            gx = gx[valid]
            gy = gy[valid]
            costs = all_costs[valid]

            # Fill grid: max cost per cell (conservative)
            for i in range(len(gx)):
                cell_val = grid[gy[i], gx[i]]
                cost_val = int(costs[i])
                if cell_val < cost_val:
                    grid[gy[i], gx[i]] = cost_val

        # Build OccupancyGrid message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.robot_frame

        msg.info = MapMetaData()
        msg.info.resolution = res
        msg.info.width = n_cells
        msg.info.height = n_cells

        # Origin: bottom-left corner of grid
        msg.info.origin = Pose()
        msg.info.origin.position.x = -half
        msg.info.origin.position.y = -half
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten grid to row-major list
        msg.data = grid.flatten().tolist()

        return msg

    def _build_merged_cloud(self, header, pts, cls, costs):
        """Build a debug PointCloud2 with merged labeled points."""
        N = pts.shape[0]
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
        point_step = 14
        buf = bytearray(N * point_step)
        pts_f32 = pts.astype(np.float32)

        for i in range(N):
            offset = i * point_step
            struct.pack_into('<fffBB', buf, offset,
                             pts_f32[i, 0], pts_f32[i, 1], pts_f32[i, 2],
                             cls[i], costs[i])

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = N
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = N * point_step
        msg.data = bytes(buf)
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
