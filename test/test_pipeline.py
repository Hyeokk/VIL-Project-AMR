#!/usr/bin/env python3
"""
test_pipeline.py — FAST-LIO2 없이 cloud_merger + GroundGrid 파이프라인 테스트

FAST-LIO2는 실제 센서 없이 초기화가 어려우므로,
FAST-LIO2 출력을 직접 시뮬레이션합니다.

발행:
  /cloud_registered_body  (FAST-LIO2 출력 시뮬레이션, body frame)
  /lx_camera_node/LxCamera_Cloud (S10 Ultra 시뮬레이션, mrdvs_tof frame)
  /Odometry               (FAST-LIO2 odometry 시뮬레이션)
  TF: odom→camera_init→body→base_footprint→base_link, base_link→mrdvs_tof

테스트 순서:
  터미널1: python3 test_pipeline.py
  터미널2: ros2 launch cloud_merger cloud_merger.launch.py
  터미널3: ros2 launch groundgrid amr_groundgrid.launch.py

검증:
  ros2 topic hz /merged_cloud
  ros2 topic hz /groundgrid/filtered_cloud
  ros2 run tf2_tools view_frames
"""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TestPipeline(Node):
    def __init__(self):
        super().__init__('test_pipeline')

        # --- Publishers ---
        self.pub_m300 = self.create_publisher(
            PointCloud2, '/cloud_registered_body', 10)
        self.pub_s10 = self.create_publisher(
            PointCloud2, '/lx_camera_node/LxCamera_Cloud', 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/Odometry', 10)

        # --- Static TFs ---
        self.static_tf_bc = StaticTransformBroadcaster(self)
        static_tfs = []

        # odom → camera_init (identity)
        static_tfs.append(self._make_static_tf(
            'odom', 'camera_init', 0, 0, 0))

        # body → base_footprint
        static_tfs.append(self._make_static_tf(
            'body', 'base_footprint', -0.3544, 0.0287, -0.5848))

        # base_footprint → base_link (from URDF)
        static_tfs.append(self._make_static_tf(
            'base_footprint', 'base_link', 0, 0, 0.1005))

        # base_link → mrdvs_tof (normally from lx_camera_ros driver)
        static_tfs.append(self._make_static_tf(
            'base_link', 'mrdvs_tof', 0.35, 0.0, 0.35))

        self.static_tf_bc.sendTransform(static_tfs)

        # --- Dynamic TF: camera_init → body (simulated FAST-LIO2) ---
        self.dynamic_tf_bc = TransformBroadcaster(self)

        # --- Timers ---
        self.timer_main = self.create_timer(0.1, self.publish_all)  # 10Hz
        self.t = 0.0

        self.get_logger().info('=== TestPipeline started ===')
        self.get_logger().info('Publishing: /cloud_registered_body, '
                               '/lx_camera_node/LxCamera_Cloud, /Odometry')
        self.get_logger().info('TF: odom→camera_init→body→base_footprint'
                               '→base_link→mrdvs_tof')
        self.get_logger().info('')
        self.get_logger().info('다른 터미널에서 실행:')
        self.get_logger().info('  ros2 launch cloud_merger cloud_merger.launch.py')
        self.get_logger().info('  ros2 launch groundgrid amr_groundgrid.launch.py')
        self.get_logger().info('')
        self.get_logger().info('검증:')
        self.get_logger().info('  ros2 topic hz /merged_cloud')
        self.get_logger().info('  ros2 topic hz /groundgrid/filtered_cloud')
        self.get_logger().info('  ros2 run tf2_tools view_frames')

    def publish_all(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # --- Simulate slow movement (circle) ---
        speed = 0.1  # m/s
        radius = 5.0
        angular = speed / radius
        x = radius * math.sin(angular * self.t)
        y = radius * (1 - math.cos(angular * self.t))
        yaw = angular * self.t
        self.t += 0.1

        # --- Dynamic TF: camera_init → body ---
        tf_dyn = TransformStamped()
        tf_dyn.header.stamp = stamp
        tf_dyn.header.frame_id = 'camera_init'
        tf_dyn.child_frame_id = 'body'
        tf_dyn.transform.translation.x = x
        tf_dyn.transform.translation.y = y
        tf_dyn.transform.translation.z = 0.0
        tf_dyn.transform.rotation.z = math.sin(yaw / 2)
        tf_dyn.transform.rotation.w = math.cos(yaw / 2)
        self.dynamic_tf_bc.sendTransform(tf_dyn)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'camera_init'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_odom.publish(odom)

        # --- M300 cloud (body frame) ---
        # 지면: 3.6m~ (M300 blind zone 반영)
        # 장애물: 랜덤
        m300_pts = []
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for r in np.arange(4.0, 12.0, 0.5):
            for a in np.arange(0, 2 * math.pi, 0.15):
                px = r * math.cos(a)
                py = r * math.sin(a)
                pz = -0.63 + np.random.normal(0, 0.02)
                m300_pts.append((px, py, pz, 40.0))

        # 장애물 (바위)
        for i in range(100):
            a = np.random.uniform(0, 2 * math.pi)
            d = np.random.uniform(5.0, 15.0)
            m300_pts.append((
                d * math.cos(a), d * math.sin(a),
                np.random.uniform(-0.3, 1.5), 80.0))

        self.pub_m300.publish(
            self._make_xyzi(m300_pts, stamp, 'body'))

        # --- S10 Ultra cloud (mrdvs_tof frame) ---
        # 전방 근거리 지면 (0.3~5m)
        s10_pts = []
        for d in np.arange(0.3, 5.0, 0.15):
            for lat in np.arange(-d * 0.7, d * 0.7, 0.2):
                px = d
                py = lat
                pz = -0.35 + np.random.normal(0, 0.03)
                s10_pts.append((px, py, pz, 100, 120, 80))

        # 전방 장애물
        for i in range(30):
            s10_pts.append((
                np.random.uniform(1.0, 3.0),
                np.random.uniform(-1.0, 1.0),
                np.random.uniform(-0.1, 0.6),
                150, 150, 150))

        self.pub_s10.publish(
            self._make_xyzrgb(s10_pts, stamp, 'mrdvs_tof'))

        if int(self.t * 10) % 50 == 0:
            self.get_logger().info(
                f't={self.t:.1f}s pos=({x:.2f},{y:.2f}) '
                f'M300={len(m300_pts)}pts S10={len(s10_pts)}pts')

    # --- Helpers ---
    def _make_static_tf(self, parent, child, x, y, z):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = float(z)
        tf.transform.rotation.w = 1.0
        return tf

    def _make_xyzi(self, points, stamp, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        buf = bytearray()
        for x, y, z, i in points:
            buf += struct.pack('ffff', x, y, z, i)
        msg.data = bytes(buf)
        return msg

    def _make_xyzrgb(self, points, stamp, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        buf = bytearray()
        for x, y, z, r, g, b in points:
            rgb_packed = struct.unpack('f', struct.pack('I',
                (int(r) << 16) | (int(g) << 8) | int(b)))[0]
            buf += struct.pack('ffff', x, y, z, rgb_packed)
        msg.data = bytes(buf)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TestPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()