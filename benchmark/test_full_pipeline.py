#!/usr/bin/env python3
"""
test_full_pipeline.py — 센서 없이 전체 파이프라인 통합 테스트

시뮬레이션 대상:
  - FAST-LIO2: TF(odom→camera_init→body) + /Odometry
  - M300 LiDAR: /cloud_registered_body (body frame)
  - S10 Ultra:  /lx_camera_node/LxCamera_Cloud (mrdvs_tof frame)
  - omorobot:   /cmd_vel 수신 → 차동구동 시뮬레이션 → 위치 업데이트

월드 구성:
  - 평탄 지면 (풀밭 시뮬레이션: 높이 노이즈)
  - 정적 장애물: 벽, 바위
  - 동적 장애물: 사람 (직선 왕복)

실행 순서:
  터미널1: python3 test_full_pipeline.py
  터미널2: ros2 launch cloud_merger cloud_merger.launch.py
  터미널3: ros2 launch groundgrid amr_groundgrid.launch.py
  터미널4: ros2 launch terrain_costmap terrain_costmap.launch.py
  터미널5: ros2 launch path_planner path_planner.launch.py
  터미널6: rviz2  (Fixed Frame: odom, 2D Goal Pose로 목표 지정)

rviz 토픽 추가:
  - /terrain_costmap (OccupancyGrid)
  - /path_planner/global_path (Path)
  - /path_planner/local_trajectory (Path)
  - /groundgrid/filtered_cloud (PointCloud2)
  - TF
"""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# =========================================================================
# 월드 정의: 정적 장애물 + 동적 장애물
# =========================================================================
# 정적 장애물 (odom frame): (cx, cy, radius, height)
STATIC_OBSTACLES = [
    (3.0, 0.0, 0.4, 1.0),     # 바위 1
    (-2.0, 3.0, 0.5, 0.8),    # 바위 2
    (1.0, -2.5, 0.3, 1.2),    # 기둥
    (5.0, 2.0, 0.6, 0.6),     # 낮은 바위
    (-3.0, -1.0, 0.35, 1.5),  # 높은 기둥
]

# 벽 (odom frame): (x_start, y_start, x_end, y_end, thickness, height)
WALLS = [
    (6.0, -3.0, 6.0, 3.0, 0.2, 1.5),   # 우측 벽
    (-4.0, 4.0, 2.0, 4.0, 0.2, 1.0),    # 상단 벽
]

# 동적 장애물 (사람): (cx, cy, radius, height, move_axis, move_range, speed)
# move_axis: 'x' or 'y', move_range: 왕복 거리, speed: m/s
DYNAMIC_OBSTACLES = [
    (2.0, 1.5, 0.3, 1.7, 'x', 2.0, 0.5),   # 사람 1: x축 왕복
    (-1.0, -1.0, 0.3, 1.7, 'y', 3.0, 0.7),  # 사람 2: y축 왕복
    (4.0, -1.0, 0.3, 1.7, 'x', 1.5, 0.3),   # 사람 3: 느리게 x축 왕복
]

# 풀밭 높이 노이즈 (지면 roughness 시뮬레이션)
GRASS_HEIGHT_STD = 0.04   # 4cm 표준편차
GROUND_Z_BODY = -0.63     # body frame에서 지면 높이 (M300 마운트 높이)
GROUND_Z_S10 = -0.35      # mrdvs_tof frame에서 지면 높이


class FullPipelineSimulator(Node):
    def __init__(self):
        super().__init__('test_full_pipeline')

        # ── 로봇 상태 (odom frame) ──
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_v = 0.0    # 현재 선속도
        self.robot_w = 0.0    # 현재 각속도

        # ── cmd_vel 수신 ──
        self.target_v = 0.0
        self.target_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ── Publishers ──
        self.pub_m300 = self.create_publisher(
            PointCloud2, '/cloud_registered_body', 10)
        self.pub_s10 = self.create_publisher(
            PointCloud2, '/lx_camera_node/LxCamera_Cloud', 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/Odometry', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/sim/obstacles', 10)

        # ── Static TFs ──
        self.static_tf_bc = StaticTransformBroadcaster(self)
        static_tfs = []

        # odom → camera_init (identity)
        static_tfs.append(self._make_static_tf(
            'odom', 'camera_init', 0, 0, 0))
        # body → base_footprint
        static_tfs.append(self._make_static_tf(
            'body', 'base_footprint', -0.3544, 0.0287, -0.5848))
        # base_footprint → base_link (URDF)
        static_tfs.append(self._make_static_tf(
            'base_footprint', 'base_link', 0, 0, 0.1005))
        # base_link → mrdvs_tof (S10 Ultra 마운트)
        static_tfs.append(self._make_static_tf(
            'base_link', 'mrdvs_tof', 0.35, 0.0, 0.35))

        self.static_tf_bc.sendTransform(static_tfs)

        # ── Dynamic TF broadcaster ──
        self.dynamic_tf_bc = TransformBroadcaster(self)

        # ── Timers ──
        self.sim_dt = 0.05   # 20Hz 시뮬레이션
        self.timer_sim = self.create_timer(self.sim_dt, self.simulation_step)
        self.timer_sensor = self.create_timer(0.1, self.publish_sensors)  # 10Hz
        self.timer_viz = self.create_timer(1.0, self.publish_obstacle_markers)  # 1Hz

        self.sim_time = 0.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Full Pipeline Simulator Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Robot: ({self.robot_x:.1f}, {self.robot_y:.1f})')
        self.get_logger().info(f'  Static obstacles: {len(STATIC_OBSTACLES)}')
        self.get_logger().info(f'  Walls: {len(WALLS)}')
        self.get_logger().info(f'  Dynamic obstacles (people): {len(DYNAMIC_OBSTACLES)}')
        self.get_logger().info('')
        self.get_logger().info('실행 순서:')
        self.get_logger().info('  1) python3 test_full_pipeline.py')
        self.get_logger().info('  2) ros2 launch cloud_merger cloud_merger.launch.py')
        self.get_logger().info('  3) ros2 launch groundgrid amr_groundgrid.launch.py')
        self.get_logger().info('  4) ros2 launch terrain_costmap terrain_costmap.launch.py')
        self.get_logger().info('  5) ros2 launch path_planner path_planner.launch.py')
        self.get_logger().info('  6) rviz2 (Fixed Frame: odom)')
        self.get_logger().info('')
        self.get_logger().info('rviz에서 "2D Goal Pose"로 목표 지정!')
        self.get_logger().info('=' * 60)

    # =====================================================================
    # cmd_vel 수신
    # =====================================================================
    def cmd_vel_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    # =====================================================================
    # 시뮬레이션 스텝 (20Hz): 로봇 운동학 업데이트
    # =====================================================================
    def simulation_step(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        self.sim_time += self.sim_dt

        # cmd_vel 워치독 (0.5초 타임아웃)
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > 0.5:
            self.target_v = 0.0
            self.target_w = 0.0

        # 간단한 가속 시뮬레이션
        acc_limit = 1.0 * self.sim_dt
        ang_acc_limit = 2.0 * self.sim_dt
        dv = np.clip(self.target_v - self.robot_v, -acc_limit, acc_limit)
        dw = np.clip(self.target_w - self.robot_w, -ang_acc_limit, ang_acc_limit)
        self.robot_v += dv
        self.robot_w += dw

        # 차동구동 kinematics
        self.robot_x += self.robot_v * math.cos(self.robot_yaw) * self.sim_dt
        self.robot_y += self.robot_v * math.sin(self.robot_yaw) * self.sim_dt
        self.robot_yaw += self.robot_w * self.sim_dt
        # normalize yaw
        self.robot_yaw = math.atan2(
            math.sin(self.robot_yaw), math.cos(self.robot_yaw))

        # ── Dynamic TF: camera_init → body ──
        tf_dyn = TransformStamped()
        tf_dyn.header.stamp = stamp
        tf_dyn.header.frame_id = 'camera_init'
        tf_dyn.child_frame_id = 'body'
        tf_dyn.transform.translation.x = self.robot_x
        tf_dyn.transform.translation.y = self.robot_y
        tf_dyn.transform.translation.z = 0.0
        tf_dyn.transform.rotation.z = math.sin(self.robot_yaw / 2)
        tf_dyn.transform.rotation.w = math.cos(self.robot_yaw / 2)
        self.dynamic_tf_bc.sendTransform(tf_dyn)

        # ── Odometry ──
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'camera_init'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.z = math.sin(self.robot_yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.robot_yaw / 2)
        odom.twist.twist.linear.x = self.robot_v
        odom.twist.twist.angular.z = self.robot_w
        self.pub_odom.publish(odom)

        # 주기적 상태 출력
        if int(self.sim_time * 20) % 100 == 0:
            self.get_logger().info(
                f'pos=({self.robot_x:.2f}, {self.robot_y:.2f}) '
                f'yaw={math.degrees(self.robot_yaw):.1f}° '
                f'v={self.robot_v:.2f} w={self.robot_w:.2f}')

    # =====================================================================
    # 동적 장애물 현재 위치 계산
    # =====================================================================
    def get_dynamic_obstacle_pos(self, obs, t):
        cx, cy, radius, height, axis, move_range, speed = obs
        # 삼각파 왕복 운동
        period = 2.0 * move_range / speed if speed > 0 else 1e9
        phase = (t % period) / period
        offset = move_range * (1.0 - abs(2.0 * phase - 1.0)) - move_range / 2.0
        if axis == 'x':
            return cx + offset, cy
        else:
            return cx, cy + offset

    # =====================================================================
    # 센서 데이터 발행 (10Hz)
    # =====================================================================
    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()
        t = self.sim_time
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        # ── 월드→body 좌표 변환 함수 ──
        def world_to_body(wx, wy, wz):
            dx = wx - self.robot_x
            dy = wy - self.robot_y
            bx = cos_yaw * dx + sin_yaw * dy
            by = -sin_yaw * dx + cos_yaw * dy
            bz = wz
            return bx, by, bz

        # ── M300 포인트클라우드 (body frame) ──
        m300_pts = []
        m300_range = 15.0
        m300_blind = 0.5

        # (1) 지면 포인트 (풀밭 시뮬레이션)
        for r in np.arange(3.5, m300_range, 0.6):
            for a in np.arange(0, 2 * math.pi, 0.12):
                bx = r * math.cos(a)
                by = r * math.sin(a)
                # 풀밭 노이즈 (위치마다 일관된 높이 = seed)
                wx = self.robot_x + cos_yaw * bx - sin_yaw * by
                wy = self.robot_y + sin_yaw * bx + cos_yaw * by
                grass_h = GRASS_HEIGHT_STD * np.sin(wx * 3.7 + wy * 2.3)
                bz = GROUND_Z_BODY + grass_h + np.random.normal(0, 0.01)
                m300_pts.append((bx, by, bz, 40.0))

        # (2) 정적 장애물 포인트
        for (cx, cy, rad, height) in STATIC_OBSTACLES:
            for az in np.arange(0, 2 * math.pi, 0.2):
                for hz in np.arange(0, height, 0.15):
                    wx = cx + rad * math.cos(az)
                    wy = cy + rad * math.sin(az)
                    wz = hz
                    bx, by, bz = world_to_body(wx, wy, wz)
                    dist = math.hypot(bx, by)
                    if m300_blind < dist < m300_range:
                        m300_pts.append((bx, by, bz, 90.0))

        # (3) 벽 포인트
        for (x1, y1, x2, y2, thick, height) in WALLS:
            n_pts = int(math.hypot(x2 - x1, y2 - y1) / 0.15)
            for i in range(n_pts):
                frac = i / max(n_pts - 1, 1)
                wx = x1 + (x2 - x1) * frac + np.random.normal(0, thick / 2)
                wy = y1 + (y2 - y1) * frac + np.random.normal(0, thick / 2)
                for hz in np.arange(0, height, 0.2):
                    bx, by, bz = world_to_body(wx, wy, hz)
                    dist = math.hypot(bx, by)
                    if m300_blind < dist < m300_range:
                        m300_pts.append((bx, by, bz, 95.0))

        # (4) 동적 장애물 (사람) 포인트
        for obs in DYNAMIC_OBSTACLES:
            dx, dy = self.get_dynamic_obstacle_pos(obs, t)
            _, _, rad, height, _, _, _ = obs
            for az in np.arange(0, 2 * math.pi, 0.3):
                for hz in np.arange(0, height, 0.2):
                    wx = dx + rad * math.cos(az)
                    wy = dy + rad * math.sin(az)
                    bx, by, bz = world_to_body(wx, wy, hz)
                    dist = math.hypot(bx, by)
                    if m300_blind < dist < m300_range:
                        m300_pts.append((bx, by, bz, 85.0))

        self.pub_m300.publish(self._make_xyzi(m300_pts, stamp, 'body'))

        # ── S10 Ultra 포인트클라우드 (mrdvs_tof frame) ──
        # S10 마운트: base_link 기준 (0.35, 0, 0.35)
        # mrdvs_tof frame에서 전방 = +x, FOV 120°×80°
        s10_pts = []
        s10_range = 5.0

        # (1) 전방 지면
        for d in np.arange(0.3, s10_range, 0.2):
            fov_half_h = d * math.tan(math.radians(60))  # 120° horizontal
            for lat in np.arange(-fov_half_h, fov_half_h, 0.2):
                px = d
                py = lat
                pz = GROUND_Z_S10 + np.random.normal(0, GRASS_HEIGHT_STD)
                s10_pts.append((px, py, pz, 100, 120, 80))

        # (2) 전방 장애물 (가까운 것만)
        # S10 마운트의 odom 위치 계산
        s10_offset_body_x = 0.35 + 0.35  # base_footprint→base_link→mrdvs_tof
        s10_world_x = self.robot_x + cos_yaw * s10_offset_body_x
        s10_world_y = self.robot_y + sin_yaw * s10_offset_body_x

        for (cx, cy, rad, height) in STATIC_OBSTACLES:
            dist = math.hypot(cx - s10_world_x, cy - s10_world_y)
            if dist < s10_range + rad:
                for az in np.arange(0, 2 * math.pi, 0.3):
                    for hz in np.arange(GROUND_Z_S10, GROUND_Z_S10 + height, 0.2):
                        wx = cx + rad * math.cos(az)
                        wy = cy + rad * math.sin(az)
                        # world → mrdvs_tof (approximate: ignore small offset)
                        dx = wx - s10_world_x
                        dy = wy - s10_world_y
                        sx = cos_yaw * dx + sin_yaw * dy
                        sy = -sin_yaw * dx + cos_yaw * dy
                        if 0.3 < sx < s10_range and abs(sy) < sx * 1.2:
                            s10_pts.append((sx, sy, hz, 180, 180, 180))

        self.pub_s10.publish(self._make_xyzrgb(s10_pts, stamp, 'mrdvs_tof'))

    # =====================================================================
    # 장애물 시각화 마커 (rviz 디버그용)
    # =====================================================================
    def publish_obstacle_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid = 0

        # 정적 장애물
        for (cx, cy, rad, height) in STATIC_OBSTACLES:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'odom'
            m.ns = 'static'
            m.id = mid; mid += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = height / 2
            m.pose.orientation.w = 1.0
            m.scale.x = rad * 2
            m.scale.y = rad * 2
            m.scale.z = height
            m.color.r = 0.6; m.color.g = 0.3; m.color.b = 0.1; m.color.a = 0.7
            markers.markers.append(m)

        # 동적 장애물 (사람)
        for obs in DYNAMIC_OBSTACLES:
            dx, dy = self.get_dynamic_obstacle_pos(obs, self.sim_time)
            _, _, rad, height, _, _, _ = obs
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'odom'
            m.ns = 'dynamic'
            m.id = mid; mid += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = dx
            m.pose.position.y = dy
            m.pose.position.z = height / 2
            m.pose.orientation.w = 1.0
            m.scale.x = rad * 2
            m.scale.y = rad * 2
            m.scale.z = height
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.8
            markers.markers.append(m)

        self.pub_markers.publish(markers)

    # =====================================================================
    # 헬퍼 함수
    # =====================================================================
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
    node = FullPipelineSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()