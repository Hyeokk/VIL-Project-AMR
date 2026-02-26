import os
import math
import signal
import rclpy
from rclpy.node import Node
from .packet_handler import *
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist, Pose

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.declare_parameter('port.name', '/dev/ttyMCU')
        self.declare_parameter('port.baudrate', 115200)
        self.declare_parameter('wheel.separation', 0.18)
        self.declare_parameter('wheel.radius', 0.034)
        self.declare_parameter('motor.gear_ratio', 34.0)
        self.declare_parameter('motor.max_lin_vel', 0.3)
        self.declare_parameter('motor.max_ang_vel', 0.5)
        self.declare_parameter('sensor.enc_pulse', 4000.0)
        self.declare_parameter('sensor.old_enc_pulse', 44.0)
        self.declare_parameter('sensor.new_enc_pulse', 1440.0)
        self.declare_parameter('sensor.use_imu', False)
        self.declare_parameter('publish_tf', True)
        
        port = self.get_parameter('port.name').value
        baudrate = self.get_parameter('port.baudrate').value
        self.wheel_separation = self.get_parameter('wheel.separation').value
        self.wheel_radius = self.get_parameter('wheel.radius').value
        self.motor_gear_ratio = self.get_parameter('motor.gear_ratio').value
        self.motor_max_lin_vel = self.get_parameter('motor.max_lin_vel').value
        self.motor_max_ang_vel = self.get_parameter('motor.max_ang_vel').value
        self.publish_tf_ = self.get_parameter('publish_tf').value
        robot_model = os.getenv('ROBOT_MODEL', 'DONKEYBOTI')
        self.print(f'ROBOT_MODEL: {robot_model}')
        motor_model = os.getenv('MOTOR_MODEL', 'NEW')
        self.print(f'MOTOR_MODEL: {motor_model}')
        if robot_model == 'R2MINI':
            if motor_model == 'NEW':
                self.enc_pulse = self.get_parameter('sensor.new_enc_pulse').value
            elif motor_model == 'OLD':
                self.enc_pulse = self.get_parameter('sensor.old_enc_pulse').value
        else: # R2, DONKEYBOTI, ETC
            self.enc_pulse = self.get_parameter('sensor.enc_pulse').value
        self.distance_per_pulse = 2.0 * math.pi * self.wheel_radius / self.enc_pulse / self.motor_gear_ratio
        self.use_imu = self.get_parameter('sensor.use_imu').value
        self.print(f'{"port.name":20} {port}')
        self.print(f'{"port.baudrate":20} {baudrate}')
        self.print(f'{"wheel.separation":20} {self.wheel_separation}')
        self.print(f'{"wheel.radius":20} {self.wheel_radius}')
        self.print(f'{"motor.gear_ratio":20} {self.motor_gear_ratio}')
        self.print(f'{"motor.max_lin_vel":20} {self.motor_max_lin_vel}')
        self.print(f'{"motor.max_ang_vel":20} {self.motor_max_ang_vel}')
        self.print(f'{"sensor.enc_pulse":20} {self.enc_pulse}')
        self.print(f'{"distance per pulse":20} {self.distance_per_pulse}')
        self.print(f'{"sensor.use_imu":20} {self.use_imu}')
        self.print(f'{"publish_tf":20} {self.publish_tf_}')

        self.ph = PacketHandler(port, baudrate)
        self.ph.start_communication()

        qos_profile = QoSProfile(depth=5)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_joint_state = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        if self.use_imu:
            self.pub_pose = self.create_publisher(Pose, 'pose', qos_profile)
        if self.publish_tf_:
            self.tf_bc = TransformBroadcaster(self)
        self.timer_5ms = self.create_timer(0.005, self.update_encoder)
        self.timer_10ms = self.create_timer(0.01, self.update_robot)
        self.timer_cmd = self.create_timer(0.02, self.update_cmd_vel)  # 50Hz 속도 스무딩

        self.ph.read_packet()
        self.enc_lh, self.enc_rh = None, None
        self.enc_lh_pre, self.enc_rh_pre = self.ph._enc[0], self.ph._enc[1]
        self.delta_lh, self.delta_rh = None, None
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.lin_vel, self.ang_vel = 0.0, 0.0
        self.time_now = None
        self.time_pre = self.get_clock().now()
        self.wheel_lh_pos, self.wheel_rh_pos = 0.0, 0.0

        # ── 속도 스무딩 & 안전 워치독 ──
        self.target_v = 0.0         # cmd_vel에서 받은 목표 속도
        self.target_w = 0.0
        self.smooth_v = 0.0         # 실제 모터에 보내는 스무딩된 속도
        self.smooth_w = 0.0
        self.smooth_alpha = 0.3     # 스무딩 계수 (0.0~1.0, 작을수록 부드러움)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.3      # [수정] 0.5→0.3초: cmd_vel 없으면 즉시 정지

        # 종료 플래그
        self._shutting_down = False

    def print(self, str_info):
        self.get_logger().info(str_info)

    def cmd_vel_callback(self, msg):
        v = max(-self.motor_max_lin_vel, min(self.motor_max_lin_vel, msg.linear.x))
        w = max(-self.motor_max_ang_vel, min(self.motor_max_ang_vel, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()
        self.target_v = v
        self.target_w = w

    def update_cmd_vel(self):
        """50Hz로 속도를 보간하여 모터에 전송 (스무딩 + 워치독)"""
        # cmd_vel 워치독: 일정 시간 cmd_vel이 안 오면 즉시 정지
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            # [수정] EMA 스무딩 우회 — 즉시 정지
            self.target_v = 0.0
            self.target_w = 0.0
            self.smooth_v = 0.0
            self.smooth_w = 0.0
            self.ph.vw_command(0.0, 0.0)
            return

        # 지수 이동 평균(EMA)으로 스무딩
        self.smooth_v += self.smooth_alpha * (self.target_v - self.smooth_v)
        self.smooth_w += self.smooth_alpha * (self.target_w - self.smooth_w)

        # 아주 작은 값은 0으로 처리 (모터 떨림 방지)
        if abs(self.smooth_v) < 0.005:
            self.smooth_v = 0.0
        if abs(self.smooth_w) < 0.005:
            self.smooth_w = 0.0

        self.ph.vw_command(self.smooth_v * 1000.0, self.smooth_w * 1000.0)

    def emergency_stop(self):
        """안전한 긴급 정지: 모터에 직접 정지 명령"""
        self.print('Emergency stop: sending motor stop command')
        self.target_v = 0.0
        self.target_w = 0.0
        self.smooth_v = 0.0
        self.smooth_w = 0.0
        try:
            for _ in range(5):
                self.ph.vw_command(0.0, 0.0)
                import time
                time.sleep(0.02)
        except Exception:
            pass

    def update_encoder(self):
        self.ph.read_packet()
        self.enc_lh = self.ph._enc[0]
        self.enc_rh = self.ph._enc[1]

    def update_robot(self):
        self.time_now = self.get_clock().now()
        dt = (self.time_now - self.time_pre).nanoseconds * 1e-9
        self.time_pre = self.time_now
        self.update_odometry(self.time_now, dt)
        self.update_jointstate(self.time_now)
        if self.use_imu:
            self.update_pose(self.ph._pose[0], self.ph._pose[1], self.ph._pose[2])

    def update_odometry(self, time_now, dt):
        self.delta_lh, self.delta_rh = self.enc_lh - self.enc_lh_pre, self.enc_rh - self.enc_rh_pre
        self.enc_lh_pre, self.enc_rh_pre = self.enc_lh, self.enc_rh
        dist_lh, dist_rh = self.delta_lh * self.distance_per_pulse, self.delta_rh * self.distance_per_pulse
        delta_s = (dist_lh + dist_rh) / 2.0
        delta_theta = (dist_rh - dist_lh) / self.wheel_separation
        self.x += delta_s * math.cos(self.theta + (delta_theta / 2.0))
        self.y += delta_s * math.sin(self.theta + (delta_theta / 2.0))
        self.theta += delta_theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        self.lin_vel = delta_s / dt
        self.ang_vel = delta_theta / dt
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = time_now.to_msg()
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0
        odometry.pose.pose.orientation.z = math.sin(self.theta/2.0)
        odometry.pose.pose.orientation.w = math.cos(self.theta/2.0)
        odometry.pose.covariance = [
            0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    1e3,    0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1e3,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1e3,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.05
        ]
        odometry.child_frame_id = "base_footprint"
        odometry.twist.twist.linear.x = self.lin_vel
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = self.ang_vel
        odometry.twist.covariance = [
            0.1,    0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.1,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    1e3,    0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1e3,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1e3,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.2
        ]
        self.pub_odom.publish(odometry)

        # TF broadcast (disabled when using FAST-LIO2 to avoid conflicts)
        if self.publish_tf_:
            odom_tf = TransformStamped()
            odom_tf.header.stamp = odometry.header.stamp
            odom_tf.header.frame_id = odometry.header.frame_id
            odom_tf.child_frame_id = odometry.child_frame_id
            odom_tf.transform.translation.x = odometry.pose.pose.position.x
            odom_tf.transform.translation.y = odometry.pose.pose.position.y
            odom_tf.transform.translation.z = odometry.pose.pose.position.z
            odom_tf.transform.rotation = odometry.pose.pose.orientation
            self.tf_bc.sendTransform(odom_tf)

    def update_jointstate(self, time_now):
        self.wheel_lh_pos += self.delta_lh * self.distance_per_pulse / self.wheel_radius
        self.wheel_rh_pos += self.delta_rh * self.distance_per_pulse / self.wheel_radius
        wheel_lh_vel = (self.lin_vel - (self.wheel_separation / 2.0) * self.ang_vel) / self.wheel_radius
        wheel_rh_vel = (self.lin_vel + (self.wheel_separation / 2.0) * self.ang_vel) / self.wheel_radius
        jointstate = JointState()
        jointstate.header.frame_id = "base_link"
        jointstate.header.stamp = time_now.to_msg()
        jointstate.name = ['wheel_left_joint', 'wheel_right_joint']
        jointstate.position = [self.wheel_lh_pos, self.wheel_rh_pos]
        jointstate.velocity = [wheel_lh_vel, wheel_rh_vel]
        jointstate.effort = []
        self.pub_joint_state.publish(jointstate)

    def update_pose(self, pose_roll, pose_pitch, pose_yaw):
        pose = Pose()
        pose.orientation.x = pose_roll
        pose.orientation.y = pose_pitch
        pose.orientation.z = pose_yaw
        self.pub_pose.publish(pose)

    def safe_shutdown(self):
        """안전한 종료 처리: 모터 정지 → 타이머 해제 → 시리얼 닫기"""
        if self._shutting_down:
            return
        self._shutting_down = True

        self.print('Safe shutdown initiated')
        # 1. 모터 즉시 정지 (가장 중요!)
        self.emergency_stop()
        # 2. 타이머 해제
        try:
            self.destroy_timer(self.timer_5ms)
            self.destroy_timer(self.timer_10ms)
            self.destroy_timer(self.timer_cmd)
        except Exception:
            pass
        # 3. 시리얼 포트 닫기 (close_port 내에서도 정지 명령 전송)
        try:
            self.ph.close_port()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()

    # SIGINT(Ctrl+C), SIGTERM 시그널에 안전한 종료 처리 등록
    def signal_handler(sig, frame):
        node.safe_shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(f'Exception occurred: {e}')
    finally:
        node.safe_shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()