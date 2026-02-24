#!/usr/bin/env python3
"""
test_monitor.py — Real-time pipeline performance monitor

Monitors all pipeline topics and prints rate (Hz), latency (ms),
and point counts every 2 seconds.

Usage:
    python3 test/test_monitor.py

Expected output (with test_pipeline.py + cloud_merger + groundgrid running):

    [ 10.0s]
      cloud_registered_body    | 10.0 Hz |    1.2 ms |  2772 pts
      LxCamera_Cloud           | 10.0 Hz |    1.5 ms |  1634 pts
      merged_cloud             | 10.0 Hz |    2.8 ms |  4406 pts
      filtered_cloud           | 10.0 Hz |   15.3 ms |  1205 pts
      Odometry                 | 10.0 Hz |    0.3 ms |        -
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class TopicStats:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.latency_sum = 0.0
        self.latency_count = 0
        self.last_width = 0

    def update(self, latency_ms, width=0):
        self.count += 1
        self.latency_sum += latency_ms
        self.latency_count += 1
        self.last_width = width

    def report_and_reset(self, elapsed):
        hz = self.count / elapsed if elapsed > 0 else 0
        avg_lat = (self.latency_sum / self.latency_count
                   if self.latency_count > 0 else 0)
        w = f'{self.last_width:5d} pts' if self.last_width > 0 else '       -'
        line = (f'  {self.name:<25s} | {hz:5.1f} Hz | {avg_lat:7.1f} ms | {w}')
        self.count = 0
        self.latency_sum = 0.0
        self.latency_count = 0
        return line


class PipelineMonitor(Node):
    def __init__(self):
        super().__init__('pipeline_monitor')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.stats = {}

        pc_topics = [
            ('/cloud_registered_body', 'cloud_registered_body', sensor_qos),
            ('/lx_camera_node/LxCamera_Cloud', 'LxCamera_Cloud', sensor_qos),
            ('/merged_cloud', 'merged_cloud', reliable_qos),
            ('/groundgrid/filtered_cloud', 'filtered_cloud', reliable_qos),
        ]
        for topic, label, qos in pc_topics:
            self.stats[label] = TopicStats(label)
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, l=label: self._pc_cb(msg, l), qos)

        self.stats['Odometry'] = TopicStats('Odometry')
        self.create_subscription(
            Odometry, '/Odometry',
            lambda msg: self._odom_cb(msg), sensor_qos)

        self.report_interval = 2.0
        self.timer = self.create_timer(self.report_interval, self._report)
        self.start_time = time.monotonic()

        self.get_logger().info(
            f'Pipeline monitor started (reporting every {self.report_interval:.0f}s)')
        self._print_header()

    def _get_latency_ms(self, header):
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(header.stamp)
        return (now - msg_time).nanoseconds / 1e6

    def _pc_cb(self, msg, label):
        self.stats[label].update(self._get_latency_ms(msg.header), msg.width)

    def _odom_cb(self, msg):
        self.stats['Odometry'].update(self._get_latency_ms(msg.header))

    def _print_header(self):
        print(f'\n  {"Topic":<25s} | {"Rate":>7s} | {"Latency":>9s} | {"Points":>8s}')
        print(f'  {"-"*25}-+-{"-"*7}-+-{"-"*9}-+-{"-"*8}')

    def _report(self):
        elapsed = time.monotonic() - self.start_time
        order = ['cloud_registered_body', 'LxCamera_Cloud',
                 'merged_cloud', 'filtered_cloud', 'Odometry']
        print(f'[{elapsed:6.1f}s]')
        for label in order:
            print(self.stats[label].report_and_reset(self.report_interval))
        print()


def main():
    rclpy.init()
    try:
        rclpy.spin(PipelineMonitor())
    except KeyboardInterrupt:
        print('\nMonitor stopped.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()