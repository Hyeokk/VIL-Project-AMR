# bringup_launch.py — DONKEYBOTI motor control + URDF TF
#
# Nodes:
#   1. robot_control        — wheel motor driver (cmd_vel -> wheels)
#   2. robot_state_publisher — URDF joint TF tree
#
# Published TFs (from URDF):
#   base_footprint -> base_link       (z=0.1005)
#   base_link      -> m300_link       (x=0.335, z=0.5295)
#   m300_link      -> m300_imu_link   (x=0.0194, y=-0.0287, z=-0.0452)
#   base_link      -> wheel_left_link, wheel_right_link, imu_link
#
# Note: base_link -> mrdvs_tof is published by lx_camera_ros driver, not here.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'DONKEYBOTI')


def generate_launch_description():
    robot_dir = get_package_share_directory('omorobot_robot')
    desc_dir = get_package_share_directory('omorobot_description')

    robot_yaml = LaunchConfiguration('robot_yaml',
        default=os.path.join(robot_dir, 'param', ROBOT_MODEL + '.yaml'))
    publish_tf = LaunchConfiguration('publish_tf', default='true')
    urdf_file = os.path.join(desc_dir, 'urdf', ROBOT_MODEL + '.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('robot_yaml', default_value=robot_yaml),
        DeclareLaunchArgument('publish_tf', default_value='true',
            description='Publish odom->base_footprint TF from wheel odometry. '
                        'Set false when using FAST-LIO2 for localization.'),

        # Robot motor control (wheel driver, cmd_vel subscriber)
        Node(
            package='omorobot_robot',
            executable='robot_control',
            name='robot_control',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_yaml,
                {'publish_tf': publish_tf},
            ],
        ),

        # URDF TF publisher
        # Publishes: base_footprint -> base_link -> m300_link -> m300_imu_link
        #            base_link -> wheel_left/right_link, imu_link
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
    ])