# bringup_launch.py — Motor control only
# Starts omorobot_robot (robot_control node) for wheel motor driving.
# All sensor drivers and state publishers are launched separately.

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'DONKEYBOTI')

def generate_launch_description():
    robot_dir = get_package_share_directory('omorobot_robot')

    robot_yaml = LaunchConfiguration('robot_yaml',
        default=os.path.join(robot_dir, 'param', ROBOT_MODEL + '.yaml'))
    publish_tf = LaunchConfiguration('publish_tf', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('robot_yaml', default_value=robot_yaml),
        DeclareLaunchArgument('publish_tf', default_value='true',
            description='Publish odom->base_footprint TF. Set false when using FAST-LIO2.'),

        # Robot motor control node
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
    ])
