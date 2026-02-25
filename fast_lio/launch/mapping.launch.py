import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    default_config = os.path.join(fast_lio_dir, 'config', 'm300.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # FAST-LIO2
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # odom → camera_init (identity, world-fixed frames)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
        ),

        # body → base_footprint
        #   base_footprint → m300_imu_link(=body) = (0.3544, -0.0287, 0.5848)
        #   inverse: (-0.3544, 0.0287, -0.5848)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_body_base_footprint',
            arguments=['-0.3544', '0.0287', '-0.5848', '0', '0', '0',
                       'body', 'base_footprint'],
        ),
    ])