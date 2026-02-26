import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cloud_merger')
    default_param_file = os.path.join(pkg_dir, 'param', 'cloud_merger.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=default_param_file,
                              description='Path to parameter YAML file'),

        # Static TF: base_link -> mrdvs_tof (S10 Ultra mount position)
        # x=0.35m forward, z=0.35m up from base_link
        # No rotation: SDK outputs in robot coordinate (XYZ = fwd/left/up)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_mrdvs_tof',
            arguments=[
                '--x', '0.35',
                '--y', '0.0',
                '--z', '0.35',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'mrdvs_tof',
            ],
        ),

        # Cloud merger node — all parameters from YAML
        Node(
            package='cloud_merger',
            executable='cloud_merger_node',
            name='cloud_merger',
            output='screen',
            parameters=[
                LaunchConfiguration('param_file'),
            ],
        ),
    ])