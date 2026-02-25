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
        DeclareLaunchArgument('m300_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('s10_topic', default_value='/lx_camera_node/LxCamera_Cloud'),
        DeclareLaunchArgument('merged_topic', default_value='/merged_cloud'),
        DeclareLaunchArgument('enable_height_filter', default_value='true'),
        DeclareLaunchArgument('max_height_from_ground', default_value='2.0',
                              description='Max height from ground [m]'),
        DeclareLaunchArgument('sensor_height', default_value='0.63',
                              description='Body frame height from ground [m]'),

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

        # Cloud merger node
        Node(
            package='cloud_merger',
            executable='cloud_merger_node',
            name='cloud_merger',
            output='screen',
            parameters=[
                LaunchConfiguration('param_file'),    # YAML base params
                {                                     # CLI overrides
                    'm300_topic': LaunchConfiguration('m300_topic'),
                    's10_topic': LaunchConfiguration('s10_topic'),
                    'merged_topic': LaunchConfiguration('merged_topic'),
                    'enable_height_filter': LaunchConfiguration('enable_height_filter'),
                    'max_height_from_ground': LaunchConfiguration('max_height_from_ground'),
                    'sensor_height': LaunchConfiguration('sensor_height'),
                },
            ],
        ),
    ])