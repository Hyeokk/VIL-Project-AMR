from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('m300_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('s10_topic', default_value='/lx_camera_node/LxCamera_Cloud'),
        DeclareLaunchArgument('merged_topic', default_value='/merged_cloud'),

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
            parameters=[{
                'm300_topic': LaunchConfiguration('m300_topic'),
                's10_topic': LaunchConfiguration('s10_topic'),
                'merged_topic': LaunchConfiguration('merged_topic'),
                'output_frame': 'body',
                's10_stale_threshold': 0.5,
            }],
        ),
    ])