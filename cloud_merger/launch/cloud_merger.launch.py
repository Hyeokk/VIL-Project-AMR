from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('m300_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('s10_topic', default_value='/lx_camera_node/LxCamera_Cloud'),
        DeclareLaunchArgument('merged_topic', default_value='/merged_cloud'),

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
