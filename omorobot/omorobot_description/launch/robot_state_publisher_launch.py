import os
from launch_ros.actions import Node
from launch import LaunchDescription
from rclpy.logging import get_logger
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'R2MINI')

def generate_launch_description():
    description_dir = get_package_share_directory('omorobot_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    with open(os.path.join(description_dir, 'urdf', ROBOT_MODEL+'.urdf'), 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)

    return ld
