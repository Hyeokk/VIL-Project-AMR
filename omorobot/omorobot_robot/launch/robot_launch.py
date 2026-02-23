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
    robot_dir = get_package_share_directory('omorobot_robot')
    
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', ROBOT_MODEL+'.yaml'))

    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    
    robot_control_node = Node(
        package='omorobot_robot',
        executable='robot_control',
        name='robot_control',
        output='screen',
        emulate_tty=True,
        parameters=[robot_yaml],
        namespace=''
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_yaml_arg)
    ld.add_action(robot_control_node)
    
    return ld
