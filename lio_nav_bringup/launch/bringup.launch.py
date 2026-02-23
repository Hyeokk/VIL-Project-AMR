# AMR Master Bringup Launch
# Starts all nodes for DONKEYBOTI autonomous navigation:
#   1. Motor control (omorobot_robot) — publish_tf=false
#   2. Robot state publisher (URDF TF for link frames)
#   3. Pacecat M300 LiDAR driver
#   4. FAST-LIO2 (LiDAR-Inertial Odometry)
#   5. Static TF bridges (odom↔camera_init, body↔base_footprint)
#   6. GroundGrid (ground segmentation)
#
# TF Tree:
#   odom ──(static identity)──> camera_init ──(FAST-LIO2)──> body
#     ──(static offset)──> base_footprint ──(URDF)──> base_link
#       ├── wheel_left_link, wheel_right_link (joint_states)
#       ├── m300_link ──> m300_imu_link
#       └── s10_ultra_link ──> optical frames
#
# Usage:
#   ros2 launch lio_nav_bringup bringup.launch.py
#   ros2 launch lio_nav_bringup bringup.launch.py rviz:=true

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'DONKEYBOTI')


def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    m300_dir = get_package_share_directory('pacecat_m300_driver')
    description_dir = get_package_share_directory('omorobot_description')
    robot_dir = get_package_share_directory('omorobot_robot')
    groundgrid_dir = get_package_share_directory('groundgrid')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # --- Launch Arguments ---
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false',
            description='Launch RViz (CPU-heavy, disabled by default)'),
    ]

    # ─── 1. Motor Control (publish_tf=false, FAST-LIO2 provides odom) ───
    robot_yaml = os.path.join(robot_dir, 'param', ROBOT_MODEL + '.yaml')
    motor_node = Node(
        package='omorobot_robot',
        executable='robot_control',
        name='robot_control',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_yaml,
            {'publish_tf': False},
        ],
    )

    # ─── 2. Robot State Publisher (URDF link frames) ───
    urdf_path = os.path.join(description_dir, 'urdf', ROBOT_MODEL + '.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    # ─── 3. Pacecat M300 LiDAR Driver ───
    m300_driver = Node(
        package='pacecat_m300_driver',
        executable='driver',
        name='pacecat_m300_driver',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(m300_dir, 'params', 'LDS-M300-E.yaml')],
    )

    # ─── 4. FAST-LIO2 ───
    fast_lio_config = os.path.join(fast_lio_dir, 'config', 'm300.yaml')
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fast_lio_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ─── 5. Static TF Bridges ───
    # odom → camera_init (identity — both are world-fixed frames)
    tf_odom_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )

    # body → base_footprint
    #   base_footprint → m300_imu_link(=body) = (0.3544, -0.0287, 0.5848)
    #   inverse: body → base_footprint = (-0.3544, 0.0287, -0.5848)
    tf_body_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_base_footprint',
        arguments=['-0.3544', '0.0287', '-0.5848', '0', '0', '0',
                   'body', 'base_footprint'],
    )

    # ─── 6. GroundGrid ───
    groundgrid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(groundgrid_dir, 'launch', 'amr_groundgrid.launch.py')
        ),
    )

    # ─── 7. RViz (optional, disabled by default) ───
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    # ─── Assemble ───
    ld = LaunchDescription(args)
    ld.add_action(motor_node)
    ld.add_action(robot_state_pub)
    ld.add_action(m300_driver)
    ld.add_action(fast_lio_node)
    ld.add_action(tf_odom_camera_init)
    ld.add_action(tf_body_base_footprint)
    ld.add_action(groundgrid_launch)
    ld.add_action(rviz_node)
    return ld