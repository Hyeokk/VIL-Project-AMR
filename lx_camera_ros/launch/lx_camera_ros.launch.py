from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='false'),

        Node(
            package="lx_camera_ros",
            executable="lx_camera_node",
            namespace="lx_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                # --- Device ---
                {"enable_gpu": 0},
                {"ip": ""},
                {"log_level": 1},
                {"log_path": "./log/"},

                # --- Streams ---
                {"is_depth": 1},
                # 1: XYZ+RGB point cloud, 2: XYZIRT lidar-style
                {"is_xyz": 1},
                {"LX_BOOL_ENABLE_3D_AMP_STREAM": 0},
                {"LX_BOOL_ENABLE_2D_STREAM": 1},
                {"LX_BOOL_ENABLE_IMU": 0},

                # --- Point Cloud Settings ---
                # Unit: 0=mm, 1=meters (ROS2 requires meters)
                {"LX_INT_XYZ_UNIT": 1},
                # Coordinate: 0=camera(right/down/fwd), 1=robot(fwd/left/up)
                {"LX_INT_XYZ_COORDINATE": 1},

                # --- Undistortion ---
                {"LX_BOOL_ENABLE_2D_UNDISTORT": 1},
                {"LX_INT_2D_UNDISTORT_SCALE": 1},
                {"LX_BOOL_ENABLE_3D_UNDISTORT": 1},
                {"LX_INT_3D_UNDISTORT_SCALE": 1},

                # --- Install position: S10 Ultra relative to base_link ---
                # No rotation needed: SDK outputs in robot coordinate (fwd/left/up)
                {"x": 0.35},
                {"y": 0.0},
                {"z": 0.35},
                {"roll": 0.0},
                {"pitch": 0.0},
                {"yaw": 0.0},
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='lx_camera',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('lx_camera_ros'), 'rviz', 'lx_camera.rviz')],
            condition=IfCondition(enable_rviz)),
    ])