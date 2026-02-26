#!/usr/bin/env python3
"""
Launch file for the amr_segmentation pipeline.

Starts 3 nodes + 1 static TF publisher:
  1. static_transform_publisher  -- base_link -> mrdvs_tof (camera mount)
  2. segmentation_node           -- RGB -> 7-class segmentation mask (NPU)
  3. pointcloud_painter_node     -- M300 LiDAR + mask -> painted pointcloud
  4. semantic_merger_node        -- painted + S10 depth -> semantic costmap

Prerequisites (must be running before this launch):
  - lx_camera_ros (S10 Ultra driver: RGB, depth cloud, CameraInfo, TF mrdvs_tof->mrdvs_rgb)
  - pacecat_m300 (M300 LiDAR driver)
  - fast_lio (FAST-LIO2: /cloud_registered_body, TF odom->base_link->body)

NOTE: cloud_merger.launch.py is NO LONGER required.
      The static TF (base_link -> mrdvs_tof) is now published by this launch file.

Usage:
  ros2 launch amr_segmentation pipeline.launch.py
  ros2 launch amr_segmentation pipeline.launch.py model_path:=/path/to/model.onnx
  ros2 launch amr_segmentation pipeline.launch.py cam_x:=0.40 cam_z:=0.30
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_segmentation')
    config_file = os.path.join(pkg_share, 'config', 'pipeline_params.yaml')

    # ---------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.expanduser(
            '~/models/ddrnet23_slim_unified7class_544x640_nearest.onnx'),
        description='Path to DDRNet23-Slim ONNX model',
    )

    use_qnn_arg = DeclareLaunchArgument(
        'use_qnn',
        default_value='true',
        description='Use QNN (Qualcomm NPU) for inference',
    )

    # Camera mount position (base_link -> mrdvs_tof)
    # Previously defined in cloud_merger.launch.py
    cam_x_arg = DeclareLaunchArgument(
        'cam_x', default_value='0.35',
        description='Camera X offset from base_link [m] (forward)')
    cam_y_arg = DeclareLaunchArgument(
        'cam_y', default_value='0.0',
        description='Camera Y offset from base_link [m] (left)')
    cam_z_arg = DeclareLaunchArgument(
        'cam_z', default_value='0.35',
        description='Camera Z offset from base_link [m] (up)')
    cam_roll_arg = DeclareLaunchArgument(
        'cam_roll', default_value='0.0',
        description='Camera roll [rad]')
    cam_pitch_arg = DeclareLaunchArgument(
        'cam_pitch', default_value='0.0',
        description='Camera pitch [rad]')
    cam_yaw_arg = DeclareLaunchArgument(
        'cam_yaw', default_value='0.0',
        description='Camera yaw [rad]')

    # ---------------------------------------------------------------
    # Static TF: base_link -> mrdvs_tof (camera mount position)
    # Migrated from cloud_merger.launch.py
    # ---------------------------------------------------------------

    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mrdvs_tof_tf_publisher',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'mrdvs_tof',
        ],
    )

    # ---------------------------------------------------------------
    # Node 1: Segmentation (NPU)
    # ---------------------------------------------------------------

    segmentation_node = Node(
        package='amr_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[
            config_file,
            {
                'model_path': LaunchConfiguration('model_path'),
                'use_qnn': LaunchConfiguration('use_qnn'),
            },
        ],
        output='screen',
    )

    # ---------------------------------------------------------------
    # Node 2: PointCloud Painter
    # ---------------------------------------------------------------

    painter_node = Node(
        package='amr_segmentation',
        executable='pointcloud_painter_node',
        name='pointcloud_painter_node',
        parameters=[config_file],
        output='screen',
    )

    # ---------------------------------------------------------------
    # Node 3: Semantic Merger + Costmap
    # ---------------------------------------------------------------

    merger_node = Node(
        package='amr_segmentation',
        executable='semantic_merger_node',
        name='semantic_merger_node',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        use_qnn_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        # Nodes
        static_tf_cam,
        segmentation_node,
        painter_node,
        merger_node,
    ])
