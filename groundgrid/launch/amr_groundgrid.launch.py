# AMR GroundGrid launch — Ground segmentation for DONKEYBOTI + M300 + FAST-LIO2
#
# TF chain (provided by other nodes):
#   odom → camera_init (static identity)
#   camera_init → body (FAST-LIO2, dynamic)
#   body → base_footprint (static offset)
#   base_footprint → base_link (URDF)
#   base_link → m300_link (URDF)

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('groundgrid')
    config = os.path.join(pkg_dir, 'param', 'm300.yaml')

    args = [
        launch.actions.DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/cloud_registered_body',
            description='FAST-LIO2 body-frame registered cloud'
        ),
        launch.actions.DeclareLaunchArgument(
            'odometry_topic',
            default_value='/Odometry',
            description='FAST-LIO2 odometry (camera_init→body)'
        ),
    ]

    groundgrid_node = launch_ros.actions.Node(
        package='groundgrid',
        executable='groundgrid_node',
        name='groundgrid_node',
        output='screen',
        parameters=[
            {'groundgrid/dataset_name': 'live'},
            {'use_sim_time': False},
            {'groundgrid/visualize': False},
            {'groundgrid/evaluation': False},
            config,
        ],
        remappings=[
            ('/pointcloud', launch.substitutions.LaunchConfiguration('pointcloud_topic')),
            ('/groundgrid/odometry_in', launch.substitutions.LaunchConfiguration('odometry_topic')),
        ]
    )

    ld = launch.LaunchDescription(args)
    ld.add_action(groundgrid_node)
    return ld