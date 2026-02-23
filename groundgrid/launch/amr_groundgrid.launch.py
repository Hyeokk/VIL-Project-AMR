# AMR GroundGrid launch — Ground segmentation for DONKEYBOTI
#
# Input: /merged_cloud (M300 + S10 Ultra, body frame)
#        /Odometry (FAST-LIO2, camera_init -> body)
#
# TF chain (provided by other nodes):
#   odom -> camera_init (static identity)
#   camera_init -> body (FAST-LIO2, dynamic)
#   body -> base_footprint (static offset)
#   base_footprint -> base_link (URDF)

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
            default_value='/merged_cloud',
            description='Merged point cloud (M300 + S10 Ultra, body frame)'
        ),
        launch.actions.DeclareLaunchArgument(
            'odometry_topic',
            default_value='/Odometry',
            description='FAST-LIO2 odometry (camera_init -> body)'
        ),
    ]

    groundgrid_node = launch_ros.actions.Node(
        package='groundgrid',
        executable='groundgrid_node',
        name='groundgrid_node',
        output='screen',
        parameters=[
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