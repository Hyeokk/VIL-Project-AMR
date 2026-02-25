# terrain_costmap launch — 3m×3m traversability costmap from GroundGrid
#
# Input: /groundgrid/grid_map, /groundgrid/filtered_cloud
#        TF: odom → base_link chain (provided by FAST-LIO2 + static TFs)
#
# Output: /terrain_costmap (nav_msgs/OccupancyGrid)
#         /terrain_costmap/terrain_cloud (PointCloud2, debug)

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('terrain_costmap')
    config = os.path.join(pkg_dir, 'param', 'terrain_costmap.yaml')

    args = [
        launch.actions.DeclareLaunchArgument(
            'costmap_size', default_value='5.0',
            description='Costmap side length [m]'
        ),
        launch.actions.DeclareLaunchArgument(
            'costmap_resolution', default_value='0.1',
            description='Costmap cell size [m]'
        ),
        launch.actions.DeclareLaunchArgument(
            'enable_terrain_accumulation', default_value='true',
            description='Accumulate ground points for blind-spot compensation'
        ),
    ]

    terrain_costmap_node = launch_ros.actions.Node(
        package='terrain_costmap',
        executable='terrain_costmap_node',
        name='terrain_costmap_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            config,
            {
                'costmap_size': launch.substitutions.LaunchConfiguration('costmap_size'),
                'costmap_resolution': launch.substitutions.LaunchConfiguration('costmap_resolution'),
                'enable_terrain_accumulation': launch.substitutions.LaunchConfiguration('enable_terrain_accumulation'),
            },
        ],
    )

    ld = launch.LaunchDescription(args)
    ld.add_action(terrain_costmap_node)
    return ld