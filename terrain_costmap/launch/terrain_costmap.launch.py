# terrain_costmap launch — traversability costmap from GroundGrid
#
# Input: /groundgrid/grid_map, /groundgrid/filtered_cloud
#        TF: odom -> base_link chain (provided by FAST-LIO2 + static TFs)
#
# Output: /terrain_costmap (nav_msgs/OccupancyGrid)
#         /terrain_costmap/terrain_cloud (PointCloud2, debug)
#
# All parameters from YAML (param/terrain_costmap.yaml)

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('terrain_costmap')
    config = os.path.join(pkg_dir, 'param', 'terrain_costmap.yaml')

    terrain_costmap_node = launch_ros.actions.Node(
        package='terrain_costmap',
        executable='terrain_costmap_node',
        name='terrain_costmap_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            config,
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action(terrain_costmap_node)
    return ld