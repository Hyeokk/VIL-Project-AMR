# path_planner launch — A* + DWA for DONKEYBOTI caterpillar robot
#
# Prerequisites:
#   1. bringup_launch.py  (robot_control: cmd_vel → motor serial)
#   2. terrain_costmap     (/terrain_costmap at 10Hz, 5m×5m)
#   3. FAST-LIO2           (TF: odom → base_link)
#
# Input:
#   /terrain_costmap  (nav_msgs/OccupancyGrid, 50x50)
#   /goal_pose        (geometry_msgs/PoseStamped, from RViz or mission planner)
#   TF: odom → base_link
#
# Output:
#   /cmd_vel          (geometry_msgs/Twist) → robot_control → serial motor
#   /path_planner/global_path       (nav_msgs/Path, visualization)
#   /path_planner/local_trajectory  (nav_msgs/Path, visualization)
#
# Usage:
#   ros2 launch path_planner path_planner.launch.py
#
# Speed configuration:
#   All parameters are loaded from param/path_planner.yaml.
#   To change speed, edit the yaml file directly.
#   Runtime override:
#     ros2 param set /path_planner_node max_lin_vel 0.3

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('path_planner')
    config = os.path.join(pkg_dir, 'param', 'path_planner.yaml')

    path_planner_node = launch_ros.actions.Node(
        package='path_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            config,
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action(path_planner_node)
    return ld