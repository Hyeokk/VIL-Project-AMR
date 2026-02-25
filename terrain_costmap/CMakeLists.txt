cmake_minimum_required(VERSION 3.22)
project(terrain_costmap)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

add_executable(terrain_costmap_node src/terrain_costmap_node.cpp)
target_include_directories(terrain_costmap_node PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(terrain_costmap_node ${PCL_LIBRARIES})
ament_target_dependencies(terrain_costmap_node
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    grid_map_ros
    grid_map_msgs
    grid_map_core
    tf2_ros
    tf2
    tf2_geometry_msgs
    pcl_conversions
)

install(TARGETS terrain_costmap_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME}/)

ament_package()