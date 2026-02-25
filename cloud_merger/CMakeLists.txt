cmake_minimum_required(VERSION 3.22)
project(cloud_merger)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

add_executable(cloud_merger_node src/cloud_merger_node.cpp)
target_include_directories(cloud_merger_node PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(cloud_merger_node ${PCL_LIBRARIES})
ament_target_dependencies(cloud_merger_node
    rclcpp sensor_msgs tf2_ros tf2 pcl_conversions)

install(TARGETS cloud_merger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME}/)

ament_package()