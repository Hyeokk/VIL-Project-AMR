cmake_minimum_required(VERSION 3.8)
project(pacecat_m300_driver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pacecat_m300_inter REQUIRED)

# SDK sources (vendor code — relaxed warnings)
set(PACECAT_SDK_SRC
  sdk/pacecatlidarsdk.cpp
  sdk/global.cpp
  sdk/playback.cpp
  sdk/upgrade.cpp
)

# Main driver node
add_executable(driver src/driver.cpp ${PACECAT_SDK_SRC})
target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driver PUBLIC c_std_99 cxx_std_17)

# Apply strict warnings only to our code, not vendor SDK
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(src/driver.cpp PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra"
  )
  # SDK: suppress pedantic warnings (zero-length arrays, etc.)
  set_source_files_properties(${PACECAT_SDK_SRC} PROPERTIES
    COMPILE_FLAGS "-Wall -Wno-pedantic"
  )
endif()

ament_target_dependencies(driver
  rclcpp
  sensor_msgs
  pacecat_m300_inter
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
