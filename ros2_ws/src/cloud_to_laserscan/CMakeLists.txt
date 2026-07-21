cmake_minimum_required(VERSION 3.8)
project(cloud_to_laserscan)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Ensure find_package honors <Package>_ROOT variables (suppress CMP0074 dev warning)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_to_2d_laserscan src/cloud_to_2d_laserscan.cpp)

ament_target_dependencies(cloud_to_2d_laserscan
  rclcpp
  sensor_msgs
  nav_msgs
  grid_map_msgs
  grid_map_ros
  tf2_ros
  pcl_conversions
)

target_link_libraries(cloud_to_2d_laserscan ${PCL_LIBRARIES})

add_executable(terrain_scan src/terrain_scan_node.cpp)

ament_target_dependencies(terrain_scan
  rclcpp
  sensor_msgs
  nav_msgs
  grid_map_msgs
  grid_map_ros
  tf2_ros
  pcl_conversions
)

target_link_libraries(terrain_scan ${PCL_LIBRARIES})

install(TARGETS
  cloud_to_2d_laserscan
  terrain_scan
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
