cmake_minimum_required(VERSION 3.8)
project(livox_multi_merger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Ensure find_package honors <Package>_ROOT variables (suppress CMP0074 dev warning)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(livox_merger src/merger_node.cpp)

ament_target_dependencies(livox_merger
  rclcpp
  livox_ros_driver2
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_include_directories(livox_merger PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  livox_merger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/calib_extrinsic.py
  scripts/check_fused.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
