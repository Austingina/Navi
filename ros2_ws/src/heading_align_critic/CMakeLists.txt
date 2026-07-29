cmake_minimum_required(VERSION 3.5)
project(heading_align_critic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  angles
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_costmap_2d
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
)

add_library(heading_align_critic SHARED
  src/heading_align.cpp
)
target_include_directories(heading_align_critic PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(heading_align_critic ${dependencies})

pluginlib_export_plugin_description_file(dwb_core heading_align_plugins.xml)

install(TARGETS heading_align_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES heading_align_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(heading_align_critic)
ament_export_dependencies(${dependencies})

ament_package()
