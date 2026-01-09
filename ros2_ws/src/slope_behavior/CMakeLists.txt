cmake_minimum_required(VERSION 3.5)
project(slope_behavior)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  grid_map_msgs
  grid_map_ros
  tf2_ros
  pluginlib
  behaviortree_cpp_v3
)

include_directories(include)

# ============================================================================
# BT 节点插件库 (用于 bt_navigator 的 plugin_lib_names)
# ============================================================================
add_library(slope_bt_nodes SHARED
  src/is_slope_ahead_condition.cpp
  src/gravity_control_action.cpp
  src/bt_register_nodes.cpp
)

# 关键：定义 BT_PLUGIN_EXPORT 宏，使 BT_REGISTER_NODES 正确导出符号
target_compile_definitions(slope_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(slope_bt_nodes ${dependencies})

# 确保符号可见性，并防止链接器优化掉 BT_RegisterNodesFromPlugin
target_compile_options(slope_bt_nodes PRIVATE -fvisibility=default)
set_target_properties(slope_bt_nodes PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)
# 强制保留 BT_RegisterNodesFromPlugin 符号
target_link_options(slope_bt_nodes PRIVATE
  "LINKER:--no-as-needed"
  "LINKER:--whole-archive"
  "LINKER:--no-whole-archive"
)

# ============================================================================
# Behavior Server 插件库 (可选，如果需要作为 behavior_server 插件)
# ============================================================================
# add_library(${PROJECT_NAME} SHARED
#   src/slope_behavior.cpp
# )
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install targets
install(TARGETS slope_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install behavior trees
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install launch/scripts (optional reference files)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(slope_bt_nodes)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
