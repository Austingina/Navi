cmake_minimum_required(VERSION 3.5)
project(rviz_global_relocalize)

find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# Enable automatic moc for Q_OBJECT processing
set(CMAKE_AUTOMOC ON)

add_library(global_relocalize_tool SHARED src/global_relocalize_tool.cpp
  include/rviz_global_relocalize/global_relocalize_tool.hpp)

ament_target_dependencies(global_relocalize_tool
  rclcpp
  std_srvs
  rviz_common
  pluginlib
)

target_link_libraries(global_relocalize_tool 
  rviz_common::rviz_common
  Qt5::Widgets
)

# Also enable AUTOMOC on the target explicitly
set_target_properties(global_relocalize_tool PROPERTIES AUTOMOC ON)

target_include_directories(global_relocalize_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS global_relocalize_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES resources/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register plugin description with pluginlib/ament index so pluginlib can find it at runtime
if(COMMAND pluginlib_export_plugin_description_file)
  # register the plugin description under the 'rviz' plugin category
  pluginlib_export_plugin_description_file(rviz_common "resources/plugin_description.xml")
else()
  message(WARNING "pluginlib_export_plugin_description_file not available; plugin descriptions may not be registered")
endif()

ament_package()
