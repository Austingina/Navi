#pragma once

#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreVector.h>

namespace rviz_global_relocalize
{

class SetBaselinkTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  SetBaselinkTool();
  ~SetBaselinkTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerCallback();
  void switchToInteractTool();
  bool getPointOnPlane(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  bool has_map_;
  bool has_position_;
  double position_x_;
  double position_y_;
  double position_z_;
  std::string map_frame_;
  
  std::mutex mutex_;
};

} // namespace rviz_global_relocalize
