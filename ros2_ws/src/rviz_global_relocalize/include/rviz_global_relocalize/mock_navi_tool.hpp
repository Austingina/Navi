#pragma once

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

namespace rviz_global_relocalize
{

class MockNaviTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  MockNaviTool();
  ~MockNaviTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void timerCallback();
  void switchToInteractTool();
  double normalizeAngle(double angle);
  double getYawFromQuaternion(double x, double y, double z, double w);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Path tracking state
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_path_index_;
  double current_x_;
  double current_y_;
  double current_yaw_;
  std::string map_frame_;
  
  // Control parameters (configurable via RViz properties)
  rviz_common::properties::FloatProperty * linear_velocity_property_;
  double linear_velocity_;   // m/s
  
  // State
  std::atomic<bool> is_running_;
  std::atomic<bool> has_path_;
  int click_count_;  // Track click count for pause/resume
  
  std::mutex mutex_;
};

} // namespace rviz_global_relocalize
