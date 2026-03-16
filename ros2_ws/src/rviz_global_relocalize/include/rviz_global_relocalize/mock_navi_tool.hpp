#pragma once

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreVector.h>

#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

namespace rviz_global_relocalize
{

/**
 * MockNaviTool - Combined tool for mock navigation testing
 * 
 * Features:
 * 1. Click on map to set initial robot position (base_link)
 * 2. Subscribe to /plan topic, automatically move robot along path
 * 3. Publish odom->base_link TF continuously
 * 
 * Usage:
 * - Activate tool and click on map to set initial position
 * - When /plan is received, robot starts moving along path
 * - Click again to pause/resume movement
 */
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
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void poseInTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timerCallback();
  void switchToInteractTool();
  bool getPointOnPlane(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point);
  double normalizeAngle(double angle);
  double getYawFromQuaternion(double x, double y, double z, double w);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_in_target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Map state
  bool has_map_;
  std::string map_frame_;
  
  // Position state (from click or path)
  bool has_position_;
  double current_x_;
  double current_y_;
  double current_yaw_;
  
  // Path tracking state
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_path_index_;
  
  // Control parameters (configurable via RViz properties)
  rviz_common::properties::FloatProperty * linear_velocity_property_;
  double linear_velocity_;   // m/s
  
  // State
  std::atomic<bool> is_following_path_;  // Currently following a path
  std::atomic<bool> is_paused_;          // Movement paused
  
  // Cooldown after manual position set (5 seconds)
  rclcpp::Time last_position_set_time_;
  bool in_cooldown_;
  static constexpr double POSITION_SET_COOLDOWN = 5.0;  // seconds
  
  // cmd_vel monitoring
  rclcpp::Time last_cmd_vel_time_;
  bool cmd_vel_active_;
  static constexpr double CMD_VEL_TIMEOUT = 0.5;  // seconds without cmd_vel to stop
  
  std::mutex mutex_;
};

} // namespace rviz_global_relocalize
