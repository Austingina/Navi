#include "rviz_global_relocalize/mock_navi_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <cmath>

using namespace std::chrono_literals;

namespace rviz_global_relocalize
{

MockNaviTool::MockNaviTool()
  : has_map_(false)
  , has_position_(false)
  , current_x_(0.0)
  , current_y_(0.0)
  , current_yaw_(0.0)
  , map_frame_("map")
  , current_path_index_(0)
  , linear_velocity_(1.0)
  , is_following_path_(false)
  , is_paused_(false)
  , in_cooldown_(false)
{
  shortcut_key_ = 'm';
}

MockNaviTool::~MockNaviTool()
{
  is_following_path_ = false;
  
  if (timer_) {
    timer_->cancel();
  }
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

void MockNaviTool::onInitialize()
{
  // Ensure rclcpp initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  node_ = std::make_shared<rclcpp::Node>("rviz_mock_navi_tool");
  
  // Create RViz properties for velocity parameters
  linear_velocity_property_ = new rviz_common::properties::FloatProperty(
    "Linear Velocity", 5.0,
    "Linear velocity in m/s for simulated movement",
    getPropertyContainer(), nullptr, this);
  linear_velocity_property_->setMin(0.5);
  linear_velocity_property_->setMax(50.0);
  
  // Create TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  
  // Subscribe to map topic to check if map is available
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    std::bind(&MockNaviTool::mapCallback, this, std::placeholders::_1));
  
  // Subscribe to /plan topic
  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1),
    std::bind(&MockNaviTool::planCallback, this, std::placeholders::_1));
  
  // Create timer to update position at 20Hz (50ms)
  timer_ = node_->create_wall_timer(
    50ms, std::bind(&MockNaviTool::timerCallback, this));
  
  // Create executor and run in separate thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() {
    executor_->spin();
  });
  
  RCLCPP_INFO(node_->get_logger(), "MockNaviTool initialized - click to set position, subscribe to /plan for path following");
}

void MockNaviTool::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_map_ = true;
  map_frame_ = msg->header.frame_id;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Received map in frame: %s", map_frame_.c_str());
}

void MockNaviTool::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (msg->poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path");
    return;
  }
  
  path_ = msg->poses;
  if (!msg->header.frame_id.empty()) {
    map_frame_ = msg->header.frame_id;
  }
  current_path_index_ = 0;
  
  // Initialize position to start of path
  current_x_ = path_[0].pose.position.x;
  current_y_ = path_[0].pose.position.y;
  current_yaw_ = getYawFromQuaternion(
    path_[0].pose.orientation.x,
    path_[0].pose.orientation.y,
    path_[0].pose.orientation.z,
    path_[0].pose.orientation.w);
  
  has_position_ = true;
  is_following_path_ = true;
  is_paused_ = false;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Received path with %zu poses. Starting from (%.2f, %.2f)",
    path_.size(), current_x_, current_y_);
}

double MockNaviTool::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double MockNaviTool::getYawFromQuaternion(double x, double y, double z, double w)
{
  // Convert quaternion to yaw (rotation around Z axis)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MockNaviTool::timerCallback()
{
  // Update velocity from properties
  linear_velocity_ = linear_velocity_property_->getFloat();
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Must have position to publish TF
  if (!has_position_) {
    return;
  }
  
  // Time step (50ms = 0.05s)
  const double dt = 0.05;
  
  // Check if in cooldown period after manual position set
  if (in_cooldown_) {
    double elapsed = (node_->now() - last_position_set_time_).seconds();
    if (elapsed >= POSITION_SET_COOLDOWN) {
      in_cooldown_ = false;
      RCLCPP_INFO(node_->get_logger(), "Cooldown ended, ready to follow path");
    }
  }
  
  // If following path and not paused and not in cooldown, update position
  if (is_following_path_ && !is_paused_ && !in_cooldown_ && !path_.empty()) {
    if (current_path_index_ < path_.size()) {
      const auto & target = path_[current_path_index_].pose;
      double target_x = target.position.x;
      double target_y = target.position.y;
      
      // Calculate distance and direction to target
      double dx = target_x - current_x_;
      double dy = target_y - current_y_;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      const double distance_threshold = 0.05;  // 5cm
      
      if (distance > distance_threshold) {
        // Always face the direction of movement
        current_yaw_ = std::atan2(dy, dx);
        
        // Move towards target
        double move = linear_velocity_ * dt;
        if (distance < move) {
          current_x_ = target_x;
          current_y_ = target_y;
        } else {
          current_x_ += move * std::cos(current_yaw_);
          current_y_ += move * std::sin(current_yaw_);
        }
      } else {
        // Reached current waypoint, move to next
        current_path_index_++;
        if (current_path_index_ < path_.size()) {
          RCLCPP_DEBUG(node_->get_logger(), "Moving to waypoint %zu/%zu", 
            current_path_index_ + 1, path_.size());
        } else {
          RCLCPP_INFO(node_->get_logger(), "Reached end of path at (%.2f, %.2f)", 
            current_x_, current_y_);
          is_following_path_ = false;
        }
      }
    }
  }
  
  // Publish TF: map -> odom (identity) and odom -> base_link
  auto now = node_->now();
  
  // map -> odom (identity transform)
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = now;
  map_to_odom.header.frame_id = map_frame_;
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;
  map_to_odom.transform.rotation.x = 0.0;
  map_to_odom.transform.rotation.y = 0.0;
  map_to_odom.transform.rotation.z = 0.0;
  map_to_odom.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(map_to_odom);
  
  // odom -> base_link (actual position)
  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = now;
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base_link";
  odom_to_base.transform.translation.x = current_x_;
  odom_to_base.transform.translation.y = current_y_;
  odom_to_base.transform.translation.z = 0.0;
  
  // Convert yaw to quaternion
  double half_yaw = current_yaw_ * 0.5;
  odom_to_base.transform.rotation.x = 0.0;
  odom_to_base.transform.rotation.y = 0.0;
  odom_to_base.transform.rotation.z = std::sin(half_yaw);
  odom_to_base.transform.rotation.w = std::cos(half_yaw);
  
  tf_broadcaster_->sendTransform(odom_to_base);
}

bool MockNaviTool::getPointOnPlane(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point)
{
  // Get the camera and viewport
  Ogre::Camera * camera = context_->getViewManager()->getCurrent()->getCamera();
  if (!camera) {
    return false;
  }
  
  Ogre::Viewport * viewport = camera->getViewport();
  if (!viewport) {
    return false;
  }
  
  // Convert screen coordinates to normalized device coordinates
  float screen_x = static_cast<float>(event.x) / static_cast<float>(viewport->getActualWidth());
  float screen_y = static_cast<float>(event.y) / static_cast<float>(viewport->getActualHeight());
  
  // Create ray from camera through the clicked point
  Ogre::Ray ray = camera->getCameraToViewportRay(screen_x, screen_y);
  
  // Create a plane at z=0 (ground plane where the map is)
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0);
  
  // Find intersection
  auto result = ray.intersects(ground_plane);
  if (result.first) {
    point = ray.getPoint(result.second);
    return true;
  }
  
  return false;
}

void MockNaviTool::activate()
{
  RCLCPP_INFO(node_->get_logger(), 
    "MockNaviTool activated - click on map to set position, or wait for /plan");
}

void MockNaviTool::deactivate()
{
  // Keep state - don't stop on deactivate
}

int MockNaviTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // Only process left mouse button release
  if (!event.leftUp()) {
    return 0;
  }
  
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_map_) {
      RCLCPP_WARN(node_->get_logger(), "No map received yet. Please wait for a map to be published on /map");
      switchToInteractTool();
      return 0;
    }
  }
  
  // Get the 3D point on the ground plane
  Ogre::Vector3 point;
  if (!getPointOnPlane(event, point)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to get point on ground plane");
    switchToInteractTool();
    return 0;
  }
  
  // Set the position
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_x_ = point.x;
    current_y_ = point.y;
    current_yaw_ = 0.0;
    has_position_ = true;
    
    // Enter cooldown period - don't follow path for 5 seconds
    in_cooldown_ = true;
    last_position_set_time_ = node_->now();
    
    // Don't clear path, just pause following
    is_paused_ = false;
    
    RCLCPP_INFO(node_->get_logger(), 
      "Set position to (%.3f, %.3f) in frame '%s'. Holding for %.0f seconds.",
      current_x_, current_y_, map_frame_.c_str(), POSITION_SET_COOLDOWN);
  }
  
  // Switch back to Interact tool
  switchToInteractTool();
  
  return 0;
}

void MockNaviTool::switchToInteractTool()
{
  if (!context_) {
    return;
  }
  
  try {
    auto tool_manager = context_->getToolManager();
    if (!tool_manager) {
      return;
    }
    
    const std::vector<std::string> candidates = {
      "Interact",
      "rviz_default_plugins::tools::Interact",
      "rviz_default_plugins/Interact"
    };
    
    bool switched = false;
    for (const auto & name : candidates) {
      for (int i = 0; i < tool_manager->numTools(); ++i) {
        rviz_common::Tool * tool = tool_manager->getTool(i);
        if (!tool) {
          continue;
        }
        
        std::string tool_name = tool->getName().toStdString();
        std::string tool_class = tool->getClassId().toStdString();
        
        if (tool_name == name || tool_class == name) {
          tool_manager->setCurrentTool(tool);
          RCLCPP_INFO(node_->get_logger(), "Switched to '%s' tool", name.c_str());
          switched = true;
          break;
        }
      }
      if (switched) {
        break;
      }
    }
    
    if (!switched) {
      RCLCPP_WARN(node_->get_logger(), "Failed to switch to Interact tool");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Exception while switching tool: %s", e.what());
  }
}

} // namespace rviz_global_relocalize

PLUGINLIB_EXPORT_CLASS(rviz_global_relocalize::MockNaviTool, rviz_common::Tool)
