#include "rviz_global_relocalize/mock_navi_tool.hpp"
#include "rviz_global_relocalize/tf_publisher_state.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <cmath>

using namespace std::chrono_literals;

namespace rviz_global_relocalize
{

MockNaviTool::MockNaviTool()
  : current_path_index_(0)
  , current_x_(0.0)
  , current_y_(0.0)
  , current_yaw_(0.0)
  , map_frame_("map")
  , linear_velocity_(1.0)
  , is_running_(false)
  , has_path_(false)
  , click_count_(0)
{
  shortcut_key_ = 'm';
}

MockNaviTool::~MockNaviTool()
{
  is_running_ = false;
  
  // Reset state so SetBaselinkTool can publish again
  TfPublisherState::instance().setMockNaviActive(false);
  
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
  
  RCLCPP_INFO(node_->get_logger(), "MockNaviTool initialized - subscribe to /plan, shortcut key: 'm'");
}

void MockNaviTool::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (msg->poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path");
    return;
  }
  
  path_ = msg->poses;
  map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  current_path_index_ = 0;
  
  // Initialize position to start of path
  current_x_ = path_[0].pose.position.x;
  current_y_ = path_[0].pose.position.y;
  current_yaw_ = getYawFromQuaternion(
    path_[0].pose.orientation.x,
    path_[0].pose.orientation.y,
    path_[0].pose.orientation.z,
    path_[0].pose.orientation.w);
  
  has_path_ = true;
  is_running_ = true;
  
  // Notify that MockNaviTool is now publishing TF
  TfPublisherState::instance().setMockNaviActive(true);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Received path with %zu poses in frame '%s'. Starting from (%.2f, %.2f)",
    path_.size(), map_frame_.c_str(), current_x_, current_y_);
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
  
  if (!has_path_ || !is_running_ || path_.empty()) {
    return;
  }
  
  // Time step (50ms = 0.05s)
  const double dt = 0.05;
  
  // Get target pose
  if (current_path_index_ >= path_.size()) {
    // Reached end of path
    RCLCPP_INFO_ONCE(node_->get_logger(), "Reached end of path at (%.2f, %.2f)", current_x_, current_y_);
    // Keep publishing the final position
    goto publish_tf;
  }
  
  {
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
      }
    }
  }
  
publish_tf:
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

void MockNaviTool::activate()
{
  click_count_++;
  
  if (click_count_ % 2 == 1) {
    // Odd click: start/resume
    if (has_path_) {
      is_running_ = true;
      RCLCPP_INFO(node_->get_logger(), 
        "MockNaviTool activated - RUNNING (click %d). Position: (%.2f, %.2f, yaw=%.2f)",
        click_count_, current_x_, current_y_, current_yaw_);
    } else {
      RCLCPP_INFO(node_->get_logger(), 
        "MockNaviTool activated - waiting for /plan message");
    }
  } else {
    // Even click: pause and switch to Interact
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), 
      "MockNaviTool PAUSED (click %d). Position: (%.2f, %.2f). Click again to resume.",
      click_count_, current_x_, current_y_);
    switchToInteractTool();
  }
}

void MockNaviTool::deactivate()
{
  // Keep state - don't stop running on deactivate
}

int MockNaviTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // We don't need mouse events for this tool
  (void)event;
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
