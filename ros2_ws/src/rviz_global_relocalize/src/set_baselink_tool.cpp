#include "rviz_global_relocalize/set_baselink_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <OgreCamera.h>
#include <OgreViewport.h>

using namespace std::chrono_literals;

namespace rviz_global_relocalize
{

SetBaselinkTool::SetBaselinkTool()
  : has_map_(false)
  , has_position_(false)
  , position_x_(0.0)
  , position_y_(0.0)
  , position_z_(0.0)
  , map_frame_("map")
{
  shortcut_key_ = 'b';
}

SetBaselinkTool::~SetBaselinkTool()
{
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

void SetBaselinkTool::onInitialize()
{
  // Ensure rclcpp initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  node_ = std::make_shared<rclcpp::Node>("rviz_set_baselink_tool");
  
  // Create TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  
  // Subscribe to map topic to check if map is available
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    std::bind(&SetBaselinkTool::mapCallback, this, std::placeholders::_1));
  
  // Create timer to publish TF at 5Hz (200ms)
  timer_ = node_->create_wall_timer(
    200ms, std::bind(&SetBaselinkTool::timerCallback, this));
  
  // Create executor and run in separate thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() {
    executor_->spin();
  });
  
  RCLCPP_INFO(node_->get_logger(), "SetBaselinkTool initialized");
}

void SetBaselinkTool::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_map_ = true;
  map_frame_ = msg->header.frame_id;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Received map in frame: %s", map_frame_.c_str());
}

void SetBaselinkTool::timerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_position_) {
    return;
  }
  
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->now();
  t.header.frame_id = map_frame_;
  t.child_frame_id = "base_link";
  
  t.transform.translation.x = position_x_;
  t.transform.translation.y = position_y_;
  t.transform.translation.z = position_z_;
  
  // Identity quaternion (no rotation)
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  
  tf_broadcaster_->sendTransform(t);
}

void SetBaselinkTool::activate()
{
  RCLCPP_INFO(node_->get_logger(), "SetBaselinkTool activated - click on the map to set base_link position");
}

void SetBaselinkTool::deactivate()
{
  // no-op
}

bool SetBaselinkTool::getPointOnPlane(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point)
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

void SetBaselinkTool::switchToInteractTool()
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
          RCLCPP_INFO(node_->get_logger(), "Switched RViz tool to '%s'", name.c_str());
          switched = true;
          break;
        }
      }
      if (switched) {
        break;
      }
    }
    
    if (!switched) {
      RCLCPP_WARN(node_->get_logger(), "Failed to switch RViz tool to Interact");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Exception while switching RViz tool: %s", e.what());
  }
}

int SetBaselinkTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
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
    position_x_ = point.x;
    position_y_ = point.y;
    position_z_ = 0.0;  // Always on ground plane
    has_position_ = true;
    
    RCLCPP_INFO(node_->get_logger(), 
      "Set base_link position to (%.3f, %.3f, %.3f) in frame '%s'. Publishing TF at 5Hz.",
      position_x_, position_y_, position_z_, map_frame_.c_str());
  }
  
  // Switch back to Interact tool
  switchToInteractTool();
  
  return 0;
}

} // namespace rviz_global_relocalize

PLUGINLIB_EXPORT_CLASS(rviz_global_relocalize::SetBaselinkTool, rviz_common::Tool)
