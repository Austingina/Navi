#include "rviz_global_relocalize/global_relocalize_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

namespace rviz_global_relocalize
{
GlobalRelocalizeTool::GlobalRelocalizeTool()
{
  shortcut_key_ = 'r';
}

GlobalRelocalizeTool::~GlobalRelocalizeTool() = default;

void GlobalRelocalizeTool::onInitialize()
{
  // Ensure rclcpp initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node_ = std::make_shared<rclcpp::Node>("rviz_global_relocalize_tool");
  client_ = node_->create_client<std_srvs::srv::Trigger>("/global_relocalize");
}

void GlobalRelocalizeTool::activate()
{
  // if (!client_->wait_for_service(500ms)) {
  //   RCLCPP_WARN(node_->get_logger(), "/global_relocalize service not available");
  //   return;
  // }

  // auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  // auto fut = client_->async_send_request(req);
  // auto ret = rclcpp::spin_until_future_complete(node_, fut, 2s);
  // if (ret == rclcpp::FutureReturnCode::SUCCESS) {
  //   auto resp = fut.get();
  //   if (resp->success) {
  //     RCLCPP_INFO(node_->get_logger(), "global_relocalize triggered: %s", resp->message.c_str());
  //   } else {
  //     RCLCPP_WARN(node_->get_logger(), "global_relocalize failed: %s", resp->message.c_str());
  //   }
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "Service call to /global_relocalize failed or timed out");
  // }
}

void GlobalRelocalizeTool::deactivate()
{
  // no-op for now; required by rviz_common::Tool interface
}

int GlobalRelocalizeTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // Trigger service on left mouse button release
  if (event.leftUp()) {
    if (!client_->wait_for_service(500ms)) {
      RCLCPP_WARN(node_->get_logger(), "/global_relocalize service not available");
      return 0;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client_->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(node_, fut, 2s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = fut.get();
      if (resp->success) {
        RCLCPP_INFO(node_->get_logger(), "global_relocalize triggered: %s", resp->message.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(), "global_relocalize failed: %s", resp->message.c_str());
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service call to /global_relocalize failed or timed out");
    }

    // After calling the service, restore RViz's tool to Interact (try several common names).
    if (context_) {
      try {
        auto tool_manager = context_->getToolManager();
        if (tool_manager) {
          const std::vector<std::string> candidates = {
            "Interact",
            "rviz_default_plugins::tools::Interact",
            "rviz_default_plugins/Interact"
          };
          bool switched = false;
          for (const auto &name : candidates) {
            // The ToolManager API in this ROS version does not provide
            // setCurrentToolByName(). Find a matching tool by name or class id
            // and call setCurrentTool(Tool*).
            bool found = false;
            for (int i = 0; i < tool_manager->numTools(); ++i) {
              rviz_common::Tool * tool = tool_manager->getTool(i);
              if (!tool) {
                continue;
              }
              // Compare against the tool's runtime name and class id
              std::string tool_name = tool->getName().toStdString();
              std::string tool_class = tool->getClassId().toStdString();
              if (tool_name == name || tool_class == name) {
                tool_manager->setCurrentTool(tool);
                RCLCPP_INFO(node_->get_logger(), "Switched RViz tool to '%s'", name.c_str());
                switched = true;
                found = true;
                break;
              }
            }
            if (found) {
              break;
            }
          }
          if (!switched) {
            RCLCPP_WARN(node_->get_logger(), "Failed to switch RViz tool to Interact (no candidate matched)");
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Exception while switching RViz tool: %s", e.what());
      }
    }
  }
  return 0;
}

} // namespace rviz_global_relocalize

PLUGINLIB_EXPORT_CLASS(rviz_global_relocalize::GlobalRelocalizeTool, rviz_common::Tool)
