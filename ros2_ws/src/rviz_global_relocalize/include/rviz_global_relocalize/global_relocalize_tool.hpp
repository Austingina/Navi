#pragma once

#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rviz_global_relocalize
{
class GlobalRelocalizeTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  GlobalRelocalizeTool();
  ~GlobalRelocalizeTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

} // namespace rviz_global_relocalize
