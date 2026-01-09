// Copyright (c) 2024, slope_behavior
// Licensed under the Apache License, Version 2.0

#ifndef SLOPE_BEHAVIOR__IS_GRAVITY_LOWERED_CONDITION_HPP_
#define SLOPE_BEHAVIOR__IS_GRAVITY_LOWERED_CONDITION_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace slope_behavior
{

class IsGravityLoweredCondition : public BT::ConditionNode
{
public:
  IsGravityLoweredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    RCLCPP_INFO(rclcpp::get_logger("slope_behavior"), 
      "=== IsGravityLoweredCondition CREATED ===");

    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    // 创建订阅
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    torso_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/hdas/feedback_torso",
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&IsGravityLoweredCondition::torsoCallback, this, std::placeholders::_1),
      sub_options);

     RCLCPP_INFO(node_->get_logger(), 
      "IsGravityLoweredCondition initialized and subscribed to /hdas/feedback_torso");
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool is_upright = true;
    if (last_torso_msg_) {
      // 判断 position 是否全为零
      for (double v : last_torso_msg_->position) {
        if (std::abs(v) > 1e-1) {  // 容差1e-1
          is_upright = false;
          break;
        }
      }
    } else {
      // RCLCPP_WARN(node_->get_logger(), 
      //   "No torso feedback received yet, assuming upright");
    }

    // RCLCPP_INFO(node_->get_logger(), 
    //   "IsGravityLowered: %s", is_upright ? "false (upright)" : "true (lowered)");

    return is_upright ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

private:
  void torsoCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_torso_msg_ = msg;
    //     // 打印 position 的值
    // std::ostringstream oss;
    // oss << "Received torso position: [";
    // for (size_t i = 0; i < msg->position.size(); ++i) {
    //   oss << msg->position[i];
    //   if (i + 1 < msg->position.size()) oss << ", ";
    // }
    // oss << "]";
    // RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torso_sub_;
  sensor_msgs::msg::JointState::SharedPtr last_torso_msg_;
  std::mutex data_mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace slope_behavior

#endif  // SLOPE_BEHAVIOR__IS_GRAVITY_LOWERED_CONDITION_HPP_
