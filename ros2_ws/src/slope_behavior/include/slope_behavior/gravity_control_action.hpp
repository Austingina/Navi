#ifndef SLOPE_BEHAVIOR__GRAVITY_CONTROL_ACTION_HPP_
#define SLOPE_BEHAVIOR__GRAVITY_CONTROL_ACTION_HPP_

#include <string>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

namespace slope_behavior
{

/**
 * @brief 重心控制基类，提供关节控制的通用功能
 */
class GravityControlActionBase : public BT::StatefulActionNode
{
public:
  GravityControlActionBase(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  virtual ~GravityControlActionBase() = default;

protected:
  /**
   * @brief 发送关节状态命令
   * @param position_left 左臂关节位置 (7个关节)
   * @param position_right 右臂关节位置 (7个关节)
   * @param position_torso 躯干关节位置 (4个关节)
   */
  void sendJointState(
    const std::vector<double> & position_left,
    const std::vector<double> & position_right,
    const std::vector<double> & position_torso);

  /**
   * @brief 发送速度限制
   */
  void sendVelLimit(
    const std::vector<double> & vel_limit_left,
    const std::vector<double> & vel_limit_right,
    const std::vector<double> & vel_limit_torso);

  // /**
  //  * @brief 发送夹爪状态
  //  */
  // void sendGripperState(double position_left, double position_right);

  // /**
  //  * @brief 发送制动模式
  //  */
  // void sendBreakingMode(bool enable);

  /**
   * @brief 停止底盘运动
   */
  void stopChassis();

  rclcpp::Node::SharedPtr node_;
  
  // 发布者
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_joint_state_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_gripper_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_gripper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr acc_limit_pub_;
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr breaking_mode_pub_;

  // QoS 配置
  rclcpp::QoS qos_profile_{10};
};

/**
 * @brief 降低重心动作节点
 * 在进入斜坡前调用，将机器人调整为低重心状态
 */
class LowerGravityAction : public GravityControlActionBase
{
public:
  LowerGravityAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 2.0, "Duration to complete action (s)"),
      BT::InputPort<double>("torso_pitch", 1.74, "Torso pitch angle for low gravity"),
      BT::InputPort<double>("torso_roll", -2.70, "Torso roll angle for low gravity"),
      BT::InputPort<double>("torso_yaw", -0.96, "Torso yaw angle for low gravity"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double duration_;
  rclcpp::Time start_time_;
  
  // 低重心状态的躯干关节角度
  double torso_pitch_;
  double torso_roll_;
  double torso_yaw_;
};

/**
 * @brief 恢复重心动作节点
 * 在离开斜坡后调用，将机器人调整为直立状态
 */
class RestoreGravityAction : public GravityControlActionBase
{
public:
  RestoreGravityAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 2.0, "Duration to complete action (s)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double duration_;
  rclcpp::Time start_time_;
};

}  // namespace slope_behavior

#endif  // SLOPE_BEHAVIOR__GRAVITY_CONTROL_ACTION_HPP_
