#include "slope_behavior/gravity_control_action.hpp"

namespace slope_behavior
{

// ============================================================================
// GravityControlActionBase 基类实现
// ============================================================================

GravityControlActionBase::GravityControlActionBase(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 配置 QoS
  qos_profile_ = rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);

  // 创建发布者
  left_joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_left", qos_profile_);
  
  right_joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_right", qos_profile_);
  
  torso_joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_torso", qos_profile_);
  
  // left_gripper_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
  //   "/motion_target/target_position_gripper_left", qos_profile_);
  
  // right_gripper_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
  //   "/motion_target/target_position_gripper_right", qos_profile_);
  
  chassis_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/motion_target/target_speed_chassis", qos_profile_);
  
  acc_limit_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/motion_target/chassis_acc_limit", qos_profile_);
  
  // breaking_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
  //   "/motion_target/brake_mode", qos_profile_);
}

void GravityControlActionBase::sendJointState(
  const std::vector<double> & position_left,
  const std::vector<double> & position_right,
  const std::vector<double> & position_torso)
{
  sensor_msgs::msg::JointState left_msg;
  left_msg.header.stamp = node_->now();
  left_msg.position = position_left;
  left_joint_state_pub_->publish(left_msg);

  sensor_msgs::msg::JointState right_msg;
  right_msg.header.stamp = node_->now();
  right_msg.position = position_right;
  right_joint_state_pub_->publish(right_msg);

  sensor_msgs::msg::JointState torso_msg;
  torso_msg.header.stamp = node_->now();
  torso_msg.position = position_torso;
  torso_joint_state_pub_->publish(torso_msg);
}

void GravityControlActionBase::sendVelLimit(
  const std::vector<double> & vel_limit_left,
  const std::vector<double> & vel_limit_right,
  const std::vector<double> & vel_limit_torso)
{
  sensor_msgs::msg::JointState left_msg;
  left_msg.header.stamp = node_->now();
  left_msg.velocity = vel_limit_left;
  left_joint_state_pub_->publish(left_msg);

  sensor_msgs::msg::JointState right_msg;
  right_msg.header.stamp = node_->now();
  right_msg.velocity = vel_limit_right;
  right_joint_state_pub_->publish(right_msg);

  sensor_msgs::msg::JointState torso_msg;
  torso_msg.header.stamp = node_->now();
  torso_msg.velocity = vel_limit_torso;
  torso_joint_state_pub_->publish(torso_msg);
}

// void GravityControlActionBase::sendGripperState(double position_left, double position_right)
// {
//   sensor_msgs::msg::JointState left_msg;
//   left_msg.header.stamp = node_->now();
//   left_msg.position = {position_left};
//   left_gripper_pub_->publish(left_msg);

//   sensor_msgs::msg::JointState right_msg;
//   right_msg.header.stamp = node_->now();
//   right_msg.position = {position_right};
//   right_gripper_pub_->publish(right_msg);
// }

// void GravityControlActionBase::sendBreakingMode(bool enable)
// {
//   std_msgs::msg::Bool msg;
//   msg.data = enable;
//   breaking_mode_pub_->publish(msg);
// }

void GravityControlActionBase::stopChassis()
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = node_->now();
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.angular.z = 0.0;
  chassis_cmd_pub_->publish(msg);
}

// ============================================================================
// LowerGravityAction 实现
// ============================================================================

LowerGravityAction::LowerGravityAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: GravityControlActionBase(action_name, conf)
{
  RCLCPP_INFO(node_->get_logger(), "LowerGravityAction node initialized");
}

BT::NodeStatus LowerGravityAction::onStart()
{
  // 获取参数
  getInput("duration", duration_);
  getInput("torso_pitch", torso_pitch_);
  getInput("torso_roll", torso_roll_);
  getInput("torso_yaw", torso_yaw_);

  // RCLCPP_INFO(node_->get_logger(), 
  //   "Starting to lower gravity center (duration: %.2f s, torso: [%.2f, %.2f, %.2f, 0.0])",
  //   duration_, torso_pitch_, torso_roll_, torso_yaw_);

  // 1. 停止底盘
  stopChassis();

  // 2. 启用制动模式
  // sendBreakingMode(true);

  // 3. 设置速度限制（躯干速度放慢以确保安全）
  sendVelLimit(
    {1.6, 1.6, 1.6, 1.6, 4.0, 4.0, 4.0},  // 左臂
    {1.6, 1.6, 1.6, 1.6, 4.0, 4.0, 4.0},  // 右臂
    {0.8, 0.8, 0.7, 0.7}                   // 躯干（较慢）
  );

  // 4. 发送低重心姿态命令
  // 双手抬起90度，躯干弯曲
  sendJointState(
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // 左臂：抬起90度
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // 右臂：抬起90度
    {torso_pitch_, torso_roll_, torso_yaw_, 0.0}     // 躯干弯曲
  );

  // 5. 夹爪保持抓握状态
  // sendGripperState(100.0, 100.0);

  start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LowerGravityAction::onRunning()
{
  // 持续停止底盘
  stopChassis();

  auto elapsed = (node_->now() - start_time_).seconds();

  // 定期发送关节状态命令以确保到达目标位置
  if (static_cast<int>(elapsed * 5) % 5 == 0) {
    sendJointState(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {torso_pitch_, torso_roll_, torso_yaw_, 0.0}
    );
  }

  if (elapsed >= duration_) {
    // RCLCPP_INFO(node_->get_logger(), "Gravity center lowered successfully");
    return BT::NodeStatus::SUCCESS;
  }

  // RCLCPP_DEBUG(node_->get_logger(), "Lowering gravity... %.1f / %.1f s", elapsed, duration_);
  return BT::NodeStatus::RUNNING;
}

void LowerGravityAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "LowerGravityAction halted");
  // 被中断时保持当前状态，不做额外操作
}

// ============================================================================
// RestoreGravityAction 实现
// ============================================================================

RestoreGravityAction::RestoreGravityAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: GravityControlActionBase(action_name, conf)
{
  RCLCPP_INFO(node_->get_logger(), "RestoreGravityAction node initialized");
}

BT::NodeStatus RestoreGravityAction::onStart()
{
  getInput("duration", duration_);

  // RCLCPP_INFO(node_->get_logger(), 
  //   "Starting to restore gravity center to upright position (duration: %.2f s)",
  //   duration_);

  // 1. 停止底盘
  stopChassis();

  // 2. 启用制动模式
  // sendBreakingMode(true);

  // 3. 设置速度限制
  sendVelLimit(
    {1.6, 1.6, 1.6, 1.6, 4.0, 4.0, 4.0},
    {1.6, 1.6, 1.6, 1.6, 4.0, 4.0, 4.0},
    {0.8, 0.8, 0.7, 0.7}
  );

  // 4. 发送直立姿态命令
  // 双手下垂，躯干直立
  sendJointState(
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // 左臂：下垂
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // 右臂：下垂
    {0.0, 0.0, 0.0, 0.0}                   // 躯干：直立
  );

  // // 5. 夹爪保持抓握状态
  // sendGripperState(100.0, 100.0);

  start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RestoreGravityAction::onRunning()
{
  // 持续停止底盘
  stopChassis();

  auto elapsed = (node_->now() - start_time_).seconds();

  // 定期发送关节状态命令
  if (static_cast<int>(elapsed * 5) % 5 == 0) {
    sendJointState(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0}
    );
  }

  if (elapsed >= duration_) {
    // RCLCPP_INFO(node_->get_logger(), "Gravity center restored to upright position");
    return BT::NodeStatus::SUCCESS;
  }

  // RCLCPP_INFO(node_->get_logger(), "Restoring gravity... %.1f / %.1f s", elapsed, duration_);
  return BT::NodeStatus::RUNNING;
}

void RestoreGravityAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "RestoreGravityAction halted");
}

}  // namespace slope_behavior
