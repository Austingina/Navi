// Copyright (c) 2024, slope_behavior
// Licensed under the Apache License, Version 2.0

/**
 * @file bt_register_nodes.cpp
 * @brief 统一的 BT 节点注册文件
 * 
 * 手动定义 BT_RegisterNodesFromPlugin 函数，
 * 确保符号正确导出供 BehaviorTree.CPP 动态加载。
 */

#include "behaviortree_cpp/bt_factory.h"
#include "slope_behavior/is_slope_ahead_condition.hpp"
#include "slope_behavior/is_gravity_lowered_condition.hpp"
#include "slope_behavior/gravity_control_action.hpp"
#include "slope_behavior/check_blackboard_variable_condition.hpp"

// 使用 BT_REGISTER_NODES 宏来正确导出符号
BT_REGISTER_NODES(factory)
{
  // 注册条件节点
  factory.registerNodeType<slope_behavior::IsSlopeAheadCondition>("IsSlopeAhead");
  factory.registerNodeType<slope_behavior::IsGravityLoweredCondition>("IsGravityLowered");
  factory.registerNodeType<slope_behavior::CheckBlackboardVariable>("CheckBlackboardVariable");
  // 注册动作节点
  factory.registerNodeType<slope_behavior::LowerGravityAction>("LowerGravity");
  factory.registerNodeType<slope_behavior::RestoreGravityAction>("RestoreGravity");
}