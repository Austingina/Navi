#pragma once
#include "behaviortree_cpp_v3/condition_node.h"
#include <any>

namespace slope_behavior
{

class CheckBlackboardVariable : public BT::ConditionNode
{
public:
  CheckBlackboardVariable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("key"), BT::InputPort<std::string>("value") };
  }

  BT::NodeStatus tick() override
  {
      std::string key, value;
      if (!getInput("key", key) || !getInput("value", value)) {
          std::cout << "CheckBlackboardVariable: missing input port" << std::endl;
          return BT::NodeStatus::FAILURE;
      }

      auto bb = config().blackboard;

      std::string stored_str;
      bool stored_bool;

      if (bb->get<std::string>(key, stored_str)) {
          // std::cout << "CheckBlackboardVariable: found string value for key: " << key << " - value: " << stored_str << std::endl;
          return (stored_str == value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      if (bb->get<bool>(key, stored_bool)) {
          bool match = (stored_bool ? "true" : "false") == value;
          // std::cout << "CheckBlackboardVariable: found bool value for key: " << key << " - value: " << (stored_bool ? "true" : "false") << " - match: " << (match ? "true" : "false") << std::endl;
          return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }

      // not found / unsupported
      std::cout << "CheckBlackboardVariable: unsupported or missing type for key: " << key << std::endl;
      return BT::NodeStatus::FAILURE;
  }

};

} // namespace slope_behavior