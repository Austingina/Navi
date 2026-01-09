#ifndef SLOPE_BEHAVIOR__IS_SLOPE_AHEAD_CONDITION_HPP_
#define SLOPE_BEHAVIOR__IS_SLOPE_AHEAD_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

namespace slope_behavior
{

/**
 * @brief 检测前方路径是否存在陡坡的条件节点
 * 
 * 该节点订阅 grid_map 话题，检查 slope 层的数据，
 * 判断局部路径前方是否存在坡度超过阈值的区域。
 */
class IsSlopeAheadCondition : public BT::ConditionNode
{
public:
  IsSlopeAheadCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~IsSlopeAheadCondition() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("slope_threshold", 15.0, "Slope threshold in degrees"),
      BT::InputPort<double>("check_distance", 2.0, "Distance to check ahead on path (m)"),
      BT::InputPort<std::string>("grid_map_topic", "/grid_map", "Grid map topic"),
      BT::InputPort<std::string>("path_topic", "/local_plan", "Local path topic"),
      BT::InputPort<std::string>("slope_layer", "slope", "Slope layer name in grid_map"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  bool checkSlopeOnPath();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  grid_map::GridMap grid_map_;
  nav_msgs::msg::Path current_path_;
  std::mutex data_mutex_;

  double slope_threshold_;
  double check_distance_;
  std::string slope_layer_;

  bool has_grid_map_{false};
  bool has_path_{false};
};

}  // namespace slope_behavior

#endif  // SLOPE_BEHAVIOR__IS_SLOPE_AHEAD_CONDITION_HPP_
