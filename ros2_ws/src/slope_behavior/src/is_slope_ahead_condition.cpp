#include "slope_behavior/is_slope_ahead_condition.hpp"
#include <cmath>

namespace slope_behavior
{

IsSlopeAheadCondition::IsSlopeAheadCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 获取参数
  getInput("slope_threshold", slope_threshold_);
  getInput("check_distance", check_distance_);
  getInput("slope_layer", slope_layer_);

  std::string grid_map_topic, path_topic;
  getInput("grid_map_topic", grid_map_topic);
  getInput("path_topic", path_topic);

  // 创建回调组
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // 创建订阅者
  grid_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&IsSlopeAheadCondition::gridMapCallback, this, std::placeholders::_1),
    sub_options);

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic, 10,
    std::bind(&IsSlopeAheadCondition::pathCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), 
    "IsSlopeAhead condition initialized (threshold: %.1f deg, check_distance: %.1f m)",
    slope_threshold_, check_distance_);
}

IsSlopeAheadCondition::~IsSlopeAheadCondition()
{
  grid_map_sub_.reset();
  path_sub_.reset();
}

void IsSlopeAheadCondition::gridMapCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  grid_map::GridMapRosConverter::fromMessage(*msg, grid_map_);
  has_grid_map_ = true;
}

void IsSlopeAheadCondition::pathCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_path_ = *msg;
  has_path_ = !msg->poses.empty();
}

// ...existing code...

bool IsSlopeAheadCondition::checkSlopeOnPath()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!grid_map_.exists(slope_layer_)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Slope layer '%s' not found in grid_map", slope_layer_.c_str());
    return false;
  }

  if (current_path_.poses.size() < 2) {
    return false;
  }
  // RCLCPP_INFO(node_->get_logger(), "Checking slope on path with %zu poses", current_path_.poses.size());
  double distance_traveled = 0.0;
  double max_slope = 0.0;

  for (size_t i = 1; i < current_path_.poses.size() && distance_traveled < check_distance_; ++i) {
    const auto & p1 = current_path_.poses[i - 1].pose.position;
    const auto & p2 = current_path_.poses[i].pose.position;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    distance_traveled += std::sqrt(dx * dx + dy * dy);

    grid_map::Position position(p2.x, p2.y);
    if (grid_map_.isInside(position)) {
      try {
        float slope_rad = grid_map_.atPosition(slope_layer_, position);
        if (!std::isnan(slope_rad)) {
          // 弧度转换为度
          double slope_deg = std::abs(slope_rad) * 180.0 / M_PI;
          max_slope = std::max(max_slope, slope_deg);
          // RCLCPP_INFO(node_->get_logger(), "Slope at  index %zu(%.2f, %.2f): %.2f degrees, distance traveled: %.2f m", i,
          //   p2.x, p2.y, slope_deg, distance_traveled);
        }
      } catch (const std::out_of_range &) {
        continue;
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),   "Max slope on path: %.2f degrees (threshold: %.2f)",  max_slope, slope_threshold_);

  return max_slope > slope_threshold_;
}

// ...existing code...

BT::NodeStatus IsSlopeAheadCondition::tick()
{
  // RCLCPP_INFO(node_->get_logger(), 
  //   "=== IsSlopeAhead tick() called, has_grid_map=%d, has_path=%d ===",
  //   has_grid_map_, has_path_);
  // 处理回调
  callback_group_executor_.spin_some();

  if (!has_grid_map_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Waiting for grid_map data");
    return BT::NodeStatus::FAILURE;
  }

  if (!has_path_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Waiting for path data");
    return BT::NodeStatus::FAILURE;
  }

  if (checkSlopeOnPath()) {
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    //   "Steep slope detected ahead!");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace slope_behavior
