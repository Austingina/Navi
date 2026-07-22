// Persistent graded terrain (slope) cost layer for nav2 costmap_2d.
// See terrain_layer.hpp for the design summary.

#include "terrain_costmap_layer/terrain_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MAX_NON_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace terrain_costmap_layer
{

void TerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"TerrainLayer: failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cost_topic", rclcpp::ParameterValue(std::string("/terrain_cost_cloud")));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("lethal_cost_threshold", rclcpp::ParameterValue(100.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "cost_topic", cost_topic_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "lethal_cost_threshold", lethal_cost_threshold_);
  double tf_tol = 0.1;
  node->get_parameter(name_ + "." + "transform_tolerance", tf_tol);
  transform_tolerance_ = tf2::durationFromSec(tf_tol);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  // Our own grid stores NO_INFORMATION where we have no terrain data, so
  // updateWithMax never lowers other layers over unobserved cells.
  setDefaultValue(NO_INFORMATION);
  TerrainLayer::matchSize();
  current_ = false;

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cost_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TerrainLayer::cloudCallback, this, std::placeholders::_1), sub_opt);

  RCLCPP_INFO(
    logger_,
    "TerrainLayer initialized. topic=%s global_frame=%s rolling=%d lethal>=%.0f",
    cost_topic_.c_str(), global_frame_.c_str(), static_cast<int>(rolling_window_),
    lethal_cost_threshold_);
}

void TerrainLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void TerrainLayer::reset()
{
  resetMaps();
  current_ = false;
}

void TerrainLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cloud_mutex_);
  latest_cloud_ = msg;
}

void TerrainLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  // Roll our own grid with the window (shifts stored data, drops the edge).
  // Same pattern as ObstacleLayer::updateBounds.
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    cloud = latest_cloud_;
  }
  if (!cloud || cloud->width * cloud->height == 0) {
    return;
  }

  // Transform the terrain cloud into the layer's global frame. Using the
  // latest available transform (TimePointZero) is robust for the slowly
  // varying map->odom on this platform and avoids extrapolation errors.
  sensor_msgs::msg::PointCloud2 cloud_g;
  if (cloud->header.frame_id == global_frame_) {
    cloud_g = *cloud;
  } else {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_->lookupTransform(global_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "TerrainLayer TF %s<-%s failed: %s", global_frame_.c_str(),
        cloud->header.frame_id.c_str(), ex.what());
      return;
    }
    tf2::doTransform(*cloud, cloud_g, tf_msg);
  }

  // The cloud must carry x, y and a float "cost" (0..100) field.
  bool has_cost = false;
  for (const auto & f : cloud_g.fields) {
    if (f.name == "cost") {has_cost = true; break;}
  }
  if (!has_cost) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "TerrainLayer: cloud on %s has no 'cost' field",
      cost_topic_.c_str());
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud_g, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(cloud_g, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ic(cloud_g, "cost");

  const double lethal_thr = lethal_cost_threshold_;
  for (; ix != ix.end(); ++ix, ++iy, ++ic) {
    const double px = *ix;
    const double py = *iy;
    const float c = *ic;
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(c)) {
      continue;
    }
    if (c <= 0.0f) {
      continue;  // flat/free: contribute nothing (never lower other layers)
    }

    unsigned char value;
    if (c >= lethal_thr) {
      value = LETHAL_OBSTACLE;
    } else {
      double scaled = c / 100.0 * static_cast<double>(MAX_NON_OBSTACLE);
      scaled = std::clamp(scaled, 1.0, static_cast<double>(MAX_NON_OBSTACLE));
      value = static_cast<unsigned char>(std::lround(scaled));
    }

    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my)) {
      continue;
    }
    const unsigned int index = getIndex(mx, my);
    // Accumulate: keep the higher of stored and new (NO_INFORMATION counts as lowest).
    if (costmap_[index] == NO_INFORMATION || value > costmap_[index]) {
      costmap_[index] = value;
    }
    touch(px, py, min_x, min_y, max_x, max_y);
  }

  current_ = true;
}

void TerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (combination_method_ == 0) {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
}

}  // namespace terrain_costmap_layer

PLUGINLIB_EXPORT_CLASS(terrain_costmap_layer::TerrainLayer, nav2_costmap_2d::Layer)
