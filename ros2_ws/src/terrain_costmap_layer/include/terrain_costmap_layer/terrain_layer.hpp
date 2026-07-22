// Persistent graded terrain (slope) cost layer for nav2 costmap_2d.
//
// Consumes a per-cell terrain cost PointCloud2 (points at cell centers, a
// float "cost" field in 0..100) and writes it into its own rolling grid:
//   cost >= lethal_cost_threshold -> LETHAL_OBSTACLE
//   0 < cost < threshold          -> graded value in (0 .. MAX_NON_OBSTACLE]
// The grid accumulates (max) and rolls with the window (same pattern as
// ObstacleLayer), so terrain cost persists as the robot moves.

#ifndef TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_
#define TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/time.h"

namespace terrain_costmap_layer
{

class TerrainLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TerrainLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;

  std::string global_frame_;
  std::string cost_topic_;
  bool rolling_window_{false};
  int combination_method_{1};
  double lethal_cost_threshold_{100.0};
  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};
};

}  // namespace terrain_costmap_layer

#endif  // TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_
