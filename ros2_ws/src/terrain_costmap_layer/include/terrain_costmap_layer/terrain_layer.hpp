// Persistent graded terrain (slope) cost layer for nav2 costmap_2d.
//
// Consumes a per-cell terrain cost PointCloud2 (points at cell centers, a
// float "cost" field in 0..100) and writes it into its own rolling grid:
//   cost >= lethal_cost_threshold -> LETHAL_OBSTACLE
//   0 < cost < threshold          -> graded value in (0 .. MAX_NON_OBSTACLE]
//   cost <= 0 (flat/free)         -> depends on clearing_enabled (see below)
// The grid rolls with the window (same pattern as ObstacleLayer), so terrain
// cost persists as the robot moves.
//
// clearing_enabled (default true): obstacle-layer-like re-observation clearing.
//   Every cell present in the current cost cloud is OVERWRITTEN with its latest
//   value -- cells re-observed flat (cost<=0) are cleared to FREE, and graded
//   values are corrected up or down. Cells NOT in the cloud (unknown slope /
//   out of the elevation map) keep their previous value. When false, falls back
//   to the legacy accumulate-max behavior (never lowers; flat contributes
//   nothing) so terrain cost only ever grows until it scrolls out / is reset.

#ifndef TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_
#define TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
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
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr elevation_clear_client_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;

  std::string global_frame_;
  std::string cost_topic_;
  bool rolling_window_{false};
  int combination_method_{1};
  double lethal_cost_threshold_{100.0};
  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};

  // Obstacle-like clearing: overwrite each observed cell with its latest cost
  // (flat -> cleared to FREE), instead of accumulating the max forever.
  bool clearing_enabled_{true};

  // 脚印清除：每周期把机器人当前脚印内的格子在本层清成 FREE，
  // 保证 terrain 代价永远不会把机器人自己困住（对齐 ObstacleLayer）。
  bool footprint_clearing_enabled_{true};
  bool clear_elevation_on_reset_{true};
  std::string elevation_clear_service_;
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
};

}  // namespace terrain_costmap_layer

#endif  // TERRAIN_COSTMAP_LAYER__TERRAIN_LAYER_HPP_
