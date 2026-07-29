// nav2 ObstacleLayer that can additionally republish the marking observations
// it consumes, as a single merged cloud on /observation_pcd (global frame).
//
// This exists so the stack can use the upstream (binary) nav2_costmap_2d
// instead of vendoring a patched copy of it: the only local change that was
// ever made to ObstacleLayer was this debug/visualization publisher.
//
// Points are published exactly as they come out of the observation buffers,
// i.e. BEFORE the min/max height and range filtering that updateBounds applies
// when marking cells. That is intentional -- the topic is meant to show what
// the layer received, not what it marked.

#ifndef TERRAIN_COSTMAP_LAYER__OBSERVATION_PUBLISHING_OBSTACLE_LAYER_HPP_
#define TERRAIN_COSTMAP_LAYER__OBSERVATION_PUBLISHING_OBSTACLE_LAYER_HPP_

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace terrain_costmap_layer
{

class ObservationPublishingObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  ObservationPublishingObstacleLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

private:
  void publishMarkingObservations();

  bool pub_lidar_observation_{false};
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    observation_pcd_pub_;
};

}  // namespace terrain_costmap_layer

#endif  // TERRAIN_COSTMAP_LAYER__OBSERVATION_PUBLISHING_OBSTACLE_LAYER_HPP_
