// See observation_publishing_obstacle_layer.hpp for the design summary.

#include "terrain_costmap_layer/observation_publishing_obstacle_layer.hpp"

#include <string>
#include <vector>

#include "nav2_costmap_2d/observation.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace terrain_costmap_layer
{

namespace
{
constexpr const char * kObservationTopic = "/observation_pcd";
}  // namespace

void ObservationPublishingObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();

  declareParameter("pub_lidar_observation", rclcpp::ParameterValue(false));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"ObservationPublishingObstacleLayer: failed to lock node"};
  }
  node->get_parameter(name_ + "." + "pub_lidar_observation", pub_lidar_observation_);

  if (pub_lidar_observation_) {
    observation_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      kObservationTopic, rclcpp::SensorDataQoS());
    observation_pcd_pub_->on_activate();
    RCLCPP_INFO(
      logger_, "Observation point cloud publisher enabled on %s", kObservationTopic);
  }
}

void ObservationPublishingObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if (pub_lidar_observation_ && observation_pcd_pub_) {
    publishMarkingObservations();
  }
}

void ObservationPublishingObstacleLayer::publishMarkingObservations()
{
  std::vector<nav2_costmap_2d::Observation> observations;
  getMarkingObservations(observations);

  size_t total_points = 0;
  for (const auto & obs : observations) {
    total_points += static_cast<size_t>(obs.cloud_->width) * obs.cloud_->height;
  }
  if (total_points == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 obs_cloud;
  obs_cloud.header.frame_id = global_frame_;
  obs_cloud.header.stamp = clock_->now();
  obs_cloud.height = 1;
  obs_cloud.is_dense = true;
  obs_cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(obs_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(total_points);

  sensor_msgs::PointCloud2Iterator<float> out_x(obs_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(obs_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(obs_cloud, "z");

  for (const auto & obs : observations) {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++out_x, ++out_y, ++out_z) {
      *out_x = *it_x;
      *out_y = *it_y;
      *out_z = *it_z;
    }
  }

  observation_pcd_pub_->publish(obs_cloud);
}

}  // namespace terrain_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  terrain_costmap_layer::ObservationPublishingObstacleLayer, nav2_costmap_2d::Layer)
