/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");
  gm::setVerbosityLevelToDebugIfFlagSet(node);

  rclcpp::QoS custom_qos = rclcpp::QoS(1).transient_local();
  auto gridMapPub = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map_from_raw_pointcloud", custom_qos);
  auto occupancyGridPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", custom_qos);

  grid_map::GridMapPclLoader gridMapPclLoader(node->get_logger());
  const std::string pathToCloud = gm::getPcdFilePath(node);
  RCLCPP_INFO(node->get_logger(), "Loading PCD file: %s", pathToCloud.c_str());

  gridMapPclLoader.loadParameters(gm::getParameterPath(node));
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, node);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(node));
  RCLCPP_INFO_STREAM(node->get_logger(), "Successfully added elevation layer");
  // Add slope layer based on elevation data
  const std::string elevationLayer = gm::kElevationLayer;
  const std::string slopeLayer = gm::kSlopeLayer;
  const std::string traversabilityLayer = gm::kTraversabilityLayer;
  
  try {
    gm::addSlopeLayer(gridMap, elevationLayer, slopeLayer);
    RCLCPP_INFO_STREAM(node->get_logger(), "Successfully added slope layer");
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to add slope layer: " << e.what());
  }
  
  // Add traversability layer based on obstacle point density
  try {
    const auto & params = gridMapPclLoader.getParameters();
    gm::addTraversabilityLayer(
      gridMap,
      gridMapPclLoader.getWorkingCloud(),
      traversabilityLayer,
      params.gridMap_.robot_height_,
      params.gridMap_.traversable_num_,
      params.gridMap_.ground_z_);
    RCLCPP_INFO_STREAM(node->get_logger(), "Successfully added traversability layer");
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to add traversability layer: " << e.what());
  }

  // Generate occupancy grid from traversability and slope layers
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  try {
    const auto & params = gridMapPclLoader.getParameters();
    gm::generateOccupancyGrid(
      gridMap,
      traversabilityLayer,
      slopeLayer,
      params.gridMap_.slope_threshold_,
      occupancyGrid);
    RCLCPP_INFO_STREAM(node->get_logger(), "Successfully generated occupancy grid");
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to generate occupancy grid: " << e.what());
  }

  // gm::saveGridMap(gridMap, node, gm::getMapRosbagTopic(node));

  // publish grid map
  auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
  msg->header.stamp = node->now();
  const auto & params = gridMapPclLoader.getParameters();
  gridMapPub->publish(*msg);
  RCLCPP_INFO_STREAM(node->get_logger(), "Published grid map to topic grid_map_from_raw_pointcloud");
  auto timer0 = node->create_wall_timer(
    std::chrono::duration<double>(params.gridMap_.interval_gridmap_),
    [&gridMapPub, &msg, &node]() {
      msg->header.stamp = node->now();
      gridMapPub->publish(*msg);
      RCLCPP_DEBUG(node->get_logger(), "Re-published grid map");
    });

  // publish occupancy grid to /map topic
  occupancyGridPub->publish(occupancyGrid);
  RCLCPP_INFO_STREAM(node->get_logger(), "Published occupancy grid to /map topic");
  
  // 使用timer定期重新发布，确保后启动的节点也能收到
  // transient_local QoS会保留最后一条消息，但定期发布更保险
  auto timer = node->create_wall_timer(
    std::chrono::duration<double>(params.gridMap_.interval_occupancy_gridmap_),
    [&occupancyGridPub, &occupancyGrid, &node]() {
      occupancyGrid.header.stamp = node->now();
      occupancyGridPub->publish(occupancyGrid);
      RCLCPP_DEBUG(node->get_logger(), "Re-published occupancy grid");
    });

  // run
  rclcpp::spin(node->get_node_base_interface());
  return EXIT_SUCCESS;
}
