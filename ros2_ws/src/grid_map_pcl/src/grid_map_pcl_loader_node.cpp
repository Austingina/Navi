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
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

class GridMapPclLoaderNode : public rclcpp::Node
{
public:
  GridMapPclLoaderNode()
  : Node("grid_map_pcl_loader_node"),
    gridMapPclLoader_(this->get_logger())
  {
  }

  void init()
  {
    gm::setVerbosityLevelToDebugIfFlagSet(this->shared_from_this());

    rclcpp::QoS custom_qos = rclcpp::QoS(1).transient_local();
    gridMapPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "grid_map_from_raw_pointcloud", custom_qos);
    occupancyGridPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", custom_qos);

    // Subscribe to change_map topic
    changeMapSub_ = this->create_subscription<std_msgs::msg::String>(
      "change_map", 10,
      std::bind(&GridMapPclLoaderNode::changeMapCallback, this, std::placeholders::_1));

    // Load initial map
    const std::string pathToCloud = gm::getPcdFilePath(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "Loading initial PCD file: %s", pathToCloud.c_str());
    loadAndPublishMap(pathToCloud);

    RCLCPP_INFO(this->get_logger(), "GridMapPclLoaderNode initialized. Listening on change_map topic.");
  }

private:
  void changeMapCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string newMapPath = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received change_map request: %s", newMapPath.c_str());
    
    // Stop existing timers
    if (gridMapTimer_) {
      gridMapTimer_->cancel();
    }
    if (occupancyGridTimer_) {
      occupancyGridTimer_->cancel();
    }

    loadAndPublishMap(newMapPath);
  }

  void loadAndPublishMap(const std::string & pathToCloud)
  {
    // Load parameters and point cloud
    gridMapPclLoader_.loadParameters(gm::getParameterPath(this->shared_from_this()));
    gridMapPclLoader_.loadCloudFromPcdFile(pathToCloud);

    gm::processPointcloud(&gridMapPclLoader_, this->shared_from_this());

    gridMap_ = gridMapPclLoader_.getGridMap();
    gridMap_.setFrameId(gm::getMapFrame(this->shared_from_this()));
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully added elevation layer");

    // Add slope layer based on elevation data
    const std::string elevationLayer = gm::kElevationLayer;
    const std::string slopeLayer = gm::kSlopeLayer;
    const std::string traversabilityLayer = gm::kTraversabilityLayer;

    try {
      gm::addSlopeLayer(gridMap_, elevationLayer, slopeLayer);
      RCLCPP_INFO_STREAM(this->get_logger(), "Successfully added slope layer");
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add slope layer: " << e.what());
    }

    // Add traversability layer based on obstacle point density
    try {
      const auto & params = gridMapPclLoader_.getParameters();
      gm::addTraversabilityLayer(
        gridMap_,
        gridMapPclLoader_.getWorkingCloud(),
        traversabilityLayer,
        params.gridMap_.robot_height_,
        params.gridMap_.traversable_num_,
        params.gridMap_.ground_z_);
      RCLCPP_INFO_STREAM(this->get_logger(), "Successfully added traversability layer");
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add traversability layer: " << e.what());
    }

    // Generate occupancy grid from traversability and slope layers
    try {
      const auto & params = gridMapPclLoader_.getParameters();
      gm::generateOccupancyGrid(
        gridMap_,
        traversabilityLayer,
        slopeLayer,
        params.gridMap_.slope_threshold_,
        occupancyGrid_);
      RCLCPP_INFO_STREAM(this->get_logger(), "Successfully generated occupancy grid");
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to generate occupancy grid: " << e.what());
    }

    // Publish grid map
    gridMapMsg_ = grid_map::GridMapRosConverter::toMessage(gridMap_);
    gridMapMsg_->header.stamp = this->now();
    gridMapPub_->publish(*gridMapMsg_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Published grid map to topic grid_map_from_raw_pointcloud");

    // Publish occupancy grid
    occupancyGrid_.header.stamp = this->now();
    occupancyGridPub_->publish(occupancyGrid_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Published occupancy grid to /map topic");

    // Create timers for periodic re-publishing
    const auto & params = gridMapPclLoader_.getParameters();
    
    gridMapTimer_ = this->create_wall_timer(
      std::chrono::duration<double>(params.gridMap_.interval_gridmap_),
      [this]() {
        gridMapMsg_->header.stamp = this->now();
        gridMapPub_->publish(*gridMapMsg_);
        RCLCPP_DEBUG(this->get_logger(), "Re-published grid map");
      });

    occupancyGridTimer_ = this->create_wall_timer(
      std::chrono::duration<double>(params.gridMap_.interval_occupancy_gridmap_),
      [this]() {
        occupancyGrid_.header.stamp = this->now();
        occupancyGridPub_->publish(occupancyGrid_);
        RCLCPP_DEBUG(this->get_logger(), "Re-published occupancy grid");
      });

    // 同步更新 ROS2 参数服务器
    this->set_parameter(rclcpp::Parameter("map_file_path", pathToCloud));

    RCLCPP_INFO(this->get_logger(), "Map processing completed for: %s", pathToCloud.c_str());
  }

  // Publishers
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPub_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr changeMapSub_;

  // Timers
  rclcpp::TimerBase::SharedPtr gridMapTimer_;
  rclcpp::TimerBase::SharedPtr occupancyGridTimer_;

  // Data
  grid_map::GridMapPclLoader gridMapPclLoader_;
  grid_map::GridMap gridMap_;
  std::unique_ptr<grid_map_msgs::msg::GridMap> gridMapMsg_;
  nav_msgs::msg::OccupancyGrid occupancyGrid_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapPclLoaderNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
