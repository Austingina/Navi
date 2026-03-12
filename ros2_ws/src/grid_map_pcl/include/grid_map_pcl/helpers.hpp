/*
 * helpers.hpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef GRID_MAP_PCL__HELPERS_HPP_
#define GRID_MAP_PCL__HELPERS_HPP_

#include <pcl/common/common.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <chrono>

namespace grid_map
{

class GridMapPclLoader;
class GridMap;

namespace grid_map_pcl
{

// Layer name constants
constexpr const char* kElevationLayer = "elevation";
constexpr const char* kSlopeLayer = "slope";
constexpr const char* kTraversabilityLayer = "traversability";  

void setVerbosityLevelToDebugIfFlagSet(const rclcpp::Node::SharedPtr & node);

std::string getParameterPath(const rclcpp::Node::SharedPtr & node);

std::string getOutputBagPath(const rclcpp::Node::SharedPtr & node);

std::string getPcdFilePath(const rclcpp::Node::SharedPtr & node);

std::string getMapFrame(const rclcpp::Node::SharedPtr & node);

std::string getMapRosbagTopic(const rclcpp::Node::SharedPtr & node);


void saveGridMap(
  const grid_map::GridMap & gridMap, const rclcpp::Node::SharedPtr & node,
  const std::string & mapTopic);

inline void printTimeElapsedToRosInfoStream(
  const std::chrono::system_clock::time_point & start,
  const std::string & prefix,
  const rclcpp::Logger & node_logger)
{
  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
  RCLCPP_INFO_STREAM(node_logger, prefix << duration << " sec");
}

void processPointcloud(
  grid_map::GridMapPclLoader * gridMapPclLoader,
  const rclcpp::Node::SharedPtr & node);

using Point = ::pcl::PointXYZ;
using Pointcloud = ::pcl::PointCloud<Point>;
enum class XYZ : int {X, Y, Z};

Eigen::Affine3f getRigidBodyTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & intrinsicRpy,
  const rclcpp::Logger & node_logger);

Eigen::Matrix3f getRotationMatrix(
  double angle, XYZ axis,
  const rclcpp::Logger & node_logger);

// processing point clouds
Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud);
Pointcloud::Ptr transformCloud(
  Pointcloud::ConstPtr inputCloud,
  const Eigen::Affine3f & transformMatrix);
Pointcloud::Ptr loadPointcloudFromPcd(const std::string & filename);

// Add slope layer to grid map based on elevation data
void addSlopeLayer(
  grid_map::GridMap & gridMap,
  const std::string & elevationLayer,
  const std::string & slopeLayer);

// Add traversability layer based on obstacle point density within robot height
void addTraversabilityLayer(
  grid_map::GridMap & gridMap,
  Pointcloud::ConstPtr cloud,
  const std::string & traversabilityLayer,
  double robotHeight,
  unsigned int traversableNum,
  double groundZ);

// Generate occupancy grid from traversability and slope layers
void generateOccupancyGrid(
  const grid_map::GridMap & gridMap,
  const std::string & traversabilityLayer,
  const std::string & slopeLayer,
  double slopeThreshold,
  nav_msgs::msg::OccupancyGrid & occupancyGrid);

}  // namespace grid_map_pcl

}  // namespace grid_map
#endif  // GRID_MAP_PCL__HELPERS_HPP_
