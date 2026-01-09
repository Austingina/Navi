/*
 * helpers.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "grid_map_pcl/helpers.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"

namespace grid_map
{
namespace grid_map_pcl
{


void setVerbosityLevelToDebugIfFlagSet(rclcpp::Node::SharedPtr & node)
{
  bool isSetVerbosityLevelToDebug = node->declare_parameter("set_verbosity_to_debug", false);

  if (!isSetVerbosityLevelToDebug) {
    return;
  }

  auto ret = rcutils_logging_set_logger_level(
    node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to change logging severity: %s",
      rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

std::string getParameterPath(rclcpp::Node::SharedPtr & node)
{
  // Default path: package share directory + config/parameters.yaml
  std::string defaultPath = ament_index_cpp::get_package_share_directory("grid_map_pcl") +
    "/config/parameters.yaml";
  node->declare_parameter("config_file_path", defaultPath);
  
  std::string filePath;
  node->get_parameter("config_file_path", filePath);
  return filePath;
}

std::string getOutputBagPath(rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("folder_path")) {
    node->declare_parameter("folder_path", std::string(""));
  }
  node->declare_parameter("output_grid_map", "output_grid_map.bag");

  std::string outputRosbagName, folderPath;
  node->get_parameter("folder_path", folderPath);
  node->get_parameter("output_grid_map", outputRosbagName);
  std::string pathToOutputBag = folderPath + "/" + outputRosbagName;
  return pathToOutputBag;
}

std::string getPcdFilePath(rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("folder_path")) {
    node->declare_parameter("folder_path", std::string(""));
  }
  node->declare_parameter("pcd_filename", "input_cloud");

  std::string inputCloudName, folderPath;
  node->get_parameter("folder_path", folderPath);
  node->get_parameter("pcd_filename", inputCloudName);
  std::string pathToCloud = folderPath + "/" + inputCloudName;
  return pathToCloud;
}

std::string getMapFrame(rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("map_frame", std::string("map"));

  std::string mapFrame;
  node->get_parameter("map_frame", mapFrame);
  return mapFrame;
}

std::string getMapRosbagTopic(rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("map_rosbag_topic", std::string("grid_map"));

  std::string mapRosbagTopic;
  node->get_parameter("map_rosbag_topic", mapRosbagTopic);
  return mapRosbagTopic;
}

void saveGridMap(
  const grid_map::GridMap & gridMap, rclcpp::Node::SharedPtr & node,
  const std::string & mapTopic)
{
  std::string pathToOutputBag = getOutputBagPath(node);
  const bool savingSuccessful = grid_map::GridMapRosConverter::saveToBag(
    gridMap, pathToOutputBag,
    mapTopic);
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Saving grid map successful: " << std::boolalpha << savingSuccessful);
}

void processPointcloud(
  grid_map::GridMapPclLoader * gridMapPclLoader,
  rclcpp::Node::SharedPtr & node)
{
  const auto start = std::chrono::high_resolution_clock::now();
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  printTimeElapsedToRosInfoStream(
    start, "Initialization took: ", node->get_logger());
  gridMapPclLoader->addLayerFromInputCloud(kElevationLayer);
  printTimeElapsedToRosInfoStream(start, "Total time: ", node->get_logger());
}

Eigen::Affine3f getRigidBodyTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & intrinsicRpy,
  const rclcpp::Logger & node_logger)
{
  Eigen::Affine3f rigidBodyTransform;
  rigidBodyTransform.setIdentity();
  rigidBodyTransform.translation() << translation.x(), translation.y(), translation.z();
  Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
  rotation *= getRotationMatrix(intrinsicRpy.x(), XYZ::X, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.y(), XYZ::Y, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.z(), XYZ::Z, node_logger);
  rigidBodyTransform.rotate(rotation);

  return rigidBodyTransform;
}

Eigen::Matrix3f getRotationMatrix(
  double angle, XYZ axis,
  const rclcpp::Logger & node_logger)
{
  Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
  switch (axis) {
    case XYZ::X: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX());
        break;
      }
    case XYZ::Y: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
        break;
      }
    case XYZ::Z: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
        break;
      }
    default:
      RCLCPP_ERROR(node_logger, "Unknown axis while trying to rotate the pointcloud");
  }
  return rotationMatrix;
}

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();

  return mean;
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string & filename)
{
  Pointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

Pointcloud::Ptr transformCloud(
  Pointcloud::ConstPtr inputCloud,
  const Eigen::Affine3f & transformMatrix)
{
  Pointcloud::Ptr transformedCloud(new Pointcloud());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}

void addSlopeLayer(
  grid_map::GridMap & gridMap,
  const std::string & elevationLayer,
  const std::string & slopeLayer)
{
  if (!gridMap.exists(elevationLayer)) {
    throw std::runtime_error("Elevation layer does not exist in grid map");
  }

  // Add slope layer if not exists, otherwise update
  if (!gridMap.exists(slopeLayer)) {
    gridMap.add(slopeLayer);
  }
  const grid_map::Matrix & elevationData = gridMap.get(elevationLayer);
  grid_map::Matrix & slopeData = gridMap.get(slopeLayer);
  const double resolution = gridMap.getResolution();

  // Calculate slope using central difference method
  for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const int i = index(0);
    const int j = index(1);

    // Check if current cell has valid elevation
    if (!std::isfinite(elevationData(i, j))) {
      slopeData(i, j) = std::nan("1");
      continue;
    }

    // Calculate gradients using central difference where possible
    float dz_dx = 0.0;
    float dz_dy = 0.0;
    bool valid_gradient = false;

    // Gradient in x direction
    if (i > 0 && i < gridMap.getSize()(0) - 1) {
      if (std::isfinite(elevationData(i + 1, j)) && std::isfinite(elevationData(i - 1, j))) {
        dz_dx = (elevationData(i + 1, j) - elevationData(i - 1, j)) / (2.0 * resolution);
        valid_gradient = true;
      }
    } else if (i == 0 && gridMap.getSize()(0) > 1) {
      if (std::isfinite(elevationData(i + 1, j))) {
        dz_dx = (elevationData(i + 1, j) - elevationData(i, j)) / resolution;
        valid_gradient = true;
      }
    } else if (i == gridMap.getSize()(0) - 1 && gridMap.getSize()(0) > 1) {
      if (std::isfinite(elevationData(i - 1, j))) {
        dz_dx = (elevationData(i, j) - elevationData(i - 1, j)) / resolution;
        valid_gradient = true;
      }
    }

    // Gradient in y direction
    if (j > 0 && j < gridMap.getSize()(1) - 1) {
      if (std::isfinite(elevationData(i, j + 1)) && std::isfinite(elevationData(i, j - 1))) {
        dz_dy = (elevationData(i, j + 1) - elevationData(i, j - 1)) / (2.0 * resolution);
        valid_gradient = true;
      }
    } else if (j == 0 && gridMap.getSize()(1) > 1) {
      if (std::isfinite(elevationData(i, j + 1))) {
        dz_dy = (elevationData(i, j + 1) - elevationData(i, j)) / resolution;
        valid_gradient = true;
      }
    } else if (j == gridMap.getSize()(1) - 1 && gridMap.getSize()(1) > 1) {
      if (std::isfinite(elevationData(i, j - 1))) {
        dz_dy = (elevationData(i, j) - elevationData(i, j - 1)) / resolution;
        valid_gradient = true;
      }
    }

    if (valid_gradient) {
      // Calculate slope angle in radians from the gradient magnitude
      const float gradient_magnitude = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
      slopeData(i, j) = std::atan(gradient_magnitude);
    } else {
      slopeData(i, j) = std::nan("1");
    }
  }
}

void addTraversabilityLayer(
  grid_map::GridMap & gridMap,
  Pointcloud::ConstPtr cloud,
  const std::string & traversabilityLayer,
  double robotHeight,
  unsigned int traversableNum,
  double groundZ)
{
  // Add traversability layer if not exists, otherwise update
  if (!gridMap.exists(traversabilityLayer)) {
    gridMap.add(traversabilityLayer);
  }
  
  grid_map::Matrix & traversabilityData = gridMap.get(traversabilityLayer);
  
  // Initialize all cells as traversable (1.0)
  traversabilityData.setConstant(1.0);
  
  // Count obstacle points in each cell (points with z <= robotHeight)
  std::vector<std::vector<unsigned int>> obstacleCount(
    gridMap.getSize()(0), 
    std::vector<unsigned int>(gridMap.getSize()(1), 0));
  
  // Iterate through point cloud and count obstacles in each cell
  for (const auto & point : cloud->points) {
    // Only consider points at or below robot height as potential obstacles
    // Also ignore points below groundZ (below floor) when projecting obstacles
    if (point.z <= robotHeight && point.z >= groundZ && std::isfinite(point.z)) {
      grid_map::Position position(point.x, point.y);
      grid_map::Index index;
      
      if (gridMap.getIndex(position, index)) {
        obstacleCount[index(0)][index(1)]++;
      }
    }
  }
  
  // Set traversability based on obstacle point count
  for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const unsigned int count = obstacleCount[index(0)][index(1)];
    
    // If obstacle points exceed threshold, mark as not traversable (0.0)
    if (count > traversableNum) {
      traversabilityData(index(0), index(1)) = 0.0;
    }
  }
}

void generateOccupancyGrid(
  const grid_map::GridMap & gridMap,
  const std::string & traversabilityLayer,
  const std::string & slopeLayer,
  double slopeThreshold,
  nav_msgs::msg::OccupancyGrid & occupancyGrid)
{
  if (!gridMap.exists(traversabilityLayer)) {
    throw std::runtime_error("Traversability layer does not exist in grid map");
  }
  if (!gridMap.exists(slopeLayer)) {
    throw std::runtime_error("Slope layer does not exist in grid map");
  }

  const grid_map::Matrix & traversabilityData = gridMap.get(traversabilityLayer);
  const grid_map::Matrix & slopeData = gridMap.get(slopeLayer);

  // Set up occupancy grid metadata
  occupancyGrid.header.frame_id = gridMap.getFrameId();
  occupancyGrid.header.stamp = rclcpp::Clock().now();
  
  occupancyGrid.info.resolution = gridMap.getResolution();
  occupancyGrid.info.width = gridMap.getSize()(0);
  occupancyGrid.info.height = gridMap.getSize()(1);
  
  // Get position of grid map origin (bottom-left corner)
  grid_map::Position position = gridMap.getPosition();
  grid_map::Length length = gridMap.getLength();
  occupancyGrid.info.origin.position.x = position.x() - length.x() / 2.0;
  occupancyGrid.info.origin.position.y = position.y() - length.y() / 2.0;
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 0.0;

  // Allocate occupancy grid data
  const size_t size = gridMap.getSize()(0) * gridMap.getSize()(1);
  occupancyGrid.data.resize(size);

  // Fill occupancy grid data
  // OccupancyGrid convention: 0 = free, 100 = occupied, -1 = unknown
  for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const float traversability = traversabilityData(index(0), index(1));
    const float slope = slopeData(index(0), index(1));

    // Calculate linear index for occupancy grid (row-major order)
    // OccupancyGrid uses standard image convention (origin at bottom-left)
    const size_t colIndex = gridMap.getSize()(0) - 1 -index(0);
    const size_t rowIndex = gridMap.getSize()(1) - 1 - index(1);
    const size_t linearIndex = rowIndex * gridMap.getSize()(0) + colIndex;

    // Determine occupancy value
    if (!std::isfinite(traversability) || !std::isfinite(slope)) {
      // Unknown cell
      occupancyGrid.data[linearIndex] = -1;
    } else if (traversability < 0.5 || slope > slopeThreshold) {
      // Not traversable: either traversability indicates obstacle or slope too steep
      occupancyGrid.data[linearIndex] = 100;
    } else {
      // Traversable: free space
      occupancyGrid.data[linearIndex] = 0;
    }
  }
}


}  // namespace grid_map_pcl
}  // namespace grid_map
