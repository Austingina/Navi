#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <mutex>
#include <limits>

using std::placeholders::_1;

class CloudTo2DLaserScanNode : public rclcpp::Node
{
public:
  CloudTo2DLaserScanNode()
  : Node("cloud_to_2d_laserscan")
  {
    declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    declare_parameter<std::string>("elevation_topic", "grid_map_from_raw_pointcloud");
    declare_parameter<std::string>("elevation_layer", "elevation");
    declare_parameter<double>("ground_tolerance", 0.05);
    declare_parameter<double>("max_height", 2.0);
    declare_parameter<std::string>("output_topic", "/cloud_as_laserscan");
    declare_parameter<double>("angle_min", -3.141592653589793);
    declare_parameter<double>("angle_max", 3.141592653589793);
    declare_parameter<double>("angle_increment", 0.008726646259971648); // 0.5 deg
    declare_parameter<double>("range_min", 0.0);
    declare_parameter<double>("range_max", 30.0);
    declare_parameter<bool>("publish_filtered_pcd", false);

    cloud_topic_ = get_parameter("cloud_topic").as_string();
    elevation_topic_ = get_parameter("elevation_topic").as_string();
    elevation_layer_ = get_parameter("elevation_layer").as_string();
    ground_tolerance_ = get_parameter("ground_tolerance").as_double();
    max_height_ = get_parameter("max_height").as_double();
    output_topic_ = get_parameter("output_topic").as_string();
    angle_min_ = get_parameter("angle_min").as_double();
    angle_max_ = get_parameter("angle_max").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();
    publish_filtered_pcd_ = get_parameter("publish_filtered_pcd").as_bool();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      elevation_topic_, 10, std::bind(&CloudTo2DLaserScanNode::gridCallback, this, _1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(), std::bind(&CloudTo2DLaserScanNode::cloudCallback, this, _1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    // publisher for filtered pointcloud after ground/max height filtering (optional)
    if (publish_filtered_pcd_) {
      pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcd", 10);
    }

    RCLCPP_INFO(get_logger(), "CloudTo2DLaserScan node started. cloud: %s, grid: %s", cloud_topic_.c_str(), elevation_topic_.c_str());
  }

private:
  void gridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid_map::GridMap grid;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid);
    grid_ = grid; // copy
    have_grid_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!have_grid_) return;

    // Transform cloud to grid frame
    sensor_msgs::msg::PointCloud2 cloud_in_grid;
    std::string grid_frame;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid_frame = grid_.getFrameId();
    }

    try {
      auto t = tf_buffer_->lookupTransform(grid_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp, tf2::durationFromSec(0.1));
      tf2::doTransform(*cloud_msg, cloud_in_grid, t);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", e.what());
      return;
    }

    // Convert original cloud (in its header frame) to PCL to compute angles/ranges
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_src;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud_src);

    // Convert transformed cloud (in grid frame) to PCL for elevation lookup
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_grid;
    pcl::fromROSMsg(cloud_in_grid, pcl_cloud_grid);

    // prepare LaserScan
    size_t ranges_n = static_cast<size_t>(std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = cloud_msg->header;
    scan_msg.header.frame_id = cloud_msg->header.frame_id;
    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.0;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;
    scan_msg.ranges.assign(ranges_n, std::numeric_limits<float>::infinity());

    // iterate points (use parallel src/grid clouds)
    if (pcl_cloud_src.size() != pcl_cloud_grid.size()) {
      RCLCPP_WARN(get_logger(), "PointCloud size mismatch after transform: src=%zu grid=%zu", pcl_cloud_src.size(), pcl_cloud_grid.size());
    }

    size_t pts_n = std::min(pcl_cloud_src.size(), pcl_cloud_grid.size());
        
    // 打印 ground_tolerance_ 与 max_height_（调试用）
    // RCLCPP_INFO(get_logger(), "ground_tolerance_=%f max_height_=%f", ground_tolerance_, max_height_);
    
  // prepare filtered pcl cloud (in original cloud frame) if requested
  pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
  if (publish_filtered_pcd_) {
    pcl_filtered.header = pcl_cloud_src.header;
  }
  for (size_t i = 0; i < pts_n; ++i) {
      const auto &p_src = pcl_cloud_src.points[i];
      const auto &p = pcl_cloud_grid.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (!std::isfinite(p_src.x) || !std::isfinite(p_src.y) || !std::isfinite(p_src.z)) continue;

      double elev = 0.0;
      bool ok = false;
      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_map::Position pos(p.x, p.y);
        if (!grid_.isInside(pos)) {
          ok = false;
        } else {
          grid_map::Index idx;
          grid_.getIndex(pos, idx);
          elev = grid_.at(elevation_layer_, idx);
          if (!std::isfinite(elev)) {
            ok = false;
          } else {
            ok = true;
          }
        }
      }
      if (!ok) continue;

      if (p.z <= elev + ground_tolerance_) continue; // below or on ground -> drop
      if (p.z > elev + max_height_) continue; // above max height -> drop

      // project to z=0 plane -> use x,y
      // compute range/angle in the original cloud frame (p_src)
      double rx = p_src.x;
      double ry = p_src.y;
      double range = std::hypot(rx, ry);
      if (range < range_min_ || range > range_max_) continue;
      double angle = std::atan2(ry, rx);
      if (angle < angle_min_ || angle > angle_max_) continue;
      double raw = (angle - angle_min_) / angle_increment_;
      int idx = static_cast<int>(std::floor(raw + 1e-12)); // 加小量以减少浮点临界误差
      if (idx < 0) continue;
      if (static_cast<size_t>(idx) >= ranges_n) {
        // 若 angle 非常接近 angle_max，允许映射到最后一个桶
        if (std::abs(angle - angle_max_) < 1e-9) {
          idx = static_cast<int>(ranges_n) - 1;
        } else {
          continue;
        }
      }
      if (range < scan_msg.ranges[idx]) scan_msg.ranges[idx] = static_cast<float>(range);
      // add the surviving point (from the original cloud frame) to filtered cloud if enabled
      if (publish_filtered_pcd_) {
        pcl_filtered.points.push_back(p_src);
      }
    }

    // publish LaserScan
    scan_pub_->publish(scan_msg);

    // publish filtered PointCloud2 if enabled and any points
    if (publish_filtered_pcd_ && !pcl_filtered.points.empty() && pcd_pub_) {
      pcl_filtered.width = static_cast<uint32_t>(pcl_filtered.points.size());
      pcl_filtered.height = 1;
      pcl_filtered.is_dense = false;

      // convert to PCLPointCloud2 then to ROS msg
      pcl::PCLPointCloud2 pcl_pc2;
      pcl::toPCLPointCloud2(pcl_filtered, pcl_pc2);
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl_conversions::fromPCL(pcl_pc2, filtered_msg);
      // use original cloud header/frame
      filtered_msg.header = cloud_msg->header;
      pcd_pub_->publish(filtered_msg);
      RCLCPP_DEBUG(get_logger(), "Published filtered pointcloud with %zu points", pcl_filtered.points.size());
    }
  }

  // members
  std::string cloud_topic_;
  std::string elevation_topic_;
  std::string elevation_layer_;
  std::string output_topic_;
  double ground_tolerance_ = 0.05;
  double max_height_ = 2.0;
  double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
  bool publish_filtered_pcd_ = false;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex grid_mutex_;
  grid_map::GridMap grid_;
  bool have_grid_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudTo2DLaserScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
