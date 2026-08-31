#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

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
    declare_parameter<bool>("fallback_on_unknown_elevation", true);
    declare_parameter<double>("fallback_min_height", -0.4);
    declare_parameter<double>("fallback_max_height", 1.6);
    declare_parameter<std::string>("output_topic", "/cloud_as_laserscan");
    declare_parameter<double>("angle_min", -3.141592653589793);
    declare_parameter<double>("angle_max", 3.141592653589793);
    declare_parameter<double>("angle_increment", 0.008726646259971648); // 0.5 deg
    declare_parameter<double>("range_min", 0.0);
    declare_parameter<double>("range_max", 30.0);
    declare_parameter<bool>("publish_filtered_pcd", false);
    // ---- Gravity-aligned (level) scan projection --------------------------
    // The input cloud frame (imu_link) is physically tilted (~15 deg), so
    // flattening the scan in that frame produces a tilted 2D scan. When
    // enabled, we instead flatten the accumulated cloud in a gravity-aligned
    // frame that shares the sensor origin and the robot heading (yaw) but has
    // zero roll/pitch, i.e. a true projection onto the world horizontal
    // (global z=0) plane. That level frame is broadcast on TF as
    // 'level_frame_id' so downstream consumers place the scan correctly.
    declare_parameter<bool>("flatten_gravity_aligned", true);
    declare_parameter<std::string>("level_frame_id", "livox_scan_level");
    // ---- Multi-frame accumulation -----------------------------------------
    // A single Mid-360 sweep is sparse; obstacles are only partially covered
    // per frame. We keep the surviving obstacle points of the last few frames
    // in the (stable) grid frame and re-project the whole window into the
    // current body frame before flattening to 2D. This densifies the scan and
    // stops obstacles from flickering in/out between frames.
    declare_parameter<bool>("accumulate_enable", true);
    declare_parameter<double>("accumulate_time", 0.4);   // sliding window [s]
    declare_parameter<int>("accumulate_max_frames", 10); // hard cap on frames
    // ---- Robust ground reference ------------------------------------------
    // The elevation map can hold spurious "sunken" cells (elevation written
    // well below the true local ground, e.g. from transient odometry z dips).
    // Comparing live ground against such a single cell marks it as an obstacle.
    // Use a neighborhood median of the elevation layer as the ground reference
    // so a few sunken outlier cells no longer create a false wall, while real
    // obstacles (which sit above the true ground) are unaffected. The median
    // also tracks slopes, so this is safe on inclines.
    declare_parameter<bool>("robust_ground_enable", true);
    declare_parameter<int>("robust_ground_radius", 3);    // cells; 3 -> 7x7 window
    declare_parameter<int>("robust_ground_min_cells", 4); // min finite cells in window
    // ---- Accumulation re-validation ---------------------------------------
    // Re-check every accumulated point against the CURRENT elevation map each
    // cycle and drop those no longer above ground. This removes TRANSIENT false
    // positives that would otherwise linger for the whole window. Only
    // meaningful together with accumulate_enable. It does NOT fix persistent map
    // errors: a cell that stays sunken re-validates as an obstacle every time.
    declare_parameter<bool>("revalidate_enable", false);
    // ---- Per-frame self-calibration ---------------------------------------
    // A transient odometry z jump lifts the whole live cloud relative to the
    // (lagging) elevation map, so every ground point reads as an obstacle. Each
    // frame we estimate that uniform offset as the median of (live_z - elev)
    // over near-ground points and subtract it before the obstacle test. This
    // removes a GLOBAL vertical bias; it leaves real slopes alone (there elev
    // tracks the terrain, so the median stays ~0) and does not fix per-cell
    // sunken outliers (that is robust_ground's job).
    declare_parameter<bool>("self_calibrate_enable", false);
    declare_parameter<double>("self_calibrate_band", 0.5);    // |z-elev| window for ground candidates
    declare_parameter<int>("self_calibrate_min_points", 100); // need this many candidates to trust dz
    declare_parameter<double>("self_calibrate_max_offset", 1.0); // clamp; ignore crazy estimates

    cloud_topic_ = get_parameter("cloud_topic").as_string();
    elevation_topic_ = get_parameter("elevation_topic").as_string();
    elevation_layer_ = get_parameter("elevation_layer").as_string();
    ground_tolerance_ = get_parameter("ground_tolerance").as_double();
    max_height_ = get_parameter("max_height").as_double();
    fallback_on_unknown_elevation_ =
      get_parameter("fallback_on_unknown_elevation").as_bool();
    fallback_min_height_ = get_parameter("fallback_min_height").as_double();
    fallback_max_height_ = get_parameter("fallback_max_height").as_double();
    output_topic_ = get_parameter("output_topic").as_string();
    angle_min_ = get_parameter("angle_min").as_double();
    angle_max_ = get_parameter("angle_max").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();
    publish_filtered_pcd_ = get_parameter("publish_filtered_pcd").as_bool();
    flatten_gravity_aligned_ = get_parameter("flatten_gravity_aligned").as_bool();
    level_frame_id_ = get_parameter("level_frame_id").as_string();
    accumulate_enable_ = get_parameter("accumulate_enable").as_bool();
    accumulate_time_ = get_parameter("accumulate_time").as_double();
    accumulate_max_frames_ = get_parameter("accumulate_max_frames").as_int();
    robust_ground_enable_ = get_parameter("robust_ground_enable").as_bool();
    robust_ground_radius_ = get_parameter("robust_ground_radius").as_int();
    robust_ground_min_cells_ = get_parameter("robust_ground_min_cells").as_int();
    // Layer the cloud callback reads its ground reference from.
    active_elevation_layer_ =
      robust_ground_enable_ ? std::string("elevation_robust") : elevation_layer_;
    revalidate_enable_ = get_parameter("revalidate_enable").as_bool();
    self_calibrate_enable_ = get_parameter("self_calibrate_enable").as_bool();
    self_calibrate_band_ = get_parameter("self_calibrate_band").as_double();
    self_calibrate_min_points_ = get_parameter("self_calibrate_min_points").as_int();
    self_calibrate_max_offset_ = get_parameter("self_calibrate_max_offset").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    grid_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      elevation_topic_, 10, std::bind(&CloudTo2DLaserScanNode::gridCallback, this, _1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(), std::bind(&CloudTo2DLaserScanNode::cloudCallback, this, _1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    // publisher for filtered pointcloud after ground/max height filtering (optional)
    if (publish_filtered_pcd_) {
      pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcd", 10);
    }

    RCLCPP_INFO(
      get_logger(),
      "CloudTo2DLaserScan node started. cloud: %s, grid: %s, accumulate: %s (%.2fs, <=%d frames), "
      "robust_ground: %s (r=%d), revalidate: %s, self_calibrate: %s",
      cloud_topic_.c_str(), elevation_topic_.c_str(),
      accumulate_enable_ ? "on" : "off", accumulate_time_, accumulate_max_frames_,
      robust_ground_enable_ ? "on" : "off", robust_ground_radius_,
      revalidate_enable_ ? "on" : "off", self_calibrate_enable_ ? "on" : "off");
  }

private:
  // One accumulated frame: surviving obstacle points expressed in the grid
  // (odom) frame, tagged with the source cloud stamp in seconds.
  struct AccumFrame
  {
    double stamp_sec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  };

  void gridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid);
    if (robust_ground_enable_ && grid.exists(elevation_layer_)) {
      addRobustGroundLayer(grid);
    }
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid_ = grid; // copy
    have_grid_ = true;
  }

  // Add "elevation_robust" = per-cell neighborhood median of the elevation
  // layer. A few sunken outlier cells are outvoted by their neighbors, so the
  // ground reference stays at the true local surface (and follows slopes).
  void addRobustGroundLayer(grid_map::GridMap & grid) const
  {
    const grid_map::Matrix & elev = grid[elevation_layer_];
    const grid_map::Size size = grid.getSize();
    const int rows = size(0);
    const int cols = size(1);
    const int r = std::max(1, robust_ground_radius_);
    grid_map::Matrix robust(rows, cols);
    std::vector<float> window;
    window.reserve((2 * r + 1) * (2 * r + 1));
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        window.clear();
        const int i0 = std::max(0, i - r), i1 = std::min(rows - 1, i + r);
        const int j0 = std::max(0, j - r), j1 = std::min(cols - 1, j + r);
        for (int ii = i0; ii <= i1; ++ii) {
          for (int jj = j0; jj <= j1; ++jj) {
            const float v = elev(ii, jj);
            if (std::isfinite(v)) window.push_back(v);
          }
        }
        if (static_cast<int>(window.size()) >= robust_ground_min_cells_) {
          const size_t mid = window.size() / 2;
          std::nth_element(window.begin(), window.begin() + mid, window.end());
          robust(i, j) = window[mid];
        } else {
          robust(i, j) = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
    grid.add("elevation_robust", robust);
  }

  // Rigid transform (TransformStamped) -> 4x4 float matrix.
  static Eigen::Matrix4f toMatrix(const geometry_msgs::msg::TransformStamped & ts)
  {
    const auto & tr = ts.transform.translation;
    const auto & q = ts.transform.rotation;
    Eigen::Quaternionf quat(
      static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z));
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    m(0, 3) = static_cast<float>(tr.x);
    m(1, 3) = static_cast<float>(tr.y);
    m(2, 3) = static_cast<float>(tr.z);
    return m;
  }

  // Re-check whether an odom-frame point is still an obstacle against the
  // current elevation map. Used to re-validate accumulated (older) points.
  bool stillObstacle(const grid_map::GridMap & grid, const pcl::PointXYZ & p) const
  {
    const grid_map::Position pos(p.x, p.y);
    if (grid.isInside(pos)) {
      grid_map::Index idx;
      if (grid.getIndex(pos, idx)) {
        const float elev = grid.at(active_elevation_layer_, idx);
        if (std::isfinite(elev)) {
          return p.z > elev + ground_tolerance_ && p.z <= elev + max_height_;
        }
      }
    }
    // No elevation here: we cannot re-check the body-frame fallback band for an
    // accumulated point, so keep it only if fallback mode is enabled.
    return fallback_on_unknown_elevation_;
  }

  // Median of (live_z - elev) over near-ground points = this frame's uniform
  // vertical offset between the live cloud and the elevation map. Returns 0 if
  // too few candidates or the estimate is implausibly large.
  float estimateZOffset(
    const grid_map::GridMap & grid,
    const pcl::PointCloud<pcl::PointXYZ> & cloud_grid, size_t pts_n)
  {
    std::vector<float> deltas;
    deltas.reserve(pts_n);
    for (size_t i = 0; i < pts_n; ++i) {
      const auto & p = cloud_grid.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const grid_map::Position pos(p.x, p.y);
      if (!grid.isInside(pos)) continue;
      grid_map::Index idx;
      if (!grid.getIndex(pos, idx)) continue;
      const float elev = grid.at(active_elevation_layer_, idx);
      if (!std::isfinite(elev)) continue;
      const float d = p.z - elev;
      if (std::abs(d) <= self_calibrate_band_) deltas.push_back(d); // near-ground only
    }
    if (static_cast<int>(deltas.size()) < self_calibrate_min_points_) return 0.0f;
    const size_t mid = deltas.size() / 2;
    std::nth_element(deltas.begin(), deltas.begin() + mid, deltas.end());
    const float med = deltas[mid];
    if (std::abs(med) > self_calibrate_max_offset_) return 0.0f;
    if (std::abs(med) > ground_tolerance_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "self-calib: live-vs-map z offset = %.3f m (subtracted)", med);
    }
    return med;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!have_grid_) return;

    std::string grid_frame;
    grid_map::GridMap grid_snapshot;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid_frame = grid_.getFrameId();
      grid_snapshot = grid_;
    }

    if (!grid_snapshot.exists(active_elevation_layer_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Grid map has no '%s' layer",
        active_elevation_layer_.c_str());
      return;
    }

    // body -> grid(odom) transform at the cloud stamp.
    geometry_msgs::msg::TransformStamped t_body_to_grid;
    try {
      t_body_to_grid = tf_buffer_->lookupTransform(
        grid_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp,
        tf2::durationFromSec(0.1));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", e.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_in_grid;
    tf2::doTransform(*cloud_msg, cloud_in_grid, t_body_to_grid);

    // Original cloud (body frame) for the fallback height band; transformed
    // cloud (grid frame) for elevation lookup and for storing in the window.
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_src;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud_src);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_grid;
    pcl::fromROSMsg(cloud_in_grid, pcl_cloud_grid);

    if (pcl_cloud_src.size() != pcl_cloud_grid.size()) {
      RCLCPP_WARN(
        get_logger(), "PointCloud size mismatch after transform: src=%zu grid=%zu",
        pcl_cloud_src.size(), pcl_cloud_grid.size());
    }
    const size_t pts_n = std::min(pcl_cloud_src.size(), pcl_cloud_grid.size());

    // ---- Self-calibration: estimate this frame's uniform live-vs-map z offset
    // as the median of (live_z - elev) over near-ground points, and subtract it
    // from the obstacle test so a transient global z jump does not flag the
    // whole ground as obstacle.
    const float dz = self_calibrate_enable_ ?
      estimateZOffset(grid_snapshot, pcl_cloud_grid, pts_n) : 0.0f;

    // ---- Pass 1: keep obstacle points, store them in the grid(odom) frame ---
    auto survivors = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    survivors->points.reserve(pts_n);
    for (size_t i = 0; i < pts_n; ++i) {
      const auto & p_src = pcl_cloud_src.points[i];
      const auto & p = pcl_cloud_grid.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (!std::isfinite(p_src.x) || !std::isfinite(p_src.y) || !std::isfinite(p_src.z)) continue;

      double elev = 0.0;
      bool have_elevation = false;
      grid_map::Position pos(p.x, p.y);
      if (grid_snapshot.isInside(pos)) {
        grid_map::Index grid_index;
        if (grid_snapshot.getIndex(pos, grid_index)) {
          elev = grid_snapshot.at(active_elevation_layer_, grid_index);
          have_elevation = std::isfinite(elev);
        }
      }

      if (have_elevation) {
        const double z = p.z - dz; // remove the global vertical offset
        if (z <= elev + ground_tolerance_) continue; // below or on ground -> drop
        if (z > elev + max_height_) continue;          // above max height -> drop
      } else {
        // A rolling elevation map is initially sparse. Do not make obstacles
        // disappear in uncovered cells; use a conservative body-frame band.
        if (!fallback_on_unknown_elevation_) continue;
        if (p_src.z < fallback_min_height_ || p_src.z > fallback_max_height_) continue;
      }
      survivors->points.push_back(p); // grid(odom) coordinates
    }

    // ---- Sliding window over the grid(odom) frame ---------------------------
    const double stamp_sec = rclcpp::Time(cloud_msg->header.stamp).seconds();
    if (!accumulate_enable_) {
      frame_buffer_.clear();
    }
    frame_buffer_.push_back({stamp_sec, survivors});
    while (!frame_buffer_.empty() &&
      (stamp_sec - frame_buffer_.front().stamp_sec > accumulate_time_ ||
      frame_buffer_.size() > static_cast<size_t>(std::max(1, accumulate_max_frames_))))
    {
      frame_buffer_.pop_front();
    }

    // Merge the whole window (still in grid/odom frame).
    pcl::PointCloud<pcl::PointXYZ> merged_grid;
    for (const auto & f : frame_buffer_) {
      merged_grid += *f.points;
    }

    // Re-project the window into the frame we flatten the 2D scan in.
    //  - Default (flatten_gravity_aligned_): a gravity-aligned "level" frame
    //    co-located with the sensor origin, sharing the robot heading (yaw) but
    //    with roll/pitch stripped. Flattening there gives a true projection onto
    //    the world horizontal (global z=0) plane instead of the tilted imu_link
    //    plane. The frame is broadcast on TF so the scan resolves downstream.
    //  - Otherwise: the original CURRENT body frame behaviour.
    const Eigen::Matrix4f m_body_to_grid = toMatrix(t_body_to_grid);
    Eigen::Matrix4f m_flat_to_grid = m_body_to_grid;
    std::string scan_frame_id = cloud_msg->header.frame_id;
    if (flatten_gravity_aligned_) {
      // Sensor origin in odom and the body heading projected onto the horizontal
      // plane (drops roll/pitch, keeps only the gravity-aligned yaw).
      const Eigen::Vector3f t = m_body_to_grid.block<3, 1>(0, 3);
      const Eigen::Vector3f x_body = m_body_to_grid.block<3, 1>(0, 0);
      const double yaw = std::atan2(x_body.y(), x_body.x());
      const Eigen::Matrix3f R_level =
        Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
      m_flat_to_grid.setIdentity();
      m_flat_to_grid.block<3, 3>(0, 0) = R_level;
      m_flat_to_grid.block<3, 1>(0, 3) = t;
      scan_frame_id = level_frame_id_;

      // Broadcast grid(odom) -> level frame so the scan's frame_id resolves.
      geometry_msgs::msg::TransformStamped tf_level;
      tf_level.header.stamp = cloud_msg->header.stamp;
      tf_level.header.frame_id = grid_frame;
      tf_level.child_frame_id = level_frame_id_;
      tf_level.transform.translation.x = t.x();
      tf_level.transform.translation.y = t.y();
      tf_level.transform.translation.z = t.z();
      const Eigen::Quaternionf q_level(R_level);
      tf_level.transform.rotation.x = q_level.x();
      tf_level.transform.rotation.y = q_level.y();
      tf_level.transform.rotation.z = q_level.z();
      tf_level.transform.rotation.w = q_level.w();
      tf_broadcaster_->sendTransform(tf_level);
    }
    const Eigen::Matrix4f t_grid_to_body = m_flat_to_grid.inverse();
    pcl::PointCloud<pcl::PointXYZ> merged_body;
    pcl::transformPointCloud(merged_grid, merged_body, t_grid_to_body);

    // ---- Pass 2: flatten to 2D LaserScan (nearest range per azimuth bin) ----
    const size_t ranges_n =
      static_cast<size_t>(std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = cloud_msg->header;
    scan_msg.header.frame_id = scan_frame_id;
    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.0;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;
    scan_msg.ranges.assign(ranges_n, std::numeric_limits<float>::infinity());

    pcl::PointCloud<pcl::PointXYZ> kept_body; // points that made it into the scan (for optional pcd)
    if (publish_filtered_pcd_) kept_body.points.reserve(merged_body.points.size());
    for (size_t i = 0; i < merged_body.points.size(); ++i) {
      const auto & p = merged_body.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
      // Re-validate accumulated points against the latest elevation map so a
      // point only counts if it is STILL above ground now.
      if (revalidate_enable_ && !stillObstacle(grid_snapshot, merged_grid.points[i])) continue;
      const double range = std::hypot(p.x, p.y);
      if (range < range_min_ || range > range_max_) continue;
      const double angle = std::atan2(p.y, p.x);
      if (angle < angle_min_ || angle > angle_max_) continue;
      const double raw = (angle - angle_min_) / angle_increment_;
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
      if (publish_filtered_pcd_) kept_body.points.push_back(p);
    }

    scan_pub_->publish(scan_msg);

    // publish the points that actually made it into the scan (body frame)
    if (publish_filtered_pcd_ && !kept_body.points.empty() && pcd_pub_) {
      kept_body.width = static_cast<uint32_t>(kept_body.points.size());
      kept_body.height = 1;
      kept_body.is_dense = false;

      pcl::PCLPointCloud2 pcl_pc2;
      pcl::toPCLPointCloud2(kept_body, pcl_pc2);
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl_conversions::fromPCL(pcl_pc2, filtered_msg);
      filtered_msg.header = cloud_msg->header;
      filtered_msg.header.frame_id = scan_frame_id;
      pcd_pub_->publish(filtered_msg);
      RCLCPP_DEBUG(
        get_logger(), "Published filtered pointcloud with %zu points",
        kept_body.points.size());
    }
  }

  // members
  std::string cloud_topic_;
  std::string elevation_topic_;
  std::string elevation_layer_;
  std::string output_topic_;
  double ground_tolerance_ = 0.05;
  double max_height_ = 2.0;
  bool fallback_on_unknown_elevation_ = true;
  double fallback_min_height_ = -0.4;
  double fallback_max_height_ = 1.6;
  double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
  bool publish_filtered_pcd_ = false;
  bool flatten_gravity_aligned_ = true;
  std::string level_frame_id_;
  bool accumulate_enable_ = true;
  double accumulate_time_ = 0.4;
  int accumulate_max_frames_ = 10;
  bool robust_ground_enable_ = true;
  int robust_ground_radius_ = 3;
  int robust_ground_min_cells_ = 4;
  std::string active_elevation_layer_;
  bool revalidate_enable_ = false;
  bool self_calibrate_enable_ = false;
  double self_calibrate_band_ = 0.5;
  int self_calibrate_min_points_ = 100;
  double self_calibrate_max_offset_ = 1.0;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex grid_mutex_;
  grid_map::GridMap grid_;
  bool have_grid_ = false;

  std::deque<AccumFrame> frame_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudTo2DLaserScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
