// 地形感知的 2D 障碍 LaserScan + 坡度层生成节点。
//
// 设计要点（与旧 cloud_to_2d_laserscan 的关键区别）：
//   1. 障碍参照用「局部地面层」= 高程图邻域低分位（跟随坡面，但不跟随戳出来的
//      障碍），而不是 raw elevation（会把障碍顶写进去，导致静态障碍被自己吃掉）。
//   2. 障碍点仍来自实时点云（响应快）；地面参照来自 grid map（慢变量，5Hz 足够）。
//   3. 坡度层 = 地面层的空间梯度，独立发布；过陡的格子额外投影进同一条 LaserScan。
//   4. 地面未知的格子回退到「脚底高度带」（base_link 在 odom 下的 z）。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using std::placeholders::_1;

class TerrainScanNode : public rclcpp::Node
{
public:
  TerrainScanNode()
  : Node("terrain_scan_node")
  {
    // ---- I/O ----------------------------------------------------------------
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/livox/points");
    elevation_topic_ =
      declare_parameter<std::string>("elevation_topic", "/grid_map_from_raw_pointcloud");
    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox_scan2");
    grid_output_topic_ =
      declare_parameter<std::string>("grid_output_topic", "/terrain_grid_map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // ---- 地面层（邻域低分位）------------------------------------------------
    // percentile=0 -> 最小值(最激进的地面), 0.5 -> 中位数. 默认取低分位, 这样
    // 障碍/植被戳出来的高点不会抬高地面参照, 静态障碍才能被检出.
    ground_percentile_ = declare_parameter<double>("ground_percentile", 0.15);
    ground_radius_ = declare_parameter<int>("ground_radius", 3);     // cells -> (2r+1)^2 窗
    ground_min_cells_ = declare_parameter<int>("ground_min_cells", 4);

    // ---- 障碍高度带(相对地面)------------------------------------------------
    obstacle_min_height_ = declare_parameter<double>("obstacle_min_height", 0.03);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 1.6);

    // ---- 地面未知时的脚底高度带回退 -----------------------------------------
    fallback_on_unknown_ = declare_parameter<bool>("fallback_on_unknown_elevation", true);
    fallback_min_height_ = declare_parameter<double>("fallback_min_height", 0.05);
    fallback_max_height_ = declare_parameter<double>("fallback_max_height", 1.6);

    // ---- 坡度 ---------------------------------------------------------------
    slope_enable_ = declare_parameter<bool>("slope_enable", true);
    slope_window_ = declare_parameter<int>("slope_window", 2);        // 中心差分步长(格)
    slope_obstacle_deg_ = declare_parameter<double>("slope_obstacle_deg", 25.0);
    slope_as_obstacle_ = declare_parameter<bool>("slope_as_obstacle_enable", true);

    // ---- LaserScan 几何 -----------------------------------------------------
    angle_min_ = declare_parameter<double>("angle_min", -3.141592653589793);
    angle_max_ = declare_parameter<double>("angle_max", 3.141592653589793);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.008726646259971648);
    range_min_ = declare_parameter<double>("range_min", 0.1);
    range_max_ = declare_parameter<double>("range_max", 30.0);

    // ---- 多帧累积(把稀疏单帧补满, 障碍不闪烁)------------------------------
    accumulate_enable_ = declare_parameter<bool>("accumulate_enable", true);
    accumulate_time_ = declare_parameter<double>("accumulate_time", 0.8);
    accumulate_max_frames_ = declare_parameter<int>("accumulate_max_frames", 10);

    publish_filtered_pcd_ = declare_parameter<bool>("publish_filtered_pcd", false);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      elevation_topic_, 10, std::bind(&TerrainScanNode::gridCallback, this, _1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainScanNode::cloudCallback, this, _1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    grid_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(grid_output_topic_, 1);
    if (publish_filtered_pcd_) {
      pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcd", 10);
    }

    RCLCPP_INFO(
      get_logger(),
      "terrain_scan started. cloud=%s grid=%s -> scan=%s grid_out=%s | ground(p=%.2f, r=%d) "
      "obstacle_band=[%.2f, %.2f] slope(%s, >%.1fdeg as obstacle=%s)",
      cloud_topic_.c_str(), elevation_topic_.c_str(), output_topic_.c_str(),
      grid_output_topic_.c_str(), ground_percentile_, ground_radius_,
      obstacle_min_height_, obstacle_max_height_, slope_enable_ ? "on" : "off",
      slope_obstacle_deg_, slope_as_obstacle_ ? "yes" : "no");
  }

private:
  struct AccumFrame
  {
    double stamp_sec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;  // grid(odom) 系
  };

  // 收到高程图: 提取地面层 + 坡度层, 存快照并转发增强后的 GridMap.
  void gridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid);
    if (!grid.exists(elevation_layer_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "incoming grid has no '%s' layer",
        elevation_layer_.c_str());
      return;
    }

    addGroundLayer(grid);
    if (slope_enable_) addSlopeLayer(grid);

    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid_ = grid;
      have_grid_ = true;
    }

    auto out = grid_map::GridMapRosConverter::toMessage(grid);
    if (out) grid_pub_->publish(*out);
  }

  // 地面层 = 邻域低分位. 低分位对"戳出来的障碍/植被"免疫, 又能跟随坡面.
  void addGroundLayer(grid_map::GridMap & grid) const
  {
    const grid_map::Matrix & elev = grid[elevation_layer_];
    const grid_map::Size size = grid.getSize();
    const int rows = size(0), cols = size(1);
    const int r = std::max(1, ground_radius_);
    const double pct = std::clamp(ground_percentile_, 0.0, 1.0);

    grid_map::Matrix ground(rows, cols);
    std::vector<float> win;
    win.reserve((2 * r + 1) * (2 * r + 1));
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        win.clear();
        const int i0 = std::max(0, i - r), i1 = std::min(rows - 1, i + r);
        const int j0 = std::max(0, j - r), j1 = std::min(cols - 1, j + r);
        for (int ii = i0; ii <= i1; ++ii) {
          for (int jj = j0; jj <= j1; ++jj) {
            const float v = elev(ii, jj);
            if (std::isfinite(v)) win.push_back(v);
          }
        }
        if (static_cast<int>(win.size()) >= ground_min_cells_) {
          const size_t k = static_cast<size_t>(
            std::lround(pct * static_cast<double>(win.size() - 1)));
          std::nth_element(win.begin(), win.begin() + k, win.end());
          ground(i, j) = win[k];
        } else {
          ground(i, j) = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
    grid.add("ground", ground);
  }

  // 坡度层 = 地面层的空间梯度(中心差分), 单位: 度. 成本为栅格数量级, 与点数无关.
  void addSlopeLayer(grid_map::GridMap & grid) const
  {
    if (!grid.exists("ground")) return;
    const grid_map::Matrix & g = grid["ground"];
    const grid_map::Size size = grid.getSize();
    const int rows = size(0), cols = size(1);
    const int w = std::max(1, slope_window_);
    const double res = grid.getResolution();
    const double span = 2.0 * w * res;

    grid_map::Matrix slope(rows, cols);
    slope.setConstant(std::numeric_limits<float>::quiet_NaN());
    for (int i = w; i < rows - w; ++i) {
      for (int j = w; j < cols - w; ++j) {
        const float gxa = g(i - w, j), gxb = g(i + w, j);
        const float gya = g(i, j - w), gyb = g(i, j + w);
        if (!std::isfinite(gxa) || !std::isfinite(gxb) ||
          !std::isfinite(gya) || !std::isfinite(gyb))
        {
          continue;
        }
        const double dzdx = (gxb - gxa) / span;
        const double dzdy = (gyb - gya) / span;
        const double rad = std::atan(std::hypot(dzdx, dzdy));
        slope(i, j) = static_cast<float>(rad * 180.0 / M_PI);
      }
    }
    grid.add("slope", slope);
  }

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

  // 把 body 系的一个点写入 LaserScan(取每个方位角最近距离).
  void insertPointToScan(
    const pcl::PointXYZ & p, sensor_msgs::msg::LaserScan & scan, size_t ranges_n) const
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
    const double range = std::hypot(p.x, p.y);
    if (range < range_min_ || range > range_max_) return;
    const double angle = std::atan2(p.y, p.x);
    if (angle < angle_min_ || angle > angle_max_) return;
    int idx = static_cast<int>(std::floor((angle - angle_min_) / angle_increment_ + 1e-12));
    if (idx < 0) return;
    if (static_cast<size_t>(idx) >= ranges_n) {
      if (std::abs(angle - angle_max_) < 1e-9) idx = static_cast<int>(ranges_n) - 1;
      else return;
    }
    if (range < scan.ranges[idx]) scan.ranges[idx] = static_cast<float>(range);
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
    if (!grid_snapshot.exists("ground")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "grid snapshot has no 'ground' layer");
      return;
    }

    // 点云原始系 -> grid(odom) 系.
    geometry_msgs::msg::TransformStamped t_body_to_grid;
    try {
      t_body_to_grid = tf_buffer_->lookupTransform(
        grid_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp,
        tf2::durationFromSec(0.1));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "TF %s<-%s failed: %s", grid_frame.c_str(),
        cloud_msg->header.frame_id.c_str(), e.what());
      return;
    }

    // 脚底高度: base_link 原点在 grid(odom) 下的 z. 拿不到就禁用脚底回退.
    bool have_foot = false;
    double z_foot = 0.0;
    try {
      auto t_foot = tf_buffer_->lookupTransform(
        grid_frame, base_frame_, cloud_msg->header.stamp, tf2::durationFromSec(0.1));
      z_foot = t_foot.transform.translation.z;
      have_foot = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "foot TF %s<-%s failed: %s",
        grid_frame.c_str(), base_frame_.c_str(), e.what());
    }

    sensor_msgs::msg::PointCloud2 cloud_in_grid;
    tf2::doTransform(*cloud_msg, cloud_in_grid, t_body_to_grid);
    pcl::PointCloud<pcl::PointXYZ> pcl_grid;
    pcl::fromROSMsg(cloud_in_grid, pcl_grid);

    // ---- Pass 1: 逐点相对"局部地面"做高度带过滤, 存活点存 grid(odom) 系 ----
    auto survivors = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    survivors->points.reserve(pcl_grid.size());
    for (const auto & p : pcl_grid.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      double ground = 0.0;
      bool have_ground = false;
      const grid_map::Position pos(p.x, p.y);
      if (grid_snapshot.isInside(pos)) {
        grid_map::Index idx;
        if (grid_snapshot.getIndex(pos, idx)) {
          ground = grid_snapshot.at("ground", idx);
          have_ground = std::isfinite(ground);
        }
      }

      if (have_ground) {
        const double h = p.z - ground;              // 相对局部地面高度
        if (h <= obstacle_min_height_) continue;    // 贴地/地面 -> 丢
        if (h > obstacle_max_height_) continue;     // 过高(天花板等) -> 丢
      } else {
        // 地面未知: 回退到脚底高度带(平地够用). 拿不到脚底就直接丢.
        if (!fallback_on_unknown_ || !have_foot) continue;
        const double h = p.z - z_foot;
        if (h < fallback_min_height_ || h > fallback_max_height_) continue;
      }
      survivors->points.push_back(p);
    }

    // ---- 滑动窗累积(grid/odom 系)------------------------------------------
    const double stamp_sec = rclcpp::Time(cloud_msg->header.stamp).seconds();
    if (!accumulate_enable_) frame_buffer_.clear();
    frame_buffer_.push_back({stamp_sec, survivors});
    while (!frame_buffer_.empty() &&
      (stamp_sec - frame_buffer_.front().stamp_sec > accumulate_time_ ||
      frame_buffer_.size() > static_cast<size_t>(std::max(1, accumulate_max_frames_))))
    {
      frame_buffer_.pop_front();
    }

    pcl::PointCloud<pcl::PointXYZ> merged_grid;
    for (const auto & f : frame_buffer_) merged_grid += *f.points;

    // 重新投回当前 body 系, 使距离/角度与实时传感器位姿一致.
    const Eigen::Matrix4f t_grid_to_body = toMatrix(t_body_to_grid).inverse();
    pcl::PointCloud<pcl::PointXYZ> merged_body;
    pcl::transformPointCloud(merged_grid, merged_body, t_grid_to_body);

    // ---- Pass 2: 扁平化为 2D LaserScan -------------------------------------
    const size_t ranges_n =
      static_cast<size_t>(std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud_msg->header;
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.assign(ranges_n, std::numeric_limits<float>::infinity());

    pcl::PointCloud<pcl::PointXYZ> kept_body;
    if (publish_filtered_pcd_) kept_body.points.reserve(merged_body.points.size());
    for (const auto & p : merged_body.points) {
      insertPointToScan(p, scan, ranges_n);
      if (publish_filtered_pcd_ && std::isfinite(p.x) && std::isfinite(p.y)) {
        kept_body.points.push_back(p);
      }
    }

    // ---- 过陡的格子直接投影进同一条 scan(即使当前帧没打到点)-------------
    if (slope_enable_ && slope_as_obstacle_ && grid_snapshot.exists("slope")) {
      addSteepCellsToScan(grid_snapshot, t_grid_to_body, scan, ranges_n);
    }

    scan_pub_->publish(scan);

    if (publish_filtered_pcd_ && !kept_body.points.empty() && pcd_pub_) {
      kept_body.width = static_cast<uint32_t>(kept_body.points.size());
      kept_body.height = 1;
      kept_body.is_dense = false;
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(kept_body, out);
      out.header = cloud_msg->header;
      pcd_pub_->publish(out);
    }
  }

  // 遍历 slope 层, 把过陡且地面有效的格心投影进 scan. bounded 约栅格数量级.
  void addSteepCellsToScan(
    const grid_map::GridMap & grid, const Eigen::Matrix4f & t_grid_to_body,
    sensor_msgs::msg::LaserScan & scan, size_t ranges_n) const
  {
    const grid_map::Matrix & slope = grid["slope"];
    const grid_map::Matrix & ground = grid["ground"];
    const grid_map::Size size = grid.getSize();
    for (int i = 0; i < size(0); ++i) {
      for (int j = 0; j < size(1); ++j) {
        const grid_map::Index idx(i, j);
        const float s = slope(i, j);
        const float g = ground(i, j);
        if (!std::isfinite(s) || !std::isfinite(g)) continue;
        if (s <= slope_obstacle_deg_) continue;
        grid_map::Position pos;
        if (!grid.getPosition(idx, pos)) continue;
        const Eigen::Vector4f pg(
          static_cast<float>(pos.x()), static_cast<float>(pos.y()), g, 1.f);
        const Eigen::Vector4f pb = t_grid_to_body * pg;
        insertPointToScan(pcl::PointXYZ(pb.x(), pb.y(), pb.z()), scan, ranges_n);
      }
    }
  }

  // ---- params ----
  std::string cloud_topic_, elevation_topic_, elevation_layer_;
  std::string output_topic_, grid_output_topic_, base_frame_;
  double ground_percentile_;
  int ground_radius_, ground_min_cells_;
  double obstacle_min_height_, obstacle_max_height_;
  bool fallback_on_unknown_;
  double fallback_min_height_, fallback_max_height_;
  bool slope_enable_, slope_as_obstacle_;
  int slope_window_;
  double slope_obstacle_deg_;
  double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
  bool accumulate_enable_;
  double accumulate_time_;
  int accumulate_max_frames_;
  bool publish_filtered_pcd_;

  // ---- ros ----
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex grid_mutex_;
  grid_map::GridMap grid_;
  bool have_grid_ = false;
  std::deque<AccumFrame> frame_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainScanNode>());
  rclcpp::shutdown();
  return 0;
}
