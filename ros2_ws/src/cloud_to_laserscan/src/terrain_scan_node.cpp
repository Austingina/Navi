// 地形感知节点：把两条链彻底解耦。
//
//   A. 障碍链（点云速率，~10Hz）：实时点云相对「局部地面层」做高度带过滤，
//      拍扁成 2D LaserScan (/livox_scan2)。参照用的 ground 取自最近一次 grid
//      快照（慢变量），因此障碍输出频率跟点云走，不再被 5Hz 的高程图拖住。
//
//   B. 坡度链（高程图速率，~5Hz）：在 grid 回调里算 ground/slope，并把坡度映射成
//      「分级代价」发布为 OccupancyGrid (/slope_costmap)：平地 0、缓坡代价递增、
//      到 slope_lethal_deg 即 100(lethal)。给 nav2 的 StaticLayer 当代价层用，
//      而不是往 scan 里塞假障碍点。
//
// 关键区别（相对旧 cloud_to_2d_laserscan）：障碍参照用邻域低分位地面层（跟随坡面、
// 不跟随戳出来的障碍），所以静态障碍不会被自己建的高程图吃掉；坡度独立成代价图。
// 执行器用 MultiThreadedExecutor + 独立回调组，重的 grid 处理不会阻塞点云链。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
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
    slope_cost_topic_ =
      declare_parameter<std::string>("slope_cost_topic", "/slope_costmap");
    // 地形代价点云：给自定义 TerrainLayer 用(渐变 + 持久)。每个有效 slope_cost
    // 格子发一个点(x,y=格心, 字段 cost=0..100),odom 系。
    cost_cloud_enable_ = declare_parameter<bool>("cost_cloud_enable", true);
    cost_cloud_topic_ =
      declare_parameter<std::string>("cost_cloud_topic", "/terrain_cost_cloud");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // ---- 地面层（邻域低分位）------------------------------------------------
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

    // ---- 坡度 -> 分级代价 ---------------------------------------------------
    // slope <= free_deg           -> 代价 0（视为平地）
    // free_deg < slope < lethal   -> 线性升到 99（缓坡代价递增，同等条件下避开）
    // slope >= lethal_deg         -> 100（lethal，基本过不去）
    slope_enable_ = declare_parameter<bool>("slope_enable", true);
    slope_window_ = declare_parameter<int>("slope_window", 2);        // 中心差分步长(格)
    slope_free_deg_ = declare_parameter<double>("slope_free_deg", 5.0);
    slope_lethal_deg_ = declare_parameter<double>("slope_lethal_deg", 25.0);
    // 代价曲线弯曲度：cost = 1 + pow(t, gamma)*98, t=(slope-free)/(lethal-free)。
    // gamma=1 线性；gamma<1 使中等坡度代价更高（更强软避让，但不改 lethal 阈值）。
    slope_gamma_ = declare_parameter<double>("slope_gamma", 1.0);

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

    // 独立回调组：5Hz 的重 grid 处理与 10Hz 点云链互不阻塞。
    grid_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions grid_opts;
    grid_opts.callback_group = grid_cbg_;
    rclcpp::SubscriptionOptions cloud_opts;
    cloud_opts.callback_group = cloud_cbg_;

    grid_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      elevation_topic_, 10, std::bind(&TerrainScanNode::gridCallback, this, _1),
      grid_opts);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainScanNode::cloudCallback, this, _1), cloud_opts);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    grid_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(grid_output_topic_, 1);
    slope_cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(slope_cost_topic_, 1);
    if (cost_cloud_enable_) {
      cost_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        cost_cloud_topic_, rclcpp::SensorDataQoS());
    }
    if (publish_filtered_pcd_) {
      pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcd", 10);
    }

    RCLCPP_INFO(
      get_logger(),
      "terrain_scan started. cloud=%s grid=%s -> scan=%s(~cloud rate) slope_cost=%s(~grid rate) "
      "| ground(p=%.2f, r=%d) obstacle_band=[%.2f, %.2f] slope_cost(free=%.1f, lethal=%.1f)",
      cloud_topic_.c_str(), elevation_topic_.c_str(), output_topic_.c_str(),
      slope_cost_topic_.c_str(), ground_percentile_, ground_radius_,
      obstacle_min_height_, obstacle_max_height_, slope_free_deg_, slope_lethal_deg_);
  }

private:
  struct AccumFrame
  {
    double stamp_sec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;  // grid(odom) 系
  };

  // ===== 坡度链（grid 速率）================================================

  // 收到高程图: 提取地面层 + 坡度层 + 分级代价层, 存快照, 转发增强 GridMap 与坡度代价图.
  void gridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    auto grid = std::make_shared<grid_map::GridMap>();
    grid_map::GridMapRosConverter::fromMessage(*msg, *grid);
    if (!grid->exists(elevation_layer_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "incoming grid has no '%s' layer",
        elevation_layer_.c_str());
      return;
    }

    addGroundLayer(*grid);
    if (slope_enable_) {
      addSlopeLayer(*grid);
      addSlopeCostLayer(*grid);
    }

    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid_snapshot_ = grid;  // shared_ptr 原子交换, cloud 链只需拷指针
    }

    auto out = grid_map::GridMapRosConverter::toMessage(*grid);
    if (out) grid_pub_->publish(*out);

    if (slope_enable_ && grid->exists("slope_cost")) {
      nav_msgs::msg::OccupancyGrid occ;
      // slope_cost 层已是 0..100（NaN=未知）; 直接线性映射到 OccupancyGrid.
      grid_map::GridMapRosConverter::toOccupancyGrid(*grid, "slope_cost", 0.0f, 100.0f, occ);
      slope_cost_pub_->publish(occ);

      // 地形代价点云(给 TerrainLayer): 每个有效格发一个点, 附 cost=0..100.
      if (cost_cloud_enable_ && cost_cloud_pub_) {
        publishCostCloud(*grid, msg->header.stamp);
      }
    }
  }

  // 把 slope_cost 层拍成 PointCloud2(x,y=格心, z=0, float cost=0..100), grid(odom) 系.
  void publishCostCloud(const grid_map::GridMap & grid, const rclcpp::Time & stamp)
  {
    const grid_map::Matrix & cost = grid["slope_cost"];
    const grid_map::Size size = grid.getSize();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = grid.getFrameId();
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "cost", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(static_cast<size_t>(size(0)) * static_cast<size_t>(size(1)));

    sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> oc(cloud, "cost");

    size_t n = 0;
    for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      const float c = cost(idx(0), idx(1));
      if (!std::isfinite(c)) continue;
      grid_map::Position pos;
      grid.getPosition(idx, pos);
      *ox = static_cast<float>(pos.x());
      *oy = static_cast<float>(pos.y());
      *oz = 0.0f;
      *oc = c;
      ++ox; ++oy; ++oz; ++oc; ++n;
    }
    mod.resize(n);
    cost_cloud_pub_->publish(cloud);
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

  // 坡度层 = 地面层的空间梯度(中心差分), 单位: 度.
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

  // 分级代价层: slope(度) -> 0..100. 平地 0, 缓坡线性升到 99, >=lethal 为 100.
  // NaN(坡度未知) 保持 NaN -> OccupancyGrid 里成 -1(未知), 不覆盖代价地图其它层.
  void addSlopeCostLayer(grid_map::GridMap & grid) const
  {
    if (!grid.exists("slope")) return;
    const grid_map::Matrix & s = grid["slope"];
    const int rows = s.rows(), cols = s.cols();
    const double free = slope_free_deg_;
    const double lethal = std::max(slope_free_deg_ + 1e-3, slope_lethal_deg_);

    grid_map::Matrix cost(rows, cols);
    cost.setConstant(std::numeric_limits<float>::quiet_NaN());
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const float sv = s(i, j);
        if (!std::isfinite(sv)) continue;  // 保持 NaN -> 未知
        double c;
        if (sv <= free) {
          c = 0.0;
        } else if (sv >= lethal) {
          c = 100.0;
        } else {
          const double t = (sv - free) / (lethal - free);   // 0..1
          const double tg = (slope_gamma_ == 1.0) ? t : std::pow(t, slope_gamma_);
          c = 1.0 + tg * 98.0;  // 1..99；gamma<1 抬高中段代价
        }
        cost(i, j) = static_cast<float>(c);
      }
    }
    grid.add("slope_cost", cost);
  }

  // ===== 障碍链（点云速率）================================================

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
    // 拿最近一次的 grid 快照（只拷 shared_ptr，几乎零开销；内容只读，跨线程安全）。
    std::shared_ptr<const grid_map::GridMap> grid;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid = grid_snapshot_;
    }
    if (!grid || !grid->exists("ground")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no ground snapshot yet");
      return;
    }
    const std::string grid_frame = grid->getFrameId();

    // 点云原始系 -> grid(odom) 系.
    geometry_msgs::msg::TransformStamped t_body_to_grid;
    try {
      t_body_to_grid = tf_buffer_->lookupTransform(
        grid_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp,
        tf2::durationFromSec(0.5));
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
        grid_frame, base_frame_, cloud_msg->header.stamp, tf2::durationFromSec(0.5));
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
      if (grid->isInside(pos)) {
        grid_map::Index idx;
        if (grid->getIndex(pos, idx)) {
          ground = grid->at("ground", idx);
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

  // ---- params ----
  std::string cloud_topic_, elevation_topic_, elevation_layer_;
  std::string output_topic_, grid_output_topic_, slope_cost_topic_, base_frame_;
  bool cost_cloud_enable_;
  std::string cost_cloud_topic_;
  double ground_percentile_;
  int ground_radius_, ground_min_cells_;
  double obstacle_min_height_, obstacle_max_height_;
  bool fallback_on_unknown_;
  double fallback_min_height_, fallback_max_height_;
  bool slope_enable_;
  int slope_window_;
  double slope_free_deg_, slope_lethal_deg_, slope_gamma_;
  double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
  bool accumulate_enable_;
  double accumulate_time_;
  int accumulate_max_frames_;
  bool publish_filtered_pcd_;

  // ---- ros ----
  rclcpp::CallbackGroup::SharedPtr grid_cbg_;
  rclcpp::CallbackGroup::SharedPtr cloud_cbg_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slope_cost_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex grid_mutex_;
  std::shared_ptr<const grid_map::GridMap> grid_snapshot_;
  std::deque<AccumFrame> frame_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainScanNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
