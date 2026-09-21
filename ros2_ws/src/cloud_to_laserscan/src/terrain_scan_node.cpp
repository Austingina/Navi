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
#include <chrono>
#include <unordered_map>
#include <vector>
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
    // 空格清除: 把"车附近、连高程都没有"的格子当作"观测到是空的", 发 cost=0,
    // 让 TerrainLayer 能清掉旧值(它只会碰点云里出现过的格子). 见 publishCostCloud.
    free_on_empty_ = declare_parameter<bool>("free_on_empty", true);
    // 可信半径: 只在这个范围内把"无数据"解释成"空". 太大会把远处从未扫到的
    // 区域也说成空; 取值应 <= 高程图半边长(map_length/2).
    free_on_empty_radius_ = declare_parameter<double>("free_on_empty_radius", 4.5);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // ---- 负障碍(悬崖/下沉)检测 --------------------------------------------
    // 原理与既有 slope/step 判据完全不同:它们是【差分型】,需要落差两侧都有地面,
    // 而悬崖在高程图上留下的 NaN 阴影宽度 ≈ H*d/h_lidar(0.5m 深、2m 远就有 1.9m,
    // 1m 深、3m 远有 5.7m),远宽于 0.6m 的差分模板 —— 结构上凑不出差分。
    //
    // 这里改用【射线穿透证据】:elevation_mapping_cupy 的 kernel 在射线穿过一个
    // "无有效高程"的格子时,把射线在该处的高度写进 upper_bound、并置 is_upper_bound
    // (custom_kernels.py:234-241)。语义是"该格地面最高只可能在这里"。
    // 它只在射线【确实穿过】时才置位,所以遮挡区/视野外没有值 —— 天然不会把
    // 未观测误报成悬崖,这是"宁可漏报也不许堵路"这条硬约束的立足点。
    cliff_enable_ = declare_parameter<bool>("cliff_enable", true);
    cliff_ub_layer_ = declare_parameter<std::string>("cliff_upper_bound_layer", "upper_bound");
    cliff_flag_layer_ =
      declare_parameter<std::string>("cliff_is_upper_bound_layer", "is_upper_bound");
    // 种子阈值:参考地面 - upper_bound 超过它即为"深"种子。
    cliff_depth_thresh_ = declare_parameter<double>("cliff_depth_thresh", 0.25);
    // 参考地面 = z_foot - cliff_base_height。
    // 【实测选定】base_link 相对 ground 层的高度 = 0.462m。曾试过三种"局部 ground"
    // 参考,在真机平地贴墙场景下的误报(格/帧):
    //   邻域 ground 最小值            -> 20.7  (贴墙时邻域里全是墙的 ground,偏高 0.4~0.8m)
    //   邻域内 terrain_ok 的 ground   -> 84.2  (terrain_ok 本身被 NaN 放行污染,更差)
    //   车脚下的 ground               -> 恒 NaN (雷达下视盲区内没有 elevation,凑不出
    //                                           ground_min_cells,这也是本文件别处
    //                                           一直用 z_foot 而非脚下 ground 的原因)
    //   z_foot - 0.46                 ->  0.00  <- 采用
    // 方向性:此值偏大只会让参考更低、drop 更小 => 更保守;偏小才会误报
    // (实测 0.30 在阈值 0.25 下产生 7.4 格/帧)。宁可取大。
    cliff_base_height_ = declare_parameter<double>("cliff_base_height", 0.46);
    // 扩展阈值(双阈值 / hysteresis 的低阈)。seed-and-grow 只能蔓延到"确实略低于参考
    // 地面"的格子,不能蔓延到任意候选。
    // 【实测必需】只用单阈值时,连通域会从悬崖深处一路蔓延到【雷达下视盲区】——
    // 盲区格同样满足"射线穿过 + 无有效高程",于是被 flood-fill 拖进来。bag 复现里
    // 因此有 159 格 lethal 落在车实际行驶路径上,而它们的 drop 中位数是 -0.04m
    // (即 upper_bound 还高于参考地面,根本不该算悬崖)。
    cliff_grow_thresh_ = declare_parameter<double>("cliff_grow_thresh", 0.10);
    // 连通域门限:整个分量至少要有这么多"深"种子、这么大面积,才判 lethal。
    cliff_min_seed_cells_ = declare_parameter<int>("cliff_min_seed_cells", 6);
    cliff_min_area_cells_ = declare_parameter<int>("cliff_min_area_cells", 10);
    // 时间确认(5Hz -> 2 帧 = 0.4s)。upper_bound 本身已是持久层,故这一项是二阶的。
    cliff_confirm_frames_ = declare_parameter<int>("cliff_confirm_frames", 2);
    // 判定环带:内径由几何决定 —— 雷达高约 0.5m、最大俯角约 22°,近于 1.3m 不可能有
    // 掠过射线,该范围内的候选属几何矛盾噪声。外径避开高程图边缘(map_length/2=5.0)。
    cliff_min_range_ = declare_parameter<double>("cliff_min_range", 1.2);
    cliff_max_range_ = declare_parameter<double>("cliff_max_range", 4.5);
    // 参考面取"坡面外推"而不是"水平面"。
    // 【为什么必须这样】drop 原本是拿 upper_bound 去比 z_foot - cliff_base_height 这条
    // 【水平线】。连续下坡时前方地面本来就低于它 d*tan(theta),于是整片掠射阴影都被判成
    // 悬崖 —— 2026-08-26 bag 的 600.2~601.6s 实测:环带内有效高程拟合坡度 7.7~7.9deg,
    // 每帧 749~795 格被判 cliff(占 10x10m 图的 8%),且 drop/距离 恒等于 tan(3.8~3.9deg),
    // 即落差是随距离线性增长的"坡"签名,不是崖。
    // 改法:沿"本格 -> 车"方向收集洞【前】的有效 ground(距离,高度),线性拟合 h=a+b*d 后
    // 外推到本格距离当参考。公共的 d*tan(theta) 项被消掉,只剩真实台阶。
    // 离线验证(analysis/terrain_2026_09_03/cliff_fix2.py,同一批帧):
    //   水平面(原)      -> 764~813 格/帧误报
    //   洞口取单点参考   -> 564~614 格(不够:阴影有几米宽,坡在洞内继续下降)
    //   阈值随距离放宽   -> 0 格,但 4.5m 处 seed 门限变 1.21m,真台阶抓不到
    //   本改法(坡面外推) -> 0 格,残余落差中位仅 0.065~0.070m(门限 0.10);
    //                      同一片坡上人为叠 0.35m 真台阶仍报 406~481 格,灵敏度未丢
    cliff_ref_grade_enable_ =
      declare_parameter<bool>("cliff_ref_grade_enable", true);
    // 拟合用的有效 ground 样本数上限。15 个(约 1.5m 回溯)足够定出局部坡度又不会把
    // 远处别的地形结构拟合进来。
    cliff_ref_fit_cells_ = declare_parameter<int>("cliff_ref_fit_cells", 15);
    // 沿射线最大回溯距离(m)。走完仍凑不够样本就走兜底。
    cliff_ref_max_search_m_ =
      declare_parameter<double>("cliff_ref_max_search_m", 4.5);
    // 兜底/限幅角(度)。两个用途:
    //   1. 拟合不出来(洞前 ground 不足)时,参考面退回水平面 + d*tan(此角),
    //      保留一层防护而不是完全不设防;
    //   2. 给拟合出的坡度限幅 —— 最多把这么陡的地形算作"地形自身坡度",
    //      更陡的部分照旧当落差。取值与 slope_lethal_deg 对齐(12deg 以上的地形
    //      坡度链已经判 lethal 了,不需要悬崖判据再替它兜)。
    cliff_ref_fallback_deg_ =
      declare_parameter<double>("cliff_ref_fallback_deg", 12.0);

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
    // 近场地面外推：ground 未知时，优先沿"点 -> 车"方向收集洞外的有效 ground，
    // 线性拟合 h=a+b*d 外推出该点的地面高度（cliffRefHeight 同款数学，悬崖链已
    // 实测验证：坡地误报 749~795 格/帧清零、灵敏度不丢）。外推失败才退回
    // fallback_on_unknown_elevation 的脚底高度带。这是 1.5m 内盲区（雷达下视
    // 打不到地面）里"人贴近车"能被检出的关键 —— 不依赖水平面假设，跟随坡度。
    near_ground_extrapolate_ = declare_parameter<bool>("near_ground_extrapolate", true);

    // ---- 坡度 -> 分级代价 ---------------------------------------------------
    // slope <= free_deg           -> 代价 0（视为平地）
    // free_deg < slope < lethal   -> 线性升到 99（缓坡代价递增，同等条件下避开）
    // slope >= lethal_deg         -> 100（lethal，基本过不去）
    slope_enable_ = declare_parameter<bool>("slope_enable", true);
    slope_window_ = declare_parameter<int>("slope_window", 2);        // 中心差分步长(格)
    // 地形掩码阈值：把"物体"格子从坡度链里剔除，见 addTerrainMask 的说明。
    // <=0 关闭对应判据。
    // 0.30 = 两条链的交接高度: 更低的路沿 obstacle 链抓不到(被 obstacle_min_height
    // 滤掉), 必须留给需要长记忆的 terrain 链; 更高的人/灌木/车归 obstacle 链(带
    // raytrace 清除). 定成 0.15 会把 20 cm 路沿当物体剔除, 反而关掉了本要抓的东西.
    slope_object_height_ = declare_parameter<double>("slope_object_height", 0.30);
    slope_terrain_band_ = declare_parameter<double>("slope_terrain_band", 1.0);
    slope_free_deg_ = declare_parameter<double>("slope_free_deg", 5.0);
    slope_lethal_deg_ = declare_parameter<double>("slope_lethal_deg", 25.0);
    // 代价曲线弯曲度：cost = 1 + pow(t, gamma)*98, t=(slope-free)/(lethal-free)。
    // gamma=1 线性；gamma<1 使中等坡度代价更高（更强软避让，但不改 lethal 阈值）。
    slope_gamma_ = declare_parameter<double>("slope_gamma", 1.0);

    // ---- LaserScan 几何 -----------------------------------------------------
    angle_min_ = declare_parameter<double>("angle_min", -3.141592653589793);
    angle_max_ = declare_parameter<double>("angle_max", 3.141592653589793);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.008726646259971648);
    // 滤掉落进 footprint 的自车点(外接半径 0.39 m，取 0.45)。BaseObstacle 只看
    // 中心格所以之前无感；ObstacleFootprint 检查整圈，自车点会判死所有轨迹。
    range_min_ = declare_parameter<double>("range_min", 0.45);
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

    // 车体在 grid 系下的位置：z 给地形掩码的高度带判据用，xy 给"空格清除"的
    // 可信半径用。拿不到就退化（掩码只用物体判据、不发空格清除），不影响主路径。
    bool have_foot = false;
    double x_foot = 0.0, y_foot = 0.0, z_foot = 0.0;
    try {
      auto t_foot = tf_buffer_->lookupTransform(
        grid->getFrameId(), base_frame_, msg->header.stamp,
        tf2::durationFromSec(0.2));
      x_foot = t_foot.transform.translation.x;
      y_foot = t_foot.transform.translation.y;
      z_foot = t_foot.transform.translation.z;
      have_foot = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "foot TF %s<-%s failed (%s); 掩码只用物体判据, 且不发空格清除",
        grid->getFrameId().c_str(), base_frame_.c_str(), e.what());
    }

    addGroundLayer(*grid);
    // 独立于 slope_enable_:悬崖判据不依赖 slope 链,也不该被它的开关关掉。
    // 必须在 addSlopeCostLayer / publishCostCloud 之前:悬崖结果并入 /terrain_cost_cloud。
    if (cliff_enable_) { addCliffLayer(*grid, msg->header.stamp); }
    if (slope_enable_) {
      addTerrainMask(*grid, z_foot, have_foot);
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
        publishCostCloud(*grid, msg->header.stamp, x_foot, y_foot, have_foot);
      }
    }
  }

  // 把 slope_cost 层拍成 PointCloud2(x,y=格心, z=0, float cost=0..100), grid(odom) 系.
  //
  // 除了有效 slope_cost, 还会为"观测到是空的"格子发 cost=0. 这是必需的:
  // TerrainLayer 只遍历本点云里的点, 没出现的格子它一律不碰、旧值永久保留
  // (见 terrain_layer.cpp cloudCallback 的循环). 所以 slope_cost 为 NaN 的格子,
  // 只要在车附近的可信半径内, 就一律发 0 —— 两种 NaN 都算"看过了那儿没东西":
  //   * 连高程数据都没有            -> 射线扫过、什么都没打到;
  //   * 有高程但坡度 NaN(差分边界)  -> 该格被观测到了, 只是中心差分窗口压到
  //     数据边界算不出梯度, 这不构成"那里可能有障碍"的证据。
  // 后者曾被 skip(理由是"确实不知道地形"), 代价是这些格子永远不进点云, 于是
  // 之前写下的 lethal 永久留在 costmap 上: 人走开、高程图都清了, 那块还堵着。
  // 实测残留成片出现在"真空区与高程图交界"(1.5~2.0 m 环带 193 格) —— 交界格
  // 自己有高程而邻居没有, 差分必然 NaN, 正好落进当年被 skip 的那一类。
  // 漏判风险由另两条链兜住: 真障碍走 obstacle_layer(scan), 悬崖在下面优先判 100.
  //
  // 负障碍(下沉台阶/悬崖)不会因此变危险: 空洞边缘的上沿格子有数据、会照常算出
  // 陡坡判 lethal, 车要进空洞必须先穿过那圈 lethal.
  void publishCostCloud(
    const grid_map::GridMap & grid, const rclcpp::Time & stamp,
    double x_robot, double y_robot, bool have_robot)
  {
    const grid_map::Matrix & cost = grid["slope_cost"];
    const grid_map::Matrix & elev = grid[elevation_layer_];
    const grid_map::Size size = grid.getSize();
    const bool clear_empty = free_on_empty_ && have_robot;
    const double r2 = free_on_empty_radius_ * free_on_empty_radius_;

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

    // 悬崖(负障碍)并入本层 —— 语义上它就是地形的一部分,不该另开一层。
    const bool has_cliff = grid.exists("cliff");
    const grid_map::Matrix * cliffm = has_cliff ? &grid["cliff"] : nullptr;

    size_t n = 0;
    size_t n_empty = 0;
    size_t n_cliff = 0;
    for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      const float c = cost(idx(0), idx(1));
      float out_cost;
      grid_map::Position pos;
      const bool is_cliff =
        has_cliff && std::isfinite((*cliffm)(idx(0), idx(1))) && (*cliffm)(idx(0), idx(1)) > 0.5f;
      if (is_cliff) {
        // 悬崖优先于 slope_cost 与空格清除:它是"射线穿过、地面确实不在那里"的正面
        // 证据,而 free_on_empty 只是"没数据"。两者的可信度不在一个量级。
        // 这一分支同时替代了原先"悬崖格跳过 free 分支"的守卫 —— 跳过只是保留旧值,
        // 主动发 100 才能让 TerrainLayer 把它写成 LETHAL。
        out_cost = 100.0f;
        grid.getPosition(idx, pos);
        ++n_cliff;
      } else if (std::isfinite(c)) {
        out_cost = c;
        grid.getPosition(idx, pos);
      } else {
        if (!clear_empty) continue;
        grid.getPosition(idx, pos);
        const double dx = pos.x() - x_robot;
        const double dy = pos.y() - y_robot;
        if (dx * dx + dy * dy > r2) continue;
        out_cost = 0.0f;
        ++n_empty;
      }
      *ox = static_cast<float>(pos.x());
      *oy = static_cast<float>(pos.y());
      *oz = 0.0f;
      *oc = out_cost;
      ++ox; ++oy; ++oz; ++oc; ++n;
    }
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cost cloud: %zu 点 (空格清除 %zu, 悬崖 %zu)", n, n_empty, n_cliff);
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

  // 地形掩码: 标出"哪些格子是车能走的地形", 把物体格子从坡度链里剔除.
  //
  // 为什么必须有: slope 是 ground 层的梯度, 而人/灌木/墙这类竖直物体会在 ground 上
  // 造成 10~150 cm 的台阶; slope_window=3 时 span=0.6 m, 只要 16 cm 高差就判 15°
  // (lethal), 中心差分又把每个台阶摊到 ±window 格宽的带上. 于是物体边缘被当成陡坡
  // 写进 TerrainLayer, 而该层对"没重新观测到"的格子保留旧值, 人走开后那圈 lethal
  // 就永久留下了.
  // 实测(10x10 m 窗, res=0.1): 有效 slope 格里 63.7% >=15°, 而地形本身相邻格高差
  // 中位仅 0.4 cm —— 即坡度链当时基本在测物体边缘, 不是地形.
  // 动态障碍归 scan 链(有 raytrace 清除), 不该进这条持久化的地形代价链.
  // 负障碍(悬崖/下沉)层。四趟遍历,产出 "cliff"(1/NaN)与 "cliff_drop"(米/NaN)。
  //
  // 与 slope/step 的差分判据【互补而非替代】:那两条要求落差两侧都有地面,悬崖的
  // NaN 阴影(≈H*d/h_lidar)远宽于差分模板所以失效;本判据逐格独立,阴影越宽覆盖
  // 越多、约束越强 —— 方向正好相反。
  // 始终会落下 "cliff"/"cliff_drop" 两层(哪怕全 NaN),这样下游按层名取用不会因为
  // 某一帧缺层而失败。
  // 悬崖判据的参考地面高度:沿"本格 -> 车"方向,用洞【前】的有效 ground 拟合
  // 局部坡面 h = a + b*d(d = 离车水平距离),再外推到本格的 d。
  // 返回 NaN = 拟合不成立(洞前有效 ground 不足),调用方走兜底。
  // b 会被限幅到 ±tan(cliff_ref_fallback_deg):最多把这么陡的地形算作"地形自身坡度",
  // 更陡的部分照旧计入落差,避免一个跑偏的拟合把真台阶抹平。
  double cliffRefHeight(
    const grid_map::GridMap & grid, const grid_map::Matrix & ground,
    const grid_map::Index & cell, const grid_map::Index & foot,
    double x_foot, double y_foot, double d_cell) const
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double di = static_cast<double>(foot(0) - cell(0));
    const double dj = static_cast<double>(foot(1) - cell(1));
    const double len = std::hypot(di, dj);
    if (len < 1e-6) { return nan; }
    const double ui = di / len, uj = dj / len;
    const double res = std::max(grid.getResolution(), 1e-3);
    const int max_steps = std::max(1, static_cast<int>(cliff_ref_max_search_m_ / res));
    const int want = std::max(4, cliff_ref_fit_cells_);
    const grid_map::Size size = grid.getSize();

    int n = 0;
    double sd = 0.0, sdd = 0.0, sh = 0.0, sdh = 0.0;
    for (int step = 1; step <= max_steps && n < want; ++step) {
      const int pi = cell(0) + static_cast<int>(std::lround(ui * step));
      const int pj = cell(1) + static_cast<int>(std::lround(uj * step));
      if (pi < 0 || pi >= size(0) || pj < 0 || pj >= size(1)) { break; }
      const float gv = ground(pi, pj);
      if (!std::isfinite(gv)) { continue; }
      grid_map::Position gp;
      if (!grid.getPosition(grid_map::Index(pi, pj), gp)) { continue; }
      const double d = std::hypot(gp.x() - x_foot, gp.y() - y_foot);
      const double h = static_cast<double>(gv);
      ++n; sd += d; sdd += d * d; sh += h; sdh += d * h;
    }
    if (n < 4) { return nan; }
    const double den = n * sdd - sd * sd;
    if (std::abs(den) < 1e-9) { return nan; }   // 样本全挤在同一距离上,定不出坡度
    double b = (n * sdh - sd * sh) / den;
    const double b_lim = std::tan(cliff_ref_fallback_deg_ * M_PI / 180.0);
    b = std::clamp(b, -b_lim, b_lim);
    const double a = (sh - b * sd) / n;
    return a + b * d_cell;
  }

  void addCliffLayer(grid_map::GridMap & grid, const rclcpp::Time & stamp)
  {
    const auto t_begin = std::chrono::steady_clock::now();
    const grid_map::Size sz = grid.getSize();
    grid_map::Matrix nanlayer(sz(0), sz(1));
    nanlayer.setConstant(std::numeric_limits<float>::quiet_NaN());

    if (!grid.exists(cliff_ub_layer_) || !grid.exists(cliff_flag_layer_)) {
      grid.add("cliff", nanlayer);
      grid.add("cliff_drop", nanlayer);
      if (!cliff_warned_no_layer_) {
        cliff_warned_no_layer_ = true;
        RCLCPP_WARN(
          get_logger(),
          "高程图缺少 '%s'/'%s' 层,悬崖检测停用(退化为原有行为)。"
          "需要在 elevation_mapping 的 publishers.elevation_map_raw.layers 里加上这两层,"
          "并确认 use_only_above_for_upper_bound: false",
          cliff_ub_layer_.c_str(), cliff_flag_layer_.c_str());
      }
      return;
    }
    // 车体位姿:xy 给距离门限,z 给参考地面。
    // 【刻意不复用 gridCallback 里那次带时刻的 t_foot 查询】:那次若 extrapolation 失败
    // 会让 have_foot=false,整帧检测停摆(实测确实频繁发生)。而这里要的只是"车所在
    // 地面大致多高、车大致在哪",对时间精度要求很低,所以按 stamp 查失败就回退到
    // TimePointZero 取最新可用 —— 宁可用几十毫秒前的位姿,也不要整帧不检测。
    double x_foot = 0.0, y_foot = 0.0, z_foot = 0.0;
    bool have_foot = false;
    for (int attempt = 0; attempt < 2 && !have_foot; ++attempt) {
      try {
        auto tf = (attempt == 0)
          ? tf_buffer_->lookupTransform(
              grid.getFrameId(), base_frame_, stamp, tf2::durationFromSec(0.05))
          : tf_buffer_->lookupTransform(grid.getFrameId(), base_frame_, tf2::TimePointZero);
        x_foot = tf.transform.translation.x;
        y_foot = tf.transform.translation.y;
        z_foot = tf.transform.translation.z;
        have_foot = true;
      } catch (const std::exception &) {
      }
    }
    if (!have_foot) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "取不到 %s<-%s 的 TF(连最新可用也没有),悬崖检测本帧跳过",
        grid.getFrameId().c_str(), base_frame_.c_str());
      grid.add("cliff", nanlayer);
      grid.add("cliff_drop", nanlayer);
      return;
    }

    const grid_map::Matrix & ub = grid[cliff_ub_layer_];
    const grid_map::Matrix & flag = grid[cliff_flag_layer_];
    const grid_map::Matrix & elev = grid[elevation_layer_];
    const grid_map::Size size = grid.getSize();
    const int rows = size(0), cols = size(1);
    const double res = grid.getResolution();

    const double ref_flat = z_foot - cliff_base_height_;  // 水平参考面(兜底用),见构造函数
    const double r_min2 = cliff_min_range_ * cliff_min_range_;
    const double r_max2 = cliff_max_range_ * cliff_max_range_;
    // 坡面外推需要 ground 层(addGroundLayer 在本函数之前调用)。缺层就整体走兜底。
    const bool use_grade = cliff_ref_grade_enable_ && grid.exists("ground");
    const grid_map::Matrix & ground_ref = use_grade ? grid["ground"] : ub;
    grid_map::Index foot_idx(0, 0);
    const bool have_foot_idx =
      grid.getIndex(grid_map::Position(x_foot, y_foot), foot_idx);
    const double fallback_tan = std::tan(cliff_ref_fallback_deg_ * M_PI / 180.0);
    size_t n_ref_fallback = 0;

    // ---- Pass 1: 候选 + drop + 深种子 -------------------------------------
    std::vector<uint8_t> cand(static_cast<size_t>(rows) * cols, 0);
    std::vector<uint8_t> seed(static_cast<size_t>(rows) * cols, 0);
    grid_map::Matrix dropm(rows, cols);
    dropm.setConstant(std::numeric_limits<float>::quiet_NaN());
    size_t n_flag = 0;

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const float fv = flag(i, j);
        if (!std::isfinite(fv) || fv <= 0.5f) { continue; }
        ++n_flag;
        // ★ 必须要求本格【没有】有效高程 ★
        // custom_kernels.py:245-261 的第二个 visibility_cleanup 分支会在【本来有效】
        // 的格子上也写 is_upper_bound=1(条件 nmap_h > nz + 0.01 - ...),而两个分支在
        // 同一个 elementwise kernel 里、线程无序 —— 一帧内同一格可能同时
        // is_valid=1 且 is_upper_bound=1,并带着过时的低 upper_bound。
        // 少了这个守卫,真实可行驶地面会被标成悬崖(实测每帧约 206 格命中该竞态)。
        const float ev = elev(i, j);
        if (std::isfinite(ev)) { continue; }
        const float uv = ub(i, j);
        if (!std::isfinite(uv)) { continue; }

        grid_map::Position pos;
        if (!grid.getPosition(grid_map::Index(i, j), pos)) { continue; }
        const double dx = pos.x() - x_foot, dy = pos.y() - y_foot;
        const double d2 = dx * dx + dy * dy;
        if (d2 < r_min2 || d2 > r_max2) { continue; }

        // 参考面:优先"坡面外推"(消掉地形自身的 d*tan(theta)),失败退回
        // "水平面 + 随距离放宽的容差"。
        const double d_cell = std::sqrt(d2);
        double ref = std::numeric_limits<double>::quiet_NaN();
        if (use_grade && have_foot_idx) {
          ref = cliffRefHeight(
            grid, ground_ref, grid_map::Index(i, j), foot_idx, x_foot, y_foot, d_cell);
        }
        if (!std::isfinite(ref)) {
          ref = ref_flat - d_cell * fallback_tan;
          ++n_ref_fallback;
        }
        const double drop = ref - static_cast<double>(uv);
        dropm(i, j) = static_cast<float>(drop);   // 无论是否入选都记录,便于离线扫参
        // 双阈值:只有"确实低于参考地面 grow_thresh 以上"的格子才可参与连通域,
        // 否则盲区格(drop≈0 甚至为负)会把连通域一路连到车脚下。
        if (drop <= cliff_grow_thresh_) { continue; }
        cand[static_cast<size_t>(i) * cols + j] = 1;
        if (drop > cliff_depth_thresh_) { seed[static_cast<size_t>(i) * cols + j] = 1; }
      }
    }

    // enable_visibility_cleanup 被关掉会让整个检测器静默失效,这里主动发现它。
    if (n_flag == 0) {
      if (++cliff_all_zero_streak_ == 50) {
        RCLCPP_ERROR(
          get_logger(),
          "连续 50 帧 '%s' 全为 0 —— 悬崖检测拿不到任何信号。"
          "检查 elevation_mapping 的 enable_visibility_cleanup 是否为 true",
          cliff_flag_layer_.c_str());
      }
    } else {
      cliff_all_zero_streak_ = 0;
    }

    // ---- Pass 2: 连通域(4-邻接)-> seed-and-grow ---------------------------
    // 只有种子的话,lethal 只会落在阴影【远端】(可确认深度随离边缘距离线性增长),
    // 边缘本身永远标不上,车就可能"合法地"规划到前轮悬空。所以对整个连通域一起标记。
    // 分量天然有界:它就是遮挡阴影,而从未观测的格子 is_upper_bound=0、不在候选里,
    // 因此 flood-fill 不会漏进未知空间。
    std::vector<uint8_t> mark(static_cast<size_t>(rows) * cols, 0);
    std::vector<uint8_t> visited(static_cast<size_t>(rows) * cols, 0);
    std::vector<int> stack;
    std::vector<int> comp;
    for (int i0 = 0; i0 < rows; ++i0) {
      for (int j0 = 0; j0 < cols; ++j0) {
        const int start = i0 * cols + j0;
        if (!cand[start] || visited[start]) { continue; }
        stack.clear(); comp.clear();
        stack.push_back(start); visited[start] = 1;
        int n_seed = 0;
        while (!stack.empty()) {
          const int cur = stack.back(); stack.pop_back();
          comp.push_back(cur);
          if (seed[cur]) { ++n_seed; }
          const int ci = cur / cols, cj = cur % cols;
          const int nb[4][2] = {{ci - 1, cj}, {ci + 1, cj}, {ci, cj - 1}, {ci, cj + 1}};
          for (const auto & q : nb) {
            if (q[0] < 0 || q[0] >= rows || q[1] < 0 || q[1] >= cols) { continue; }
            const int ni = q[0] * cols + q[1];
            if (cand[ni] && !visited[ni]) { visited[ni] = 1; stack.push_back(ni); }
          }
        }
        if (n_seed >= cliff_min_seed_cells_ &&
          static_cast<int>(comp.size()) >= cliff_min_area_cells_)
        {
          for (const int c : comp) { mark[c] = 1; }
        }
      }
    }

    // ---- Pass 3: 时间确认(odom 系栅格上的命中计数,不随 grid 滚动漂移)-------
    grid_map::Matrix cliff(rows, cols);
    cliff.setConstant(std::numeric_limits<float>::quiet_NaN());
    size_t n_lethal = 0, n_cand_total = 0;
    for (size_t q = 0; q < cand.size(); ++q) { if (cand[q]) { ++n_cand_total; } }
    const double key_res = (res > 1e-6) ? res : 0.1;
    const int64_t purge_r = static_cast<int64_t>((cliff_max_range_ + 1.0) / key_res);
    const int64_t kx0 = static_cast<int64_t>(std::llround(x_foot / key_res));
    const int64_t ky0 = static_cast<int64_t>(std::llround(y_foot / key_res));

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const size_t idx = static_cast<size_t>(i) * cols + j;
        if (!cand[idx]) { continue; }
        grid_map::Position pos;
        if (!grid.getPosition(grid_map::Index(i, j), pos)) { continue; }
        const int64_t kx = static_cast<int64_t>(std::llround(pos.x() / key_res));
        const int64_t ky = static_cast<int64_t>(std::llround(pos.y() / key_res));
        const int64_t key = (kx << 32) ^ (ky & 0xffffffffLL);
        auto it = cliff_hits_.find(key);
        int hits = (it == cliff_hits_.end()) ? 0 : it->second;
        if (mark[idx]) {
          hits = std::min(hits + 1, 8);
        } else {
          hits = std::max(hits - 1, 0);
        }
        if (hits == 0) {
          if (it != cliff_hits_.end()) { cliff_hits_.erase(it); }
        } else if (it == cliff_hits_.end()) {
          cliff_hits_.emplace(key, static_cast<uint8_t>(hits));
        } else {
          it->second = static_cast<uint8_t>(hits);
        }
        if (hits >= cliff_confirm_frames_) { cliff(i, j) = 1.0f; ++n_lethal; }
      }
    }
    n_cand_last_ = n_cand_total; n_lethal_last_ = n_lethal;
    n_ref_fallback_last_ = n_ref_fallback;
    // 清理远离车的旧 key,避免 map 无界增长
    for (auto it = cliff_hits_.begin(); it != cliff_hits_.end(); ) {
      const int64_t kx = it->first >> 32;
      const int64_t ky = static_cast<int32_t>(it->first & 0xffffffffLL);
      if (std::llabs(kx - kx0) > purge_r || std::llabs(ky - ky0) > purge_r) {
        it = cliff_hits_.erase(it);
      } else {
        ++it;
      }
    }

    // ---- Pass 4: 落层(随 /terrain_grid_map 发出,便于 RViz 与离线扫参)------
    grid.add("cliff", cliff);
    grid.add("cliff_drop", dropm);

    // 耗时统计。本层的算力预算:5Hz 即 200ms/帧,目标 p95 < 20ms。
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();
    cliff_ms_sum_ += ms;
    cliff_ms_max_ = std::max(cliff_ms_max_, ms);
    ++cliff_ms_n_;
    if (cliff_ms_n_ >= 50) {
      RCLCPP_INFO(
        get_logger(),
        "悬崖层耗时: 近 %zu 帧 均值 %.2f ms 峰值 %.2f ms | 候选 %zu 格, lethal %zu 格"
        ", 参考面走兜底 %zu 格(坡面拟合失败)",
        cliff_ms_n_, cliff_ms_sum_ / cliff_ms_n_, cliff_ms_max_, n_cand_last_, n_lethal_last_,
        n_ref_fallback_last_);
      cliff_ms_sum_ = 0.0; cliff_ms_max_ = 0.0; cliff_ms_n_ = 0;
    }
  }

  void addTerrainMask(grid_map::GridMap & grid, double z_foot, bool have_foot) const
  {
    if (!grid.exists("ground") || !grid.exists(elevation_layer_)) return;
    const grid_map::Matrix & elev = grid[elevation_layer_];
    const grid_map::Matrix & g = grid["ground"];
    const grid_map::Size size = grid.getSize();

    grid_map::Matrix ok(size(0), size(1));
    ok.setZero();
    for (int i = 0; i < size(0); ++i) {
      for (int j = 0; j < size(1); ++j) {
        const float gv = g(i, j);
        if (!std::isfinite(gv)) continue;
        // 判据 1(物体): 该格高出局部地面太多 -> 是物体, 不是地形.
        const float ev = elev(i, j);
        if (slope_object_height_ > 0.0 && std::isfinite(ev) &&
          (ev - gv) > slope_object_height_)
        {
          continue;
        }
        // 判据 2(高度带): 物体核心区的 ground 会被物体自己顶起来, 此时
        // elevation-ground≈0, 判据 1 抓不到. 用"地面层相对脚底的高度"兜住
        // 花台/墙顶这类抬升结构. 真实斜坡在 10 m 窗内的高差远小于该带宽
        // (15° 走 5 m 也只升 1.34 m), 所以不会误杀坡道.
        if (have_foot && slope_terrain_band_ > 0.0 &&
          std::abs(static_cast<double>(gv) - z_foot) > slope_terrain_band_)
        {
          continue;
        }
        ok(i, j) = 1.0f;
      }
    }
    grid.add("terrain_ok", ok);
  }

  // 坡度层 = 地面层的空间梯度(中心差分), 单位: 度.
  // 只在中心与四个差分邻居都是地形(terrain_ok)时计算, 否则留 NaN —— 物体边缘
  // 不再产生假陡坡.
  void addSlopeLayer(grid_map::GridMap & grid) const
  {
    if (!grid.exists("ground")) return;
    const grid_map::Matrix & g = grid["ground"];
    const bool has_mask = grid.exists("terrain_ok");
    const grid_map::Matrix & mask =
      has_mask ? grid["terrain_ok"] : grid["ground"];  // has_mask=false 时不会被读
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
        // 差分模板里只要碰到非地形格, 这个坡度就不可信 -> 留 NaN.
        if (has_mask &&
          (mask(i, j) < 0.5f ||
          mask(i - w, j) < 0.5f || mask(i + w, j) < 0.5f ||
          mask(i, j - w) < 0.5f || mask(i, j + w) < 0.5f))
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

    // 点云系(body) -> base_link。这是雷达安装姿态,是 static TF,所以用 TimePointZero
    // 取最新即可 —— 【刻意不按 cloud_msg 的时刻去查】:带时刻的查询会引入第二个
    // extrapolation 失败点,而本函数里 TF 失败就得整帧放弃,那会把 scan 频率打对折。
    // 下面需要的 grid->base_link 由它与已经取到的 t_body_to_grid 复合得出,
    // 因此全函数只有一个 TF 失败点(t_body_to_grid),与改动前保持一致。
    geometry_msgs::msg::TransformStamped t_body_to_base;
    try {
      t_body_to_base = tf_buffer_->lookupTransform(
        base_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "static TF %s<-%s failed: %s",
        base_frame_.c_str(), cloud_msg->header.frame_id.c_str(), e.what());
      return;
    }

    // 脚底高度 z_foot = base_link 原点在 grid(odom) 下的 z,由上面两个变换复合反解,
    // 不再单独查一次 TF(理由同上)。x_foot/y_foot 同时解出,供近场地面外推用。
    const Eigen::Matrix4f m_base_to_grid =
      (toMatrix(t_body_to_base) * toMatrix(t_body_to_grid).inverse()).inverse();
    const bool have_foot = true;
    const double x_foot = static_cast<double>(m_base_to_grid(0, 3));
    const double y_foot = static_cast<double>(m_base_to_grid(1, 3));
    const double z_foot = static_cast<double>(m_base_to_grid(2, 3));
    // 近场外推的起点: 车脚下格。车不在 grid 内时外推整体禁用(退兜底)。
    grid_map::Index foot_idx(0, 0);
    const bool have_foot_idx =
      near_ground_extrapolate_ &&
      grid->getIndex(grid_map::Position(x_foot, y_foot), foot_idx);

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
      grid_map::Index idx;
      bool have_idx = false;
      const grid_map::Position pos(p.x, p.y);
      if (grid->isInside(pos)) {
        if (grid->getIndex(pos, idx)) {
          have_idx = true;
          ground = grid->at("ground", idx);
          have_ground = std::isfinite(ground);
        }
      }

      if (have_ground) {
        const double h = p.z - ground;              // 相对局部地面高度
        if (h <= obstacle_min_height_) continue;    // 贴地/地面 -> 丢
        if (h > obstacle_max_height_) continue;     // 过高(天花板等) -> 丢
      } else {
        // 地面未知: 优先坡面外推补一个地面参照(跟随坡度, 见构造函数的参数说明),
        // 外推失败才退回脚底高度带(平地够用). 都拿不到就直接丢.
        double g_ref = std::numeric_limits<double>::quiet_NaN();
        if (have_foot_idx && have_idx) {
          const double d_p = std::hypot(p.x - x_foot, p.y - y_foot);
          g_ref = cliffRefHeight(*grid, (*grid)["ground"], idx, foot_idx,
                                 x_foot, y_foot, d_p);
        }
        if (std::isfinite(g_ref)) {
          const double h = p.z - g_ref;             // 相对外推地面高度
          if (h <= obstacle_min_height_) continue;  // 与正常路径同一判据
          if (h > obstacle_max_height_) continue;
        } else if (fallback_on_unknown_ && have_foot) {
          const double h = p.z - z_foot;
          if (h < fallback_min_height_ || h > fallback_max_height_) continue;
        } else {
          continue;
        }
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

    // 重新投回车体系 base_link, 使距离/角度与实时车体位姿一致.
    //
    // 【必须是 base_link 而不是 body】。body 是 FAST-LIO 的 IMU/雷达系,相对 base_link 带
    // 着雷达的安装姿态 —— 本车是 Ry(+15deg)*Rz(+90deg)。绕 z 的 90deg 不影响平面的水平性,
    // 但那 15deg 俯仰会让 body 的 xy 平面相对地面倾斜 15deg。若在 body 系里扁平化:
    //   * 发出的 LaserScan 平面是倾斜的(在 RViz 里肉眼可见地斜插进地面);
    //   * range 是沿倾斜平面的投影而非真实水平距离(cos15deg -> 约 3.4% 偏差);
    //   * Nav2 的 obstacle_layer 会按这个倾斜平面做 raytrace,清障/标障都偏。
    // base_link 的 z 轴是竖直的(车体系),在它里面扁平化才等于"俯视投影",
    // 而且 scan 的角度也回到 base_link 的直觉方向(0 = 车头)。
    const Eigen::Matrix4f m_grid_to_base =
      toMatrix(t_body_to_base) * toMatrix(t_body_to_grid).inverse();
    pcl::PointCloud<pcl::PointXYZ> merged_base;
    pcl::transformPointCloud(merged_grid, merged_base, m_grid_to_base);

    // ---- Pass 2: 扁平化为 2D LaserScan -------------------------------------
    const size_t ranges_n =
      static_cast<size_t>(std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    sensor_msgs::msg::LaserScan scan;
    // stamp 沿用输入点云(与 TF 查询时刻一致),frame 换成 base_link —— 见上面的说明。
    scan.header.stamp = cloud_msg->header.stamp;
    scan.header.frame_id = base_frame_;
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.assign(ranges_n, std::numeric_limits<float>::infinity());

    pcl::PointCloud<pcl::PointXYZ> kept_base;
    if (publish_filtered_pcd_) kept_base.points.reserve(merged_base.points.size());
    for (const auto & p : merged_base.points) {
      insertPointToScan(p, scan, ranges_n);
      if (publish_filtered_pcd_ && std::isfinite(p.x) && std::isfinite(p.y)) {
        kept_base.points.push_back(p);
      }
    }

    scan_pub_->publish(scan);

    if (publish_filtered_pcd_ && !kept_base.points.empty() && pcd_pub_) {
      kept_base.width = static_cast<uint32_t>(kept_base.points.size());
      kept_base.height = 1;
      kept_base.is_dense = false;
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(kept_base, out);
      out.header.stamp = cloud_msg->header.stamp;
      out.header.frame_id = base_frame_;
      pcd_pub_->publish(out);
    }
  }

  // ---- params ----
  std::string cloud_topic_, elevation_topic_, elevation_layer_;
  std::string output_topic_, grid_output_topic_, slope_cost_topic_, base_frame_;
  bool cost_cloud_enable_;
  std::string cost_cloud_topic_;
  bool free_on_empty_;
  double free_on_empty_radius_;
  double ground_percentile_;
  int ground_radius_, ground_min_cells_;
  double obstacle_min_height_, obstacle_max_height_;
  bool fallback_on_unknown_;
  double fallback_min_height_, fallback_max_height_;
  bool near_ground_extrapolate_;
  bool slope_enable_;
  int slope_window_;
  double slope_object_height_, slope_terrain_band_;
  double slope_free_deg_, slope_lethal_deg_, slope_gamma_;
  double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
  bool accumulate_enable_;
  double accumulate_time_;
  int accumulate_max_frames_;
  bool publish_filtered_pcd_;

  // ---- 负障碍检测 ----
  bool cliff_enable_;
  std::string cliff_ub_layer_, cliff_flag_layer_;
  double cliff_depth_thresh_, cliff_base_height_, cliff_grow_thresh_;
  int cliff_min_seed_cells_, cliff_min_area_cells_, cliff_confirm_frames_;
  double cliff_min_range_, cliff_max_range_;
  bool cliff_ref_grade_enable_;
  int cliff_ref_fit_cells_;
  double cliff_ref_max_search_m_, cliff_ref_fallback_deg_;
  // odom 系 0.1m 栅格上的命中计数,用于时间确认。key 不随 grid 滚动漂移。
  std::unordered_map<int64_t, uint8_t> cliff_hits_;
  bool cliff_warned_no_layer_{false};
  double cliff_ms_sum_{0.0}, cliff_ms_max_{0.0};
  size_t cliff_ms_n_{0}, n_cand_last_{0}, n_lethal_last_{0}, n_ref_fallback_last_{0};
  size_t cliff_all_zero_streak_{0};

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
