// livox_multi_merger — 把 N 路 Livox CustomMsg 合并成单路,供只能吃一个 lid_topic 的
// FAST-LIO 消费。驱动(robot-sensors 的 livox_lidar_publisher)以 multi_topic=1 分别发
// /livox/front_lidar、/livox/back_lidar(未来还有 left/right),而 FAST-LIO 的
// common.lid_topic 是单个 string,只创建一个订阅。
//
// 设计约束(每一条都来自实测或 FAST-LIO 源码,改动前请先读完):
//
//  1) 输出坐标系 = primary(front)雷达系。FAST-LIO 的 mapping.extrinsic_T/extrinsic_R
//     是 lidar->IMU 外参,而它用的 IMU 是 /livox/front_imu。输出到 front 系,这组外参
//     无需任何改动。换成 base_link 就得同步改 extrinsic 和 indoor/outdoor 两套 TF。
//
//  2) per-lidar 过滤必须在外参变换【之前】。驱动对无回波点填的是精确 (0,0,0)
//     (实测 front 2.82%、back 4.18%)。单雷达时它们被 FAST-LIO 的 blind 滤掉;但若先
//     变换再滤,back 的零点会整体平移到它的安装位置(|t|~1.27m > blind 0.5),约 830
//     点/帧全部存活,在 costmap 里变成永久障碍物、在 SLAM 里变成虚假特征。
//
//  3) line 必须留在 [0, scan_line) 即 0..3。preprocess.cpp:169 的 avia_handler 会丢弃
//     line >= N_SCANS 的点,所以不能给 back 的 line 加偏移来标记来源。
//     (来源也无法靠 CustomMsg.lidar_id 区分 —— 它是 uint8,而 handle 是 IP 数值,
//      192.168.1.101/102 截断后都只剩 192。实测两路都是 192。)
//
//  4) offset_time 是 uint32 无符号。两路 timebase 不同,基准取所有参与帧的【最小】
//     timebase,才能保证重算后的 offset 非负。
//
//  5) 输出点必须按 offset_time 升序排序。laserMapping.cpp:412 用
//     points.back().curvature 推 lidar_end_time —— 取的是数组【最后一个】元素而不是
//     最大值。不排序则 IMU 积分区间截错,去畸变失效。
//
//  6) 输出 header.stamp 必须严格单调递增。laserMapping.cpp:324 一旦发现时间回退就
//     "lidar loop back, clear buffer",缓冲区被反复清空。
//
//  7) 两端 QoS 都必须 RELIABLE,且 depth 要够。
//     发布端:FAST-LIO 用 create_subscription(lid_topic, 20, ...) 即默认 QoS(reliable),
//     而 best_effort 的 publisher 与 reliable 的 subscriber 【不兼容】,会静默收不到数据。
//     订阅端:一开始图"传感器数据惯例"用了 SensorDataQoS(BEST_EFFORT, depth 5),结果大量
//     丢帧 —— CustomMsg 每帧约 400KB,DDS 必须分片,best_effort 丢任一分片即整帧丢失且
//     不重传。实测相邻收到帧的 stamp 间隔从应有的 100ms 恶化到 mean 193ms / max 7500ms
//     (平均丢一半,最长断了 75 帧),而 CPU 仅 14.9%、UDP rmem 已是 25MiB,都不是瓶颈。
//     驱动端是 RELIABLE 发布、FAST-LIO 以 RELIABLE 订阅且工作正常,说明这条链路本就
//     需要 reliable。故订阅端对齐 FAST-LIO:RELIABLE + depth 20。
//
//  8) rpy 约定:R = Rz(yaw) * Ry(pitch) * Rx(roll),与 tf2 的 static_transform_publisher
//     以及 scipy 的 Rotation.from_euler('xyz', ...) 一致。标定脚本输出可直接填进 yaml。
//
//  9) 【不能在收到 primary 帧时立即合并】。两路的 stamp 对得极准(实测偏差 |max| 1.1ms),
//     但【到达顺序】不保证:实测约 65% 的情况下 back 帧比同期的 front 帧后到。若在
//     primary 回调里立刻取 buffer 里"最近"的 back 帧,拿到的会是上一帧(差约 100ms),
//     超出容差从而被判失步 —— 早期版本就是这样在 100ms 内反复 降级/恢复 刷屏的。
//
//     因此改为「等待凑齐、超时降级」:primary 帧先入 pending 队列,每次任一路到达时
//     尝试 flush,凑齐即发(通常只需等几十毫秒),等待超过 max_wait_ms 才降级发出。
//     定时器兜底,保证某路彻底停流时 pending 不会卡住。
//
//     代价:输出比 primary 帧晚 0~max_wait_ms。这是多雷达合并的固有延迟,无法回避 ——
//     退而用"上一帧的 back"在空间和时间上都是错的(机器人已移动,且 offset_time 会
//     跨越 100ms 以上),比降级为单雷达更糟。节点会统计实际等待时间供调参。
//
//     输出严格按 primary 帧顺序,只处理 pending 队首、不跳帧,以保证约束 (6) 的单调性。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

namespace
{
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

// MID360 的 line 取值域。超出这个范围的点会被 FAST-LIO 的 avia_handler 丢弃,
// 这里同样丢弃并计入统计,以免静默地把点送进下游再被丢。
constexpr uint8_t kMaxLine = 4;

Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw)
{
  // R = Rz(yaw) * Ry(pitch) * Rx(roll) —— 见文件头约定 (8)
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

struct LidarSlot
{
  std::string name;
  std::string topic;
  bool is_primary{false};

  double min_range{0.5};
  double max_range{100.0};
  double min_range_sq{0.25};
  double max_range_sq{10000.0};

  // 该雷达坐标系 -> primary 雷达坐标系
  Eigen::Matrix3f rot{Eigen::Matrix3f::Identity()};
  Eigen::Vector3f trans{Eigen::Vector3f::Zero()};
  bool identity{false};  // 免去 primary 的逐点矩阵乘法

  rclcpp::Subscription<CustomMsg>::SharedPtr sub;
  std::deque<CustomMsg::ConstSharedPtr> buffer;  // 仅非 primary 使用
  rclcpp::Time last_rx{0, 0, RCL_ROS_TIME};
  bool ever_rx{false};
  bool degraded{false};  // 当前是否处于"缺这一路"状态,用于边沿触发日志

  // 到达节奏诊断。同时统计【到达间隔】(墙钟)和【stamp 间隔】(雷达自己的时间戳):
  //   两者都稳定  -> 链路健康
  //   stamp 稳、到达抖 -> 传输/DDS/调度抖动,雷达本身没问题
  //   stamp 也抖   -> 雷达出帧或时间戳本身有问题
  uint64_t prev_timebase{0};
  rclcpp::Time prev_rx{0, 0, RCL_ROS_TIME};
  bool has_prev{false};
  double rx_gap_sum{0.0}, rx_gap_max{0.0}, rx_gap_min{1e18};
  double tb_gap_sum{0.0}, tb_gap_max{0.0}, tb_gap_min{1e18};
  size_t gap_n{0};

  void noteArrival(const rclcpp::Time & t, uint64_t timebase)
  {
    if (has_prev) {
      const double rx_ms = (t - prev_rx).nanoseconds() / 1e6;
      const double tb_ms =
          (static_cast<int64_t>(timebase) - static_cast<int64_t>(prev_timebase)) / 1e6;
      rx_gap_sum += rx_ms;
      rx_gap_max = std::max(rx_gap_max, rx_ms);
      rx_gap_min = std::min(rx_gap_min, rx_ms);
      tb_gap_sum += tb_ms;
      tb_gap_max = std::max(tb_gap_max, tb_ms);
      tb_gap_min = std::min(tb_gap_min, tb_ms);
      ++gap_n;
    }
    prev_rx = t;
    prev_timebase = timebase;
    has_prev = true;
  }

  void resetGapStats()
  {
    rx_gap_sum = tb_gap_sum = 0.0;
    rx_gap_max = tb_gap_max = 0.0;
    rx_gap_min = tb_gap_min = 1e18;
    gap_n = 0;
  }
};

struct FrameStats
{
  size_t total{0};
  size_t dropped_zero{0};
  size_t dropped_range{0};
  size_t dropped_line{0};
  size_t dropped_body{0};
  size_t kept{0};
};

// 等待凑齐的 primary 帧。arrival 是它到达本节点的时刻,用于判断是否已等够 max_wait。
struct PendingPrimary
{
  CustomMsg::ConstSharedPtr msg;
  rclcpp::Time arrival;
};
}  // namespace

class LivoxMerger : public rclcpp::Node
{
public:
  LivoxMerger() : rclcpp::Node("livox_merger")
  {
    primary_name_ = declare_parameter<std::string>("primary", "front");
    auto names = declare_parameter<std::vector<std::string>>(
        "lidars", std::vector<std::string>{"front", "back"});

    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/fused_lidar");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");

    sync_tolerance_ns_ = static_cast<int64_t>(
        declare_parameter<double>("sync_tolerance_ms", 20.0) * 1e6);
    degrade_timeout_ns_ = static_cast<int64_t>(
        declare_parameter<double>("degrade_timeout_ms", 300.0) * 1e6);
    max_wait_ns_ = static_cast<int64_t>(
        declare_parameter<double>("max_wait_ms", 60.0) * 1e6);
    max_pending_ = static_cast<size_t>(declare_parameter<int>("max_pending", 8));
    if (max_pending_ < 1) { max_pending_ = 8; }
    {
      const int bs = declare_parameter<int>("buffer_size", 5);
      if (bs < 1) {
        RCLCPP_WARN(get_logger(), "buffer_size=%d 无效,回退为 5", bs);
        buffer_size_ = 5;
      } else {
        buffer_size_ = static_cast<size_t>(bs);
      }
    }

    body_filter_enable_ = declare_parameter<bool>("body_filter_enable", false);
    {
      auto bmin = declare_parameter<std::vector<double>>(
          "body_filter_min", std::vector<double>{-1.0, -0.5, -1.0});
      auto bmax = declare_parameter<std::vector<double>>(
          "body_filter_max", std::vector<double>{1.0, 0.5, 0.3});
      if (bmin.size() != 3 || bmax.size() != 3) {
        RCLCPP_ERROR(get_logger(),
                     "body_filter_min/max 必须是 3 元数组,已强制关闭 body filter");
        body_filter_enable_ = false;
      } else {
        body_min_ = Eigen::Vector3f(static_cast<float>(bmin[0]), static_cast<float>(bmin[1]),
                                    static_cast<float>(bmin[2]));
        body_max_ = Eigen::Vector3f(static_cast<float>(bmax[0]), static_cast<float>(bmax[1]),
                                    static_cast<float>(bmax[2]));
      }
    }

    stats_period_s_ = declare_parameter<double>("stats_period_s", 10.0);

    if (names.empty()) {
      RCLCPP_FATAL(get_logger(), "参数 lidars 为空,无事可做");
      throw std::runtime_error("empty lidars list");
    }

    // 逐路声明子参数。ROS2 会把 yaml 的嵌套结构展平成 "front.topic" 这种名字。
    for (const auto & name : names) {
      LidarSlot slot;
      slot.name = name;
      slot.is_primary = (name == primary_name_);
      slot.topic = declare_parameter<std::string>(name + ".topic", "/livox/" + name + "_lidar");
      slot.min_range = declare_parameter<double>(name + ".min_range", 0.5);
      slot.max_range = declare_parameter<double>(name + ".max_range", 100.0);
      slot.min_range_sq = slot.min_range * slot.min_range;
      slot.max_range_sq = slot.max_range * slot.max_range;

      auto xyz_rpy = declare_parameter<std::vector<double>>(
          name + ".xyz_rpy", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

      if (slot.is_primary) {
        // primary 定义了输出坐标系,其外参恒为单位。配了也忽略,并明确告知。
        slot.identity = true;
        bool nonzero = false;
        for (double v : xyz_rpy) {
          if (std::fabs(v) > 1e-12) { nonzero = true; break; }
        }
        if (nonzero) {
          RCLCPP_WARN(get_logger(),
                      "'%s' 是 primary,输出坐标系即它自身,配置的 xyz_rpy 被忽略",
                      name.c_str());
        }
      } else {
        if (xyz_rpy.size() != 6) {
          RCLCPP_FATAL(get_logger(), "'%s.xyz_rpy' 需要 6 个元素 [x y z roll pitch yaw],实际 %zu",
                       name.c_str(), xyz_rpy.size());
          throw std::runtime_error("bad xyz_rpy");
        }
        slot.trans = Eigen::Vector3f(static_cast<float>(xyz_rpy[0]),
                                     static_cast<float>(xyz_rpy[1]),
                                     static_cast<float>(xyz_rpy[2]));
        slot.rot = rpyToMatrix(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]).cast<float>();
        slot.identity = slot.trans.isZero(0.0f) &&
                        slot.rot.isApprox(Eigen::Matrix3f::Identity(), 1e-9f);
        if (slot.identity) {
          RCLCPP_WARN(get_logger(),
                      "'%s' 的外参是单位变换 —— 若尚未标定,合并结果会是两团错位的点云。"
                      "请先跑 calib_extrinsic.py 并把结果填进 lidar_extrinsics.yaml",
                      name.c_str());
        }
      }
      slots_.push_back(std::move(slot));
    }

    primary_idx_ = SIZE_MAX;
    for (size_t i = 0; i < slots_.size(); ++i) {
      if (slots_[i].is_primary) {
        if (primary_idx_ != SIZE_MAX) {
          RCLCPP_FATAL(get_logger(), "有多路被标记为 primary");
          throw std::runtime_error("multiple primaries");
        }
        primary_idx_ = i;
      }
    }
    if (primary_idx_ == SIZE_MAX) {
      RCLCPP_FATAL(get_logger(), "primary '%s' 不在 lidars 列表里", primary_name_.c_str());
      throw std::runtime_error("primary not found");
    }

    // 两端都用 RELIABLE + 足够的 depth,与 FAST-LIO 对齐。理由见文件头约定 (7) ——
    // best_effort 会让 400KB 的分片消息大量整帧丢失。
    const int qos_depth = declare_parameter<int>("qos_depth", 20);
    rclcpp::QoS io_qos(rclcpp::KeepLast(qos_depth > 0 ? static_cast<size_t>(qos_depth) : 20));
    io_qos.reliable();

    // 发布端可靠性开关。默认 reliable(FAST-LIO 用默认 QoS 订阅,必须如此)。
    // 仅在排查"RELIABLE publisher 被僵尸 reader 拖住"这类 DDS 故障时临时设 false ——
    // best_effort 不等确认,能把传输问题和 DDS 匹配问题区分开。
    // 注意:设成 false 后 FAST-LIO 将收不到任何数据(QoS 不兼容),不可用于生产。
    const bool pub_reliable = declare_parameter<bool>("publish_reliable", true);
    rclcpp::QoS pub_qos = io_qos;
    if (!pub_reliable) {
      pub_qos.best_effort();
      RCLCPP_WARN(get_logger(),
                  "publish_reliable=false:发布端为 BEST_EFFORT,仅供排查,"
                  "FAST-LIO 在此设置下收不到数据");
    }
    pub_ = create_publisher<CustomMsg>(output_topic_, pub_qos);

    for (size_t i = 0; i < slots_.size(); ++i) {
      auto & slot = slots_[i];
      if (slot.is_primary) {
        slot.sub = create_subscription<CustomMsg>(
            slot.topic, io_qos,
            [this](CustomMsg::ConstSharedPtr msg) { onPrimary(std::move(msg)); });
      } else {
        slot.sub = create_subscription<CustomMsg>(
            slot.topic, io_qos,
            [this, i](CustomMsg::ConstSharedPtr msg) { onSecondary(i, std::move(msg)); });
      }
      RCLCPP_INFO(get_logger(), "订阅 %s%s", slot.topic.c_str(),
                  slot.is_primary ? "  (primary,定义输出坐标系)" : "");
    }

    RCLCPP_INFO(get_logger(),
                "发布 %s (frame_id=%s, reliable)  sync_tol=%.1fms  max_wait=%.1fms  "
                "degrade_timeout=%.0fms",
                output_topic_.c_str(), output_frame_.c_str(), sync_tolerance_ns_ / 1e6,
                max_wait_ns_ / 1e6, degrade_timeout_ns_ / 1e6);

    // 兜底 flush:正常情况下由 onSecondary 触发,这个定时器只在某路彻底停流时起作用,
    // 保证 pending 队首不会一直卡着不发。
    flush_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tryFlush(); });

    last_stats_log_ = now();
  }

private:
  void onSecondary(size_t idx, CustomMsg::ConstSharedPtr msg)
  {
    auto & slot = slots_[idx];
    const auto t = now();
    slot.noteArrival(t, msg->timebase);
    slot.last_rx = t;
    slot.ever_rx = true;
    slot.buffer.push_back(std::move(msg));
    while (slot.buffer.size() > buffer_size_) {
      slot.buffer.pop_front();
    }
    // 这一路的新帧可能正是 pending 队首在等的,立刻试一次 —— 正常情况下合并就在这里发生。
    tryFlush();
  }

  void onPrimary(CustomMsg::ConstSharedPtr pmsg)
  {
    auto & primary = slots_[primary_idx_];
    const auto t = now();
    primary.noteArrival(t, pmsg->timebase);
    primary.last_rx = t;
    primary.ever_rx = true;

    pending_.push_back(PendingPrimary{std::move(pmsg), t});
    if (pending_.size() > max_pending_) {
      // 理论上不该发生(max_wait 会先把队首发出去)。真发生了说明 flush 被卡住,
      // 丢最老的一帧以免无界增长,并且必须告警 —— 这是逻辑异常而非正常降级。
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "pending 队列超过 %zu 帧,丢弃最老帧。max_wait_ms 是否设得过大?",
                            max_pending_);
      pending_.pop_front();
    }
    tryFlush();
  }

  // pickNearest 的结果。即使超容差也把最接近帧的时间差带出来 —— 降级时靠它区分成因:
  // 差值约等于一帧(100ms)说明同期帧还没到(到达延迟);差值只是略超容差才是真的
  // 时间同步问题。两者的现场处置完全不同,日志不能混为一谈。
  struct NearestResult
  {
    CustomMsg::ConstSharedPtr msg;  // 超容差或无候选时为 nullptr
    int64_t best_adt{std::numeric_limits<int64_t>::max()};
    bool had_candidate{false};
  };

  NearestResult pickNearest(const LidarSlot & slot, uint64_t t_ref) const
  {
    NearestResult r;
    CustomMsg::ConstSharedPtr best{nullptr};
    for (const auto & m : slot.buffer) {
      const int64_t adt =
          std::llabs(static_cast<int64_t>(m->timebase) - static_cast<int64_t>(t_ref));
      if (adt < r.best_adt) {
        r.best_adt = adt;
        best = m;
      }
    }
    r.had_candidate = static_cast<bool>(best);
    if (best && r.best_adt <= sync_tolerance_ns_) {
      r.msg = best;
    }
    return r;
  }

  // 尝试把 pending 队首发出去。只处理队首、绝不跳帧,以保证输出严格按 primary 帧顺序
  // (约束 6 的单调性依赖这一点)。
  void tryFlush()
  {
    while (!pending_.empty()) {
      const auto & head = pending_.front();
      const int64_t waited_ns = (now() - head.arrival).nanoseconds();

      std::vector<std::pair<size_t, CustomMsg::ConstSharedPtr>> parts;
      parts.reserve(slots_.size());
      parts.emplace_back(primary_idx_, head.msg);

      size_t missing = 0;
      std::vector<NearestResult> nearest(slots_.size());
      for (size_t i = 0; i < slots_.size(); ++i) {
        if (i == primary_idx_) { continue; }
        nearest[i] = pickNearest(slots_[i], head.msg->timebase);
        if (nearest[i].msg) {
          parts.emplace_back(i, nearest[i].msg);
        } else {
          ++missing;
        }
      }

      if (missing > 0 && waited_ns < max_wait_ns_) {
        return;  // 还没等够,继续等这一路到达
      }

      // 凑齐了,或者已等到上限 —— 无论如何都要发,不能把队列卡死
      publishMerged(parts, nearest, waited_ns, missing);
      pending_.pop_front();
    }
  }

  // 更新每路的降级状态并打印边沿日志(只在状态翻转时打一次,避免每帧刷屏)。
  // 日志按成因分四类,因为现场处置手段完全不同 —— 把到达延迟报成"PTP 失锁"会把人
  // 引去查时间同步,而真因可能只是网络抖动。
  void updateDegradeState(const std::vector<std::pair<size_t, CustomMsg::ConstSharedPtr>> & parts,
                          const std::vector<NearestResult> & nearest)
  {
    for (size_t i = 0; i < slots_.size(); ++i) {
      if (i == primary_idx_) { continue; }
      auto & slot = slots_[i];
      const bool present = std::any_of(parts.begin(), parts.end(),
                                       [i](const auto & pr) { return pr.first == i; });
      if (present) {
        if (slot.degraded) {
          slot.degraded = false;
          RCLCPP_INFO(get_logger(), "'%s' 恢复,重新参与合并", slot.name.c_str());
        }
        continue;
      }
      if (slot.degraded) { continue; }
      slot.degraded = true;

      const double silent_ms = (now() - slot.last_rx).nanoseconds() / 1e6;
      const auto & nr = nearest[i];

      if (!slot.ever_rx) {
        RCLCPP_WARN(get_logger(),
                    "'%s' (%s) 从未收到数据,降级为不含该路的合并。"
                    "检查该路是否在发布、话题名是否配错",
                    slot.name.c_str(), slot.topic.c_str());
      } else if (silent_ms > degrade_timeout_ns_ / 1e6) {
        RCLCPP_WARN(get_logger(),
                    "'%s' 已静默 %.0fms(超过 %.0fms 阈值),判定为掉线,降级为不含该路的合并。"
                    "检查雷达供电与网络",
                    slot.name.c_str(), silent_ms, degrade_timeout_ns_ / 1e6);
      } else if (!nr.had_candidate ||
                 nr.best_adt > sync_tolerance_ns_ + 50'000'000 /* 容差 + 50ms */) {
        // 最接近的帧差了约一整帧,说明同期帧根本还没到 —— 到达延迟,不是时间同步问题
        RCLCPP_WARN(get_logger(),
                    "'%s' 等待 %.1fms 后同期帧仍未到达(最近帧差 %.1fms,约一整帧),"
                    "降级为不含该路的合并。这是到达延迟而非时间同步问题;"
                    "偶发可忽略,频繁出现则加大 max_wait_ms",
                    slot.name.c_str(), max_wait_ns_ / 1e6,
                    nr.had_candidate ? nr.best_adt / 1e6 : -1.0);
      } else {
        // 帧到了,但 stamp 差略微超出容差 —— 这才是真的时间同步问题
        RCLCPP_WARN(get_logger(),
                    "'%s' 的帧已到达但 stamp 差 %.2fms 超出容差 %.1fms,降级为不含该路的合并。"
                    "这是时间同步问题:检查 PTP 是否失锁,或放宽 sync_tolerance_ms",
                    slot.name.c_str(), nr.best_adt / 1e6, sync_tolerance_ns_ / 1e6);
      }
    }
  }

  void publishMerged(const std::vector<std::pair<size_t, CustomMsg::ConstSharedPtr>> & parts,
                     const std::vector<NearestResult> & nearest, int64_t waited_ns, size_t missing)
  {
    updateDegradeState(parts, nearest);

    // ---- 2. 基准时间取所有参与帧的最小 timebase,保证 offset_time 非负 ----
    uint64_t t_base = parts.front().second->timebase;
    for (const auto & [idx, m] : parts) {
      (void)idx;
      t_base = std::min(t_base, m->timebase);
    }

    // ---- 3. 单调性保护:时间回退就丢帧 ----
    if (has_published_ && t_base <= last_timebase_) {
      ++dropped_nonmonotonic_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "输出时间未前进(t_base=%lu <= 上一帧 %lu),丢弃该帧以免 FAST-LIO 清缓冲。"
          "累计丢弃 %lu 帧",
          static_cast<unsigned long>(t_base), static_cast<unsigned long>(last_timebase_),
          static_cast<unsigned long>(dropped_nonmonotonic_));
      return;
    }

    // ---- 4. 逐路过滤 + 变换 ----
    size_t reserve = 0;
    for (const auto & [idx, m] : parts) {
      (void)idx;
      reserve += m->points.size();
    }

    auto out = std::make_unique<CustomMsg>();
    out->points.reserve(reserve);

    FrameStats st;
    for (const auto & [idx, m] : parts) {
      const auto & slot = slots_[idx];
      // 该路每个点的绝对时间 = m->timebase + p.offset_time,重算到 t_base 基准。
      // t_base <= m->timebase 恒成立,故 base_shift >= 0,加到 offset_time 上不会下溢。
      const uint64_t base_shift = m->timebase - t_base;

      for (const auto & p : m->points) {
        ++st.total;

        // (a) 无回波填充点:驱动对无回波填的是精确 (0,0,0)。必须在【自身坐标系】剔除,
        //     否则变换后会平移到该雷达的安装位置并存活下来。顺带挡掉 NaN/Inf。
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          ++st.dropped_zero;
          continue;
        }
        const float x = p.x, y = p.y, z = p.z;
        const double r_sq = static_cast<double>(x) * x + static_cast<double>(y) * y +
                            static_cast<double>(z) * z;
        if (r_sq == 0.0) {
          ++st.dropped_zero;
          continue;
        }

        // (b) per-lidar blind / 最远距离,在自身坐标系里判断才有物理意义
        if (r_sq < slot.min_range_sq || r_sq > slot.max_range_sq) {
          ++st.dropped_range;
          continue;
        }

        // (c) line 必须落在 avia_handler 接受的范围内,否则下游照样丢
        if (p.line >= kMaxLine) {
          ++st.dropped_line;
          continue;
        }

        // (d) 外参变换到 primary 系
        Eigen::Vector3f v(x, y, z);
        if (!slot.identity) {
          v = slot.rot * v + slot.trans;
        }

        // (e) 可选车身框剔除(在 primary 系)。实测车体落在 MID360 下方 FOV 盲区,
        //     近场 r∈[0.3,1.2) 是空的,所以默认关闭。
        if (body_filter_enable_ && (v.x() >= body_min_.x() && v.x() <= body_max_.x() &&
                                    v.y() >= body_min_.y() && v.y() <= body_max_.y() &&
                                    v.z() >= body_min_.z() && v.z() <= body_max_.z())) {
          ++st.dropped_body;
          continue;
        }

        CustomPoint q;
        q.x = v.x();
        q.y = v.y();
        q.z = v.z();
        q.reflectivity = p.reflectivity;
        q.tag = p.tag;
        q.line = p.line;  // 保持原值,不加来源偏移(见文件头约定 3)
        q.offset_time = static_cast<uint32_t>(base_shift + p.offset_time);
        out->points.push_back(q);
        ++st.kept;
      }
    }

    // ---- 5. 按 offset_time 升序:FAST-LIO 靠 points.back() 推 lidar_end_time ----
    std::sort(out->points.begin(), out->points.end(),
              [](const CustomPoint & a, const CustomPoint & b) {
                return a.offset_time < b.offset_time;
              });

    // ---- 6. 组装并发布 ----
    out->header.stamp = rclcpp::Time(static_cast<int64_t>(t_base), RCL_ROS_TIME);
    out->header.frame_id = output_frame_;
    out->timebase = t_base;
    out->point_num = static_cast<uint32_t>(out->points.size());
    out->lidar_id = 0;  // 该字段无来源区分能力(实测各路都是 192),不赋予语义
    out->rsvd = {0, 0, 0};

    last_timebase_ = t_base;
    has_published_ = true;

    // 长期匹配到 0 个订阅者时提醒一次。正常情况下 FAST-LIO 会订阅本话题;若它在跑却仍
    // 为 0,说明数据没有目的地(publish() 不会因此报错,静默得很)。
    ++published_frames_;
    if (pub_->get_subscription_count() == 0) {
      // 约 30s(10Hz)持续无订阅者才提醒,避免启动阶段误报
      if (++zero_sub_streak_ == 300) {
        RCLCPP_WARN(get_logger(), "已发布 %zu 帧,但 %s 始终没有订阅者",
                    published_frames_, output_topic_.c_str());
      }
    } else {
      if (zero_sub_streak_ >= 300) {
        RCLCPP_INFO(get_logger(), "订阅者已接入(%zu 个)",
                    pub_->get_subscription_count());
      }
      zero_sub_streak_ = 0;
    }

    pub_->publish(std::move(out));

    // 已经用过的、比本帧更早的 secondary 帧不会再被匹配到,清掉以免 buffer 里堆陈旧数据
    // (陈旧帧本身会被容差挡掉,但留着毫无用处,还会拖慢 pickNearest 的线性扫描)。
    for (size_t i = 0; i < slots_.size(); ++i) {
      if (i == primary_idx_) { continue; }
      auto & buf = slots_[i].buffer;
      while (!buf.empty() &&
             static_cast<int64_t>(buf.front()->timebase) <
                 static_cast<int64_t>(t_base) - sync_tolerance_ns_) {
        buf.pop_front();
      }
    }

    accumStats(st, parts.size(), waited_ns, missing);
  }

  void accumStats(const FrameStats & st, size_t n_parts, int64_t waited_ns, size_t missing)
  {
    acc_.total += st.total;
    acc_.dropped_zero += st.dropped_zero;
    acc_.dropped_range += st.dropped_range;
    acc_.dropped_line += st.dropped_line;
    acc_.dropped_body += st.dropped_body;
    acc_.kept += st.kept;
    ++acc_frames_;
    acc_parts_ += static_cast<double>(n_parts);

    const double waited_ms = waited_ns / 1e6;
    acc_wait_sum_ms_ += waited_ms;
    acc_wait_max_ms_ = std::max(acc_wait_max_ms_, waited_ms);
    if (missing > 0) { ++acc_incomplete_; }

    if (stats_period_s_ <= 0.0) { return; }
    const auto t = now();
    if ((t - last_stats_log_).seconds() < stats_period_s_) { return; }
    last_stats_log_ = t;

    const double f = static_cast<double>(acc_frames_);
    RCLCPP_INFO(get_logger(),
                "近 %zu 帧: 平均 %.2f 路/帧, 入 %.0f 点 -> 出 %.0f 点 "
                "(零点 %.0f, 距离 %.0f, line %.0f, 车身 %.0f) | "
                "凑齐等待 均值 %.1fms 峰值 %.1fms | 不完整 %zu 帧",
                acc_frames_, acc_parts_ / f, acc_.total / f, acc_.kept / f,
                acc_.dropped_zero / f, acc_.dropped_range / f, acc_.dropped_line / f,
                acc_.dropped_body / f, acc_wait_sum_ms_ / f, acc_wait_max_ms_,
                acc_incomplete_);

    // 逐路的到达节奏。stamp 间隔稳而到达间隔抖 => 传输/调度抖动,不是雷达的问题。
    for (auto & slot : slots_) {
      if (slot.gap_n == 0) { continue; }
      const double n = static_cast<double>(slot.gap_n);
      RCLCPP_INFO(get_logger(),
                  "  [%s] 到达间隔 mean=%.1f min=%.1f max=%.1f ms | "
                  "stamp 间隔 mean=%.1f min=%.1f max=%.1f ms  (n=%zu)",
                  slot.name.c_str(), slot.rx_gap_sum / n, slot.rx_gap_min, slot.rx_gap_max,
                  slot.tb_gap_sum / n, slot.tb_gap_min, slot.tb_gap_max, slot.gap_n);
      slot.resetGapStats();
    }

    // 只有不完整率真的偏高时才建议调参。长尾上偶发几帧降级是无害的(那一帧点云依然正确,
    // 只是少了第二路视野),为此加大 max_wait 会抬高所有帧的最坏延迟,不值得。
    const double incomplete_ratio = static_cast<double>(acc_incomplete_) / f;
    if (incomplete_ratio > 0.05) {
      RCLCPP_WARN(get_logger(),
                  "不完整帧占比 %.1f%%(%zu/%zu),等待峰值 %.1fms / 上限 %.1fms。"
                  "若持续如此,建议加大 max_wait_ms",
                  incomplete_ratio * 100.0, acc_incomplete_, acc_frames_, acc_wait_max_ms_,
                  max_wait_ns_ / 1e6);
    }

    acc_ = FrameStats{};
    acc_frames_ = 0;
    acc_parts_ = 0.0;
    acc_wait_sum_ms_ = 0.0;
    acc_wait_max_ms_ = 0.0;
    acc_incomplete_ = 0;
  }

  std::string primary_name_, output_topic_, output_frame_;
  int64_t sync_tolerance_ns_{20'000'000};
  int64_t degrade_timeout_ns_{300'000'000};
  int64_t max_wait_ns_{60'000'000};
  size_t buffer_size_{5};
  size_t max_pending_{8};

  bool body_filter_enable_{false};
  Eigen::Vector3f body_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f body_max_{Eigen::Vector3f::Zero()};

  std::vector<LidarSlot> slots_;
  size_t primary_idx_{SIZE_MAX};

  rclcpp::Publisher<CustomMsg>::SharedPtr pub_;

  // 等待凑齐的 primary 帧队列。严格 FIFO,只处理队首,保证输出顺序 = primary 帧顺序。
  std::deque<PendingPrimary> pending_;
  rclcpp::TimerBase::SharedPtr flush_timer_;

  uint64_t last_timebase_{0};
  bool has_published_{false};
  uint64_t dropped_nonmonotonic_{0};
  size_t published_frames_{0};
  size_t zero_sub_streak_{0};

  double stats_period_s_{10.0};
  rclcpp::Time last_stats_log_;
  FrameStats acc_;
  size_t acc_frames_{0};
  double acc_parts_{0.0};
  double acc_wait_sum_ms_{0.0};
  double acc_wait_max_ms_{0.0};
  size_t acc_incomplete_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LivoxMerger>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("livox_merger"), "启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
