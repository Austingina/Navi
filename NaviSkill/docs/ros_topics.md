# NaviSkill ROS 接口说明（Topic / Action / Service / TF）

本文档概括 NaviSkill 在运行时**订阅、监听、发布**的 ROS2 接口，以及它们的消息类型与用途。

绝大部分接口集中在导航节点 `NavNode`（`nav_node.py`）中创建；可视化进程
（`gl_viewer/ros_bridge.py`，仅 `server.py -v` 时启用）另有一路按需的 LiDAR 订阅。

> 说明：相机话题名可通过 `--image_topic` 配置，默认
> `/hdas/camera_head/left_raw/image_raw_color/compressed`；LiDAR 可视化话题默认
> `/cloud_registered`。其余话题名当前为硬编码。

---

## 1. 订阅 / 监听的 Topic（`NavNode`）

| Topic | 消息类型 | 频率/QoS | 用途 |
|-------|----------|----------|------|
| `/map` | `nav_msgs/OccupancyGrid` | depth 10 | 接收占据栅格地图，提取地图边界（`map_bounds`），缓存完整地图消息。 |
| `/hdas/camera_head/left_raw/image_raw_color/compressed`（可配置） | `sensor_msgs/CompressedImage` | depth 10 | VPR 引导/重定位用的相机帧。订阅句柄常驻不销毁，由 `_camera_enabled` 标志门控是否真正处理帧（引用计数 enable/disable）。 |
| `/plan` | `nav_msgs/Path` | depth 10 | Nav2 全局规划路径。下发新目标后 planner 会先清空再发新路径，配合 `_plan_epoch` 判断路径是否对应当前目标。 |
| `/motion_control/chassis_speed` | `geometry_msgs/TwistStamped` | depth 10 | 底盘实际速度反馈，用于**卡住检测**（下发了 cmd_vel 但底盘长时间不动则判定卡死）。 |
| `/odom` | `nav_msgs/Odometry` | depth 10（节流 ~10Hz） | chassis_ekf 输出的局部里程，平滑且无 VPR 重定位跳变。用于 VPR 共识采集的**运动门控**（相邻采样帧之间要求足够位移/转角）。 |
| `/elevator_moving` | `std_msgs/Bool` | depth 10 | FAST-LIO 通告电梯升降是否进行中。升降期间暂停所有 `cmd_vel` 与 Nav2 目标，结束后再恢复并重发 `last_goal`。 |
| `/cloud_registered`（可配置，按需） | `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/CustomMsg` | depth 5 | **仅可视化用**，由 `enable_lidar_subscription_for_viz()` 在开启 viz 时创建。回调只缓存原始消息（O(1)），numpy 解码延迟到 viz 线程，避免占用 rclpy 线程 GIL 拖慢定位。 |
| `/livox/lidar`（按需，可选） | `sensor_msgs/PointCloud2`（xfer_format=0）或 `livox_ros_driver2/CustomMsg`（xfer_format=1） | depth 5 | Livox MID360 **原始传感器系点云**。可作为可视化 LiDAR 源替代 `/cloud_registered`（`--viz_lidar_topic /livox/lidar`，需 `lidar→base_link` 外参）。同时也是 FAST-LIO 建图/定位的点云输入。 |
| `/livox/imu` | `sensor_msgs/Imu` | depth 10 | Livox 内置 IMU。由 **FAST-LIO 里程计**（`common.imu_topic`）与 **chassis_ekf** 节点订阅融合（提供 yaw rate）。属于 NaviSkill 之下的 SLAM/里程计层输入，`NavNode` 不直接订阅。 |

> 注：`/livox/lidar` 与 `/livox/imu` 是 Livox 驱动发布的**原始传感器话题**。在 NaviSkill 中
> `/livox/lidar` 仅作为可视化点云的可选源；二者主要由底层的 FAST-LIO（`laserOdometry3dbbs`）
> 与 `chassis_ekf_node` 消费，用于产出 `map→odom` / `odom→base_link` 的 TF 与 `/odom`。

---

## 2. 发布的 Topic（`NavNode`）

| Topic | 消息类型 | 用途 |
|-------|----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制输出（PID 精调阶段：先旋转对齐再平移逼近目标）。 |
| `/change_map` | `std_msgs/String` | 切换地图，发布新地图文件路径；由 `/grid_map_pcl_loader_node` 与 `/laser_mapping` 接收处理。 |
| `/pose_in_target` | `geometry_msgs/PoseStamped` | 切换地图后，向新地图发布机器人的初始位姿（用于重定位/坐标衔接）。 |

---

## 3. Action Client

| Action | 类型 | 用途 |
|--------|------|------|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 向 Nav2 下发导航目标点；通过 goal handle 跟踪 `accepted/active/result`，结合 `GoalStatus` 判断到达/中止/取消。 |

---

## 4. TF 监听（`tf2_ros.TransformListener`）

| 变换 | 用途 |
|------|------|
| `map` → `base_link` | 10Hz 定时查询，获取机器人在地图中的实时位姿（`update_robot_pose`）。 |
| `odom` → `base_link` | 作为 TF **新鲜度**指标：由底盘里程 ~50–100Hz 广播，比合成 stamp 更能反映定位是否实时（合成 `map→base_link` 的 stamp 受 FAST-LIO 低频广播 `map→odom` 影响而偏旧）。查询失败时回退到合成 stamp。 |

TF 新鲜度用于判断定位是否丢失：连续异常或 stamp 落后超过阈值（`_tf_stale_fail_sec = 10s`）则将 `tf_ready` 置 false。

---

## 5. Service Client（按需调用，带缓存复用）

服务 client 通过 `_get_cached_client((srv_type, srv_name))` 缓存复用，避免频繁
`create_client` 造成的 DDS endpoint 注册/销毁抖动（换图轮询时尤其明显）。

| Service | 类型 | 用途 |
|---------|------|------|
| `/grid_map_pcl_loader_node/get_parameters` | `rcl_interfaces/srv/GetParameters` | 读取 `map_file_path` 参数，得到当前加载的地图路径。 |
| `/elevator_mode` | `std_srvs/srv/SetBool` | 进/出电梯模式：进梯点锁定 Z（`enter_elevator`），出梯点解锁 Z（`exit_elevator`）。 |
| `{node_name}/set_parameters` | `rcl_interfaces/srv/SetParameters` | 动态设置远程节点参数（如 `/controller_server` 的 `general_goal_checker.xy_goal_tolerance`）。支持 bool/int/float/str。 |

---

## 6. 消息/服务类型导入一览

来自 `nav_node.py` 顶部：

- `geometry_msgs.msg`: `PoseStamped`, `Twist`, `TwistStamped`, `TransformStamped`
- `nav_msgs.msg`: `OccupancyGrid`, `Odometry`, `Path`
- `sensor_msgs.msg`: `CompressedImage`（及可视化用的 `PointCloud2`）
- `std_msgs.msg`: `String`, `Bool`
- `std_srvs.srv`: `SetBool`
- `rcl_interfaces.srv`: `GetParameters`, `SetParameters`
- `rcl_interfaces.msg`: `Parameter`, `ParameterValue`, `ParameterType`
- `action_msgs.msg`: `GoalStatus`, `GoalStatusArray`
- `nav2_msgs.action`: `NavigateToPose`
- `livox_ros_driver2/livox_ros_driver`（可选，可视化 CustomMsg 路径）

---

## 7. 可视化进程（`gl_viewer/ros_bridge.py`）

仅在 `server.py -v` 启动 GL 可视化时运行，独立于 `NavNode`：

| Topic | 消息类型 | QoS | 用途 |
|-------|----------|-----|------|
| LiDAR（默认 `/cloud_registered`） | `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/CustomMsg` | BEST_EFFORT / KEEP_LAST depth 1，按 `_lidar_min_interval` 节流 | 自动探测发布者消息类型后订阅，用于 3D 点云渲染。 |

该进程还会复用 `NavNode` 的 TF 缓冲做静态外参查询（如 `base_link → lidar`）。
