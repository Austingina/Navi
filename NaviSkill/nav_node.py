# ROS导航节点类
import rclpy  # ROS2 Python客户端库
from rclpy.node import Node  # ROS节点基类
from rcl_interfaces.srv import GetParameters, SetParameters  # 参数服务
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from rclpy.executors import SingleThreadedExecutor  # 单线程执行器
from rclpy.parameter import Parameter  # 节点参数（用于 use_sim_time 仿真时钟）
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # 回调组（MultiThreadedExecutor 并行调度）
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped  # 位姿消息类型和速度控制消息类型
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # 占据网格、里程计、全局路径
from sensor_msgs.msg import CompressedImage  # 压缩图像（用于 VPR 引导）
from sensor_msgs.msg import NavSatFix, NavSatStatus  # GNSS 定位（室外 GPS 导航）
from std_msgs.msg import String, Bool  # 字符串消息类型 / 布尔消息类型
from std_srvs.srv import SetBool  # 用于 elevator_mode 服务
from action_msgs.msg import GoalStatus, GoalStatusArray  # Action状态消息
import logging  # 日志记录库
import threading  # 线程处理库
from collections import deque
from typing import Any, Dict, Optional, Tuple
import time
import math  # 数学库
import os  # 文件路径处理
import subprocess  # 用于播放音频
# 添加tf2相关导入
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

# 导入action相关库
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal


class NavNode(Node):
    """
    导航节点类

    功能：
    1. 发布导航目标点到ROS话题
    2. 订阅里程计数据获取当前位置
    3. 检测是否到达目标点

    check_goal_reached 三态常量（供 server 等调用方判断）：
    """
    GOAL_CHECK_REACHED = "reached"
    GOAL_CHECK_PENDING = "pending"
    GOAL_CHECK_FAILED = "failed"

    # 导航模式：室内点云 SLAM 定位导航 / 室外 GNSS(GPS) 导航
    NAV_MODE_SLAM = "slam"
    NAV_MODE_GPS = "gps"

    # 参考 campus_nav（route_planner.launch.py / route_graph.ENU）的默认 ENU datum，
    # 室外 GPS 目标点经纬度按此 datum 换算成 campus_map 平面坐标下发给路网规划器。
    DEFAULT_GPS_DATUM_LAT = 22.574357630323675
    DEFAULT_GPS_DATUM_LON = 113.93507078035577
    _EARTH_R = 6378137.0  # WGS84 平均半径（m），与 campus_nav.route_graph 保持一致

    def __init__(
        self,
        camera_image_topic: str = "/hdas/camera_head/left_raw/image_raw_color/compressed",
        *,
        gps_fix_topic: str = "/gps/fix",
        odometry_gps_topic: str = "/odometry/gps",
        campus_goal_topic: str = "campus_goal_pose",
        campus_frame: str = "campus_map",
        gps_datum_lat: float = DEFAULT_GPS_DATUM_LAT,
        gps_datum_lon: float = DEFAULT_GPS_DATUM_LON,
        initial_nav_mode: str = NAV_MODE_SLAM,
        use_sim_time: Optional[bool] = None,
    ):
        """初始化ROS节点和相关发布者/订阅者

        :param camera_image_topic: 用于 VPR 引导/重定位的相机 CompressedImage 话题，
                                    默认 ``/hdas/camera_head/left_raw/image_raw_color/compressed``
                                    （部署机 408 项目使用 hdas head 左目 raw 压缩图像）。
                                    注意这与 vpr_preprocess.py 解析离线 rosbag 时
                                    使用的 ``/camera_agent/img_left/compressed`` 不同；
                                    数据库本身只关心特征空间，不要求 live 话题与 bag
                                    话题同名，只要相机参数与去畸变配置一致即可。
        :param gps_fix_topic: 室外 GNSS 定位的 NavSatFix 话题（默认 ``/gps/fix``，
                              与 outdoor_localization_dammy.launch.py 的 navsat 输入一致）。
        :param odometry_gps_topic: navsat_transform 输出的 GPS 里程 ``/odometry/gps``。
        :param campus_goal_topic: campus_nav 路网规划器的目标点话题（PoseStamped）。
        :param campus_frame: campus_nav 使用的 ENU 平面 TF（默认 ``campus_map``）。
        :param gps_datum_lat/gps_datum_lon: 经纬度→campus_map 平面换算所用 ENU 原点，
                              必须与 campus_nav 的 datum 一致（否则目标点会错位）。
        :param initial_nav_mode: 初始导航模式 ``"slam"``（默认）或 ``"gps"``。
        :param use_sim_time: 是否使用仿真时钟（订阅 ``/clock``）。``None`` 时按环境变量
            ``NAVI_SIM_ARG`` 自动判定（非空即为仿真）。仿真下必须为 True，否则本节点用
            墙上时钟、而 TF 盖的是仿真时间戳，两者相差几十年会导致 TF 新鲜度恒定“超标”，
            进而 ``current_pose`` 永远刷不出、定位一直判为丢失。
        """
        # 先确保rclpy已经初始化
        if not rclpy.ok():
            rclpy.init()

        # 解析仿真时钟：未显式指定时按 NAVI_SIM_ARG 环境变量自动判定
        if use_sim_time is None:
            use_sim_time = bool(os.environ.get('NAVI_SIM_ARG', '').strip())

        # 调用父类构造函数，设置节点名称；仿真下通过 parameter_overrides 打开 use_sim_time，
        # 使 get_clock() 走 /clock 仿真时钟，与 TF 时间戳一致。
        super().__init__(
            'nav_node',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time)),
            ],
        )
        self.get_logger().info(f"nav_node use_sim_time={bool(use_sim_time)}")

        # ── 回调组：与 MultiThreadedExecutor(num_threads=3) 配合使用，
        # 使 TF 心跳 / 重量级消息 / call-queue 三组并行执行，互不阻塞。
        self._cb_group_timer = MutuallyExclusiveCallbackGroup()  # update_robot_pose 定时器
        self._cb_group_bulk  = MutuallyExclusiveCallbackGroup()  # map / camera / lidar / odom 等
        self._cb_group_queue = MutuallyExclusiveCallbackGroup()  # call-queue 定时器

        # 初始化执行器和线程属性
        self.executor = None
        self.ros_thread = None
        # 在__init__方法中添加停止定时器属性
        self.stop_timer = None
        
        self.map_bound = {
            "xmin": -10.0,
            "xmax": 10.0,
            "ymin": -10.0,
            "ymax": 10.0
        }
        self.map_received = False  # 标记是否已接收到地图
        self.current_map = None  # 保存完整的地图消息

        try:
            # 创建navigate_to_pose action client
            self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            # 创建tf监听器替换里程计订阅
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            # 创建定时器定期获取tf变换
            self.tf_timer = self.create_timer(0.1, self.update_robot_pose, callback_group=self._cb_group_timer)  # 10Hz更新频率

            # 创建速度控制发布者，发布到/cmd_vel_nav话题
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

            # 创建地图切换发布者，发布到/change_map话题
            self.change_map_pub = self.create_publisher(String, '/change_map', 10)

            # 创建切换地图后初始位姿发布者
            self.pose_in_target_pub = self.create_publisher(PoseStamped, '/pose_in_target', 10)

            # 创建地图订阅者，用于获取地图边界
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                10,
                callback_group=self._cb_group_bulk,
            )

            # 相机 CompressedImage：用于 VPR 引导/重定位。
            # 订阅句柄在 __init__ 时一次性创建，永不销毁，彻底消除
            # create/destroy_subscription 带来的 Destroyable 竞争。
            # _camera_enabled 标志控制是否真正处理帧（False 时回调立即 return）。
            # enable/disable_camera_subscription 通过引用计数翻转该标志。
            self.camera_image_topic = camera_image_topic
            self._latest_image_lock = threading.Lock()
            self._latest_image_bytes: Optional[bytes] = None
            self._latest_image_stamp_ns: Optional[int] = None
            self._latest_image_event = threading.Event()
            self._camera_enabled: bool = False  # 帧处理门控标志
            self._camera_sub_lock = threading.Lock()
            self._camera_sub_refcount = 0
            # 订阅句柄一次性创建，永不销毁
            self.camera_image_sub = self.create_subscription(
                CompressedImage,
                self.camera_image_topic,
                self._camera_image_callback,
                10,
                callback_group=self._cb_group_bulk,
            )
            # Nav2 全局路径（下发新 goal 后 planner 会先清空再发布新路径）
            self._plan_lock = threading.Lock()
            self._plan_epoch = 0
            self._latest_plan: Optional[Path] = None
            self.plan_sub = self.create_subscription(
                Path,
                '/plan',
                self._nav_plan_callback,
                10,
                callback_group=self._cb_group_bulk,
            )
            # 存储当前机器人位姿
            self.current_pose = None
            # 存储最后发布的目标点
            self.last_goal = None
            # 存储当前goal handle
            self.current_goal_handle = None
            # 存储导航状态 - 改为更简单的布尔状态
            self.nav_goal_reached = False
            self.nav_goal_active = False
            # 导航已终止且未成功到达（中止/取消/未接受/精调卡住等），供 check_goal_reached 与上层立即结束等待
            self._nav_terminal_message: Optional[str] = None
            # PID控制状态
            self.pid_control_active = False
            self.pid_rotation_phase = True  # True为旋转阶段，False为移动阶段
            
            # TF状态跟踪
            self.tf_ready = False  # TF是否可用（定位是否成功）
            # TransformException 路径下的连续失败计数（仅用于 lookup_transform 抛异常的情况）
            self.tf_fail_count = 0
            self.tf_fail_threshold = 3  # 连续异常多少次认为定位丢失
            # 查询成功但 transform.stamp 落后此时钟超过该秒数时，认为可能是缓冲内旧 TF（换图后 LIO 暂停广播时常如此）
            self._tf_stale_warn_sec = 1.5
            # 落后超过该秒数则直接判为定位失败（LIO 长时间未广播新 TF），独立于 tf_fail_count，直接将 tf_ready 置 false
            self._tf_stale_fail_sec = 10.0
            self._last_stale_tf_warn_mono = 0.0  # time.monotonic()，节流 stale 警告
            
            # 底盘速度反馈订阅（用于检测卡住状态）
            self.chassis_speed = None  # 当前底盘速度反馈
            self.chassis_speed_sub = self.create_subscription(
                TwistStamped,
                '/motion_control/chassis_speed',
                self.chassis_speed_callback,
                10,
                callback_group=self._cb_group_bulk,
            )

            # /odom 订阅：用于"运动门控"——VPR 共识采集时，相邻两帧之间要求
            # 机器人发生足够的位移或旋转（见 vpr_bootstrap.VprRelocator
            # 的 MIN_MOVE_M / MIN_ROT_DEG），否则就等下一周期再采。
            # /odom 是 chassis_ekf 节点输出的局部里程，平滑且没有 VPR
            # 重定位带来的 jump，正好适合做这种 incremental 距离/角度判断。
            # 单元为 (px, py, yaw_rad, stamp_sec)；GIL 保证 tuple 引用读写原子。
            self._latest_odom_pose: Optional[Tuple[float, float, float, float]] = None
            # /odom 节流：底盘通常 50-100Hz 广播，但消费方（VPR 运动门控）只关心
            # 累计位移/转角，无需每帧都进入回调。保留 ~10Hz 已足够，丢掉的帧只是
            # 在节省 ROS spin 线程的 CPU/GIL 占用。0.0 表示首帧必定通过。
            self._odom_min_keep_interval_sec = 0.1
            self._odom_last_keep_mono: float = 0.0
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self._odom_callback,
                10,
                callback_group=self._cb_group_bulk,
            )
            # 卡住检测相关变量
            self.stuck_check_threshold = 0.1  # 速度阈值，低于此值认为没有移动
            self.cmd_vel_sent = None  # 最近发送的速度命令
            self.stuck_count = 0  # 连续卡住计数
            self.stuck_count_threshold = 10  # 连续多少次认为卡住（约0.5秒@50ms周期）

            # 电梯升降状态：FAST-LIO 通过 /elevator_moving 通告升降是否进行中。
            # 升降期间需要暂停所有 cmd_vel 输出与 Nav2 目标，等到值变 false 再继续。
            self.elevator_moving = False
            # 是否因为电梯升降主动取消了当前 Nav2 目标，恢复后需要重发 last_goal
            self._elevator_paused_nav = False
            # 标记当前进行中的 cancel 是为电梯暂停而发起的，避免在 result 回调里设置 _nav_terminal_message
            self._elevator_cancel_in_flight = False
            # 电梯升降结束后给底盘一段唤醒宽限期，期间不做卡住检测，避免误把 chassis 重启过程判为卡死
            self._elevator_resume_grace_sec = 3.0
            self._elevator_resume_grace_until = 0.0  # time.monotonic() 截止时间，0 表示当前不在宽限期
            self.elevator_moving_sub = self.create_subscription(
                Bool,
                '/elevator_moving',
                self._elevator_moving_callback,
                10,
                callback_group=self._cb_group_bulk,
            )

            # ── 导航模式（SLAM 室内点云定位 / GPS 室外 GNSS）与室外导航支撑 ──────
            # 两种模式共用同一套 Nav2/TF 后端：
            #   - SLAM：FAST-LIO 对先验 PCD 地图重定位，map→base_link 由点云配准维持，
            #           目标点是地图语义点，走 publish_goal → navigate_to_pose。
            #   - GPS ：室外无先验地图，navsat_transform + map_odom_from_gps 用 GNSS
            #           融合出 map→odom→base_link；目标点是经纬度/路网点，交给
            #           campus_nav 路网规划器（campus_goal_pose）做道路级规划后驱动 Nav2。
            # nav_mode 只影响“定位是否就绪的判据”和“目标点如何下发”，不改变底层 Nav2。
            mode = initial_nav_mode if initial_nav_mode in (
                self.NAV_MODE_SLAM, self.NAV_MODE_GPS) else self.NAV_MODE_SLAM
            self.nav_mode = mode
            self._nav_mode_lock = threading.Lock()
            # /nav_mode：向其它节点通告当前导航模式（切换时发布，便于联动）
            self.nav_mode_pub = self.create_publisher(String, '/nav_mode', 10)
            # GNSS 新鲜度门限（秒）：超过此时长未更新的 fix 视为不可用
            self._gps_fresh_sec = 2.0

            # GNSS fix（NavSatFix）：室外定位可用性判据。缓存最新一帧（含 mono 时间）。
            self.gps_fix_topic = gps_fix_topic
            self._gps_lock = threading.Lock()
            self._latest_gps_fix: Optional[Dict[str, Any]] = None
            self._latest_gps_odom: Optional[Tuple[float, float, float]] = None  # (x, y, mono)
            # 切到 GPS 模式后复位；仅新室外栈的 navsat_transform 发布 /odometry/gps
            # 才会置回 True，作为“新室外栈已就绪”的判据（见 update_robot_pose 门控）。
            self._gps_odom_received = False
            self.gps_fix_sub = self.create_subscription(
                NavSatFix,
                gps_fix_topic,
                self._gps_fix_callback,
                10,
                callback_group=self._cb_group_bulk,
            )
            # navsat_transform 输出的 GPS 里程（map/odom 系），可选，用于健康度参考
            self.gps_odom_sub = self.create_subscription(
                Odometry,
                odometry_gps_topic,
                self._gps_odom_callback,
                10,
                callback_group=self._cb_group_bulk,
            )

            # campus_nav 路网规划器目标点：室外 GPS 模式下把经纬度/平面点作为 goal
            # 发到该话题，规划器会吸附到最近道路、A* 出道路级路线并分批喂 Nav2。
            self.campus_frame = campus_frame
            self.campus_goal_pub = self.create_publisher(
                PoseStamped, campus_goal_topic, 10)
            # 经纬度 ↔ campus_map 平面（等距圆柱投影，与 route_graph.ENU 完全一致）
            self.gps_datum_lat = float(gps_datum_lat)
            self.gps_datum_lon = float(gps_datum_lon)
            self._gps_datum_cos0 = math.cos(math.radians(self.gps_datum_lat))
            # 室外当前目标（campus_frame 下的 PoseStamped）及运行标志
            self._outdoor_goal: Optional[PoseStamped] = None
            self._outdoor_nav_active = False
            # 关联的语义点名称 + 到达判定距离阈值（供 wait_goal_reached 按 target 判定）。
            self._outdoor_goal_name: Optional[str] = None
            self._outdoor_goal_tol: float = 3.0
            # SLAM 目标到达判定容差：与室外对称，下发目标时设定、供 wait_goal_reached
            # 取回（不再在 wait 时传入）。xy 单位米、yaw 单位弧度。
            self._slam_goal_xy_tol: float = 0.1
            self._slam_goal_yaw_tol: float = 0.1

            # 音频文件路径
            self.audio_dir = os.path.join(os.path.dirname(__file__), 'audio')

            # Service client 缓存：避免 get_current_map_path / set_elevator_mode /
            # set_remote_parameter 每次调用都 create_client（DDS endpoint 注册
            # 与销毁开销不可忽视，换图轮询时尤其明显）。键为 (srv_type, srv_name)。
            self._svc_client_cache: Dict[Tuple[Any, str], Any] = {}
            self._svc_client_cache_lock = threading.Lock()

            self.get_logger().info("ROS节点初始化成功")
            # call queue for scheduling calls to be executed inside the ROS executor thread
            self._call_queue = deque()
            self._call_queue_lock = threading.Lock()
            # timer running in ROS thread will flush the queue
            self._queue_timer = self.create_timer(0.02, self._process_call_queue, callback_group=self._cb_group_queue)

            # ── 可视化相关的按需订阅（仅在 server.py -v 启动时由 visualization.py 触发）
            # 关键：回调里只缓存 raw msg + 元数据（O(1)），不在 rclpy 线程里做
            # Python 层的逐点 numpy 化——MID360 单帧 24k 点用 for-loop 会占数十毫秒
            # GIL，把 update_robot_pose 的 timer 拖到几秒后才能跑，导致 TF 时间戳
            # 持续落后。Python→numpy 解码由 viz worker 线程在 get_latest_lidar_xyz()
            # 里完成，慢一点最多只影响 viz 帧率，不会反过来影响定位。
            self._lidar_lock = threading.Lock()
            self._lidar_msg: Optional[Any] = None         # 最近一帧原始 ROS msg
            self._lidar_msg_kind: str = ""                 # "pc2" / "livox_custom"
            self._lidar_frame_id: str = ""
            self._lidar_stamp_ns: int = 0
            self._lidar_seq: int = 0                       # viz 用来判断是否有新帧
            self._lidar_sub = None
            self._lidar_topic: Optional[str] = None

            # 通告初始导航模式，便于其它节点（路网规划器/桥接等）联动
            self._publish_nav_mode()
            self.get_logger().info(f"当前导航模式: {self.nav_mode}")
        except Exception as e:
            self.get_logger().error(f"ROS节点初始化失败: {e}")
            raise

    # ─── 可视化辅助：按需订阅 LiDAR topic（PointCloud2 / Livox CustomMsg 均支持）
    # 推荐用 FAST_LIO 的 /cloud_registered（已在 world/odom 系），也支持原始 /livox/lidar
    def enable_lidar_subscription_for_viz(self, topic: str = "/cloud_registered") -> bool:
        """启用一个 LiDAR 订阅，仅缓存最新一帧用于可视化。

        自动识别话题类型：
          - sensor_msgs/PointCloud2           （xfer_format=0 时）
          - livox_ros_driver2_msgs/CustomMsg  （xfer_format=1 时；驱动 ROS2 包名）
          - livox_ros_driver2/CustomMsg       （某些发行版下的旧包名）

        返回 True 表示订阅成功；False 表示找不到话题/消息类型导入失败。
        """
        import numpy as np
        if self._lidar_sub is not None:
            return True  # 已经订阅
        # 查找话题实际类型（topic 可能尚未发布时返回空）
        try:
            type_pairs = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warning(f"enable_lidar_subscription_for_viz: 查询话题失败: {e}")
            type_pairs = []
        msg_type_name = None
        for t, types in type_pairs:
            if t == topic and types:
                msg_type_name = types[0]
                break
        # 找不到也允许：用户可能晚一点才启动 livox 驱动；先按 PointCloud2 预订阅
        if msg_type_name is None:
            self.get_logger().warning(
                f"viz: 话题 {topic} 当前未发布，先按 PointCloud2 兜底订阅；"
                f"如果驱动用 xfer_format=1 (CustomMsg)，请改 launch 或调 enable_lidar_subscription_for_viz 之前先启动驱动。"
            )
            msg_type_name = "sensor_msgs/msg/PointCloud2"

        def _store(msg, kind: str) -> None:
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            with self._lidar_lock:
                self._lidar_msg = msg
                self._lidar_msg_kind = kind
                self._lidar_frame_id = msg.header.frame_id or ""
                self._lidar_stamp_ns = stamp_ns
                self._lidar_seq += 1

        # PointCloud2 路径
        if msg_type_name.endswith("/PointCloud2"):
            from sensor_msgs.msg import PointCloud2

            def _cb(msg: PointCloud2):
                # 仅缓存 raw msg，不在此处做 numpy 化（避免占用 rclpy 线程 GIL）
                _store(msg, "pc2")

            self._lidar_sub = self.create_subscription(PointCloud2, topic, _cb, 5, callback_group=self._cb_group_bulk)
            self._lidar_topic = topic
            self.get_logger().info(f"viz: 已订阅 {topic} 作为 PointCloud2")
            return True

        # Livox CustomMsg 路径
        if "CustomMsg" in msg_type_name:
            CustomMsg = None
            for pkg in ("livox_ros_driver2", "livox_ros_driver"):
                try:
                    mod = __import__(f"{pkg}.msg", fromlist=["CustomMsg"])
                    CustomMsg = getattr(mod, "CustomMsg")
                    break
                except Exception:
                    continue
            if CustomMsg is None:
                self.get_logger().error(
                    f"viz: 话题 {topic} 是 CustomMsg，但无法 import livox_ros_driver2.msg。\n"
                    "  请确认已 source ros2_ws/install/setup.bash 并安装了 livox_ros_driver2 Python 接口。\n"
                    "  也可以把 Livox launch 的 xfer_format 改成 0 以发 PointCloud2。"
                )
                return False

            def _cb(msg):
                _store(msg, "livox_custom")

            self._lidar_sub = self.create_subscription(CustomMsg, topic, _cb, 5, callback_group=self._cb_group_bulk)
            self._lidar_topic = topic
            self.get_logger().info(f"viz: 已订阅 {topic} 作为 livox CustomMsg")
            return True

        self.get_logger().error(f"viz: 暂不支持的消息类型 {msg_type_name} (topic={topic})")
        return False

    def get_latest_lidar_xyz(self) -> Tuple[Optional["np.ndarray"], str, int, int]:
        """供 visualization 拉取：返回 (xyz Nx3 或 None, frame_id, stamp_ns, seq)。

        在调用线程中完成 ROS msg → numpy 解码（不再阻塞 rclpy executor）。
        ``seq`` 是递增计数器，调用方可以用来判断"自上次以来有没有新帧"以做跳采。
        """
        import numpy as np
        with self._lidar_lock:
            msg = self._lidar_msg
            kind = self._lidar_msg_kind
            frame_id = self._lidar_frame_id
            stamp_ns = self._lidar_stamp_ns
            seq = self._lidar_seq
        if msg is None:
            return (None, frame_id, stamp_ns, seq)
        try:
            if kind == "pc2":
                from sensor_msgs_py import point_cloud2 as pc2
                arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
                if arr.size == 0:
                    return (None, frame_id, stamp_ns, seq)
                return (np.asarray(arr, dtype=np.float32).reshape(-1, 3), frame_id, stamp_ns, seq)
            if kind == "livox_custom":
                pts = msg.points
                n = len(pts)
                if n == 0:
                    return (None, frame_id, stamp_ns, seq)
                # list-comprehension + np.array 比逐元素赋值快 3-5 倍
                xyz = np.array([(p.x, p.y, p.z) for p in pts], dtype=np.float32)
                return (xyz, frame_id, stamp_ns, seq)
        except Exception as e:
            self.get_logger().warning(f"get_latest_lidar_xyz 解码失败: {e}")
        return (None, frame_id, stamp_ns, seq)

    def lookup_static_transform(self, target_frame: str, source_frame: str,
                                timeout_sec: float = 0.5) -> Optional[dict]:
        """单次 tf 查询（用于 viz 拿 base_link→lidar 等静态外参，cache 在调用方）。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            return {
                "translation": {
                    "x": float(tf.transform.translation.x),
                    "y": float(tf.transform.translation.y),
                    "z": float(tf.transform.translation.z),
                },
                "rotation": {
                    "x": float(tf.transform.rotation.x),
                    "y": float(tf.transform.rotation.y),
                    "z": float(tf.transform.rotation.z),
                    "w": float(tf.transform.rotation.w),
                },
            }
        except Exception:
            return None

    def _process_call_queue(self):
        """Process pending callables queued from other threads. Runs in ROS executor thread."""
        try:
            while True:
                with self._call_queue_lock:
                    if not self._call_queue:
                        break
                    func, args, kwargs = self._call_queue.popleft()
                try:
                    func(*args, **kwargs)
                except Exception as e:
                    self.get_logger().error(f"执行队列调用出错: {e}")
        except Exception as e:
            self.get_logger().error(f"处理调用队列时出错: {e}")

    def schedule_sync_call(self, func, *args, timeout: float = 5.0, **kwargs):
        """Schedule a callable to run inside ROS executor thread and wait for the result.

        The provided func should accept no special event parameter. The return value
        will be captured and returned. If timeout expires, raises TimeoutError.
        """
        done_event = threading.Event()
        result = {"value": None, "exc": None}

        def wrapper():
            try:
                result["value"] = func(*args, **kwargs)
            except Exception as e:
                result["exc"] = e
            finally:
                done_event.set()

        with self._call_queue_lock:
            self._call_queue.append((wrapper, (), {}))

        completed = done_event.wait(timeout)
        if not completed:
            raise TimeoutError("Timed out waiting for scheduled call to complete")
        if result["exc"]:
            raise result["exc"]
        return result["value"]

    def _nav_plan_callback(self, msg: Path) -> None:
        """缓存最近一次与当前 goal 代次一致的 /plan（Nav2 常见为 nav_msgs/Path）。"""
        ep_at_receive = self._plan_epoch
        copy = Path()
        copy.header = msg.header
        copy.poses = list(msg.poses)
        with self._plan_lock:
            if ep_at_receive != self._plan_epoch:
                return
            self._latest_plan = copy

    @staticmethod
    def path_msg_to_dict(path: Path) -> dict:
        """将 nav_msgs/Path 转为可 JSON 序列化的字典。"""
        poses_out = []
        for p in path.poses:
            poses_out.append({
                "position": {
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                },
                "orientation": {
                    "x": float(p.pose.orientation.x),
                    "y": float(p.pose.orientation.y),
                    "z": float(p.pose.orientation.z),
                    "w": float(p.pose.orientation.w),
                },
            })
        return {
            "frame_id": path.header.frame_id,
            "stamp": {
                "sec": int(path.header.stamp.sec),
                "nanosec": int(path.header.stamp.nanosec),
            },
            "poses": poses_out,
            "pose_count": len(poses_out),
        }

    def get_navigation_plan_snapshot(self) -> Optional[dict]:
        """
        返回当前缓存的非空全局路径；若无或 poses 为空则 None。
        应在 publish_goal 之后轮询，直到 planner 发布新路径。
        """
        with self._plan_lock:
            if self._latest_plan is None or len(self._latest_plan.poses) == 0:
                return None
            return NavNode.path_msg_to_dict(self._latest_plan)

    def wait_for_navigation_plan(self, timeout_sec: float = 10.0, poll_interval: float = 0.05) -> Tuple[bool, Optional[dict], str]:
        """
        阻塞等待非空 /plan（适用于已调用 publish_goal 之后）。

        返回:
            (True, plan_dict, "") 成功
            (False, None, 错误说明) 超时或失败
        """
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            snap = self.get_navigation_plan_snapshot()
            if snap is not None:
                return (True, snap, "")
            time.sleep(poll_interval)
        return (False, None, f"{timeout_sec}s 内未收到非空的 /plan 消息")

    def map_callback(self, msg):
        """
        地图话题回调函数 - 从地图消息中获取边界信息
        
        参数:
        msg: OccupancyGrid消息，包含地图数据和元信息
        """
        try:
            # 保存完整的地图消息
            self.current_map = msg
            
            # 获取地图元信息
            map_info = msg.info
            resolution = map_info.resolution  # 地图分辨率（米/像素）
            width = map_info.width           # 地图宽度（像素）
            height = map_info.height         # 地图高度（像素）
            origin = map_info.origin         # 地图原点在世界坐标系中的位置
            
            # 计算地图在世界坐标系中的边界
            # 原点是地图左下角的位置
            xmin = origin.position.x
            ymin = origin.position.y
            xmax = origin.position.x + width * resolution
            ymax = origin.position.y + height * resolution
            
            # 更新地图边界
            self.map_bound = {
                "xmin": xmin,
                "xmax": xmax,
                "ymin": ymin,
                "ymax": ymax
            }
            
            # 标记已接收到地图
            if not self.map_received:
                self.map_received = True
                self.get_logger().info(f"地图边界已更新:")
                self.get_logger().info(f"  X范围: [{xmin:.3f}, {xmax:.3f}]")
                self.get_logger().info(f"  Y范围: [{ymin:.3f}, {ymax:.3f}]")
                self.get_logger().info(f"  地图尺寸: {width}x{height} 像素")
                self.get_logger().info(f"  分辨率: {resolution:.3f} 米/像素")
            
        except Exception as e:
            self.get_logger().error(f"处理地图消息失败: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def _camera_image_callback(self, msg: CompressedImage) -> None:
        """缓存最新一帧相机 CompressedImage（仅保留 bytes 与时间戳）。"""
        if not self._camera_enabled:
            return  # VPR idle 时丢帧，无任何 Destroyable 操作
        try:
            data = bytes(msg.data)
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            with self._latest_image_lock:
                self._latest_image_bytes = data
                self._latest_image_stamp_ns = stamp_ns
            self._latest_image_event.set()
        except Exception as e:
            self.get_logger().error(f"camera image callback 出错: {e}")

    def enable_camera_subscription(self) -> None:
        """启用相机帧处理；引用计数 +1，归零前保持启用。

        线程安全且幂等。订阅句柄在 __init__ 时已创建，此处只翻转
        _camera_enabled 标志，彻底避免 create/destroy_subscription 的
        Destroyable 竞争。
        """
        with self._camera_sub_lock:
            self._camera_sub_refcount += 1
            if self._camera_sub_refcount == 1:
                self._camera_enabled = True
                self.get_logger().info(
                    f"camera 订阅已启用: {self.camera_image_topic}"
                )
    def disable_camera_subscription(self) -> None:
        """禁用相机帧处理；引用计数 -1，归零时停止处理帧并清空缓存。

        线程安全；多调 disable 不报错（refcount 下界为 0）。
        只翻转 _camera_enabled 标志，不做任何 Destroyable 操作。
        """
        with self._camera_sub_lock:
            if self._camera_sub_refcount > 0:
                self._camera_sub_refcount -= 1
            if self._camera_sub_refcount > 0:
                return
            if self._camera_enabled:
                self._camera_enabled = False
                # 清掉陈旧帧；让 get_latest_image_bytes 等到下次启用后再取新帧
                with self._latest_image_lock:
                    self._latest_image_bytes = None
                    self._latest_image_stamp_ns = None
                self._latest_image_event.clear()
                self.get_logger().info("camera 订阅已禁用")
    def get_latest_image_bytes(
        self,
        timeout_sec: float = 5.0,
        max_age_sec: Optional[float] = None,
    ) -> Optional[Tuple[bytes, int]]:
        """获取最新一帧相机 CompressedImage 的字节与时间戳（ns）。

        :param timeout_sec: 等待首帧到来的超时时间（已收到过任意一帧时立刻返回）
        :param max_age_sec: 若指定，要求最新一帧相对当前 ROS 时间的年龄不超过该值；
                             否则继续等待新帧。
        :return: ``(jpeg_bytes, stamp_ns)`` 或 None（超时未拿到合规帧）
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while True:
            with self._latest_image_lock:
                data = self._latest_image_bytes
                stamp_ns = self._latest_image_stamp_ns

            if data is not None and stamp_ns is not None:
                if max_age_sec is None:
                    return data, stamp_ns
                try:
                    now_ns = self.get_clock().now().nanoseconds
                    age_ns = now_ns - stamp_ns
                    if age_ns < int(max_age_sec * 1e9):
                        return data, stamp_ns
                except Exception:
                    return data, stamp_ns

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            # 等待回调线程置事件后再轮询；超时时退出
            self._latest_image_event.clear()
            self._latest_image_event.wait(timeout=min(0.2, remaining))

    def _on_relocalization_request(self, reason: str) -> None:
        """换图/发初始位姿后：清定位标志，避免把旧 TF 当新定位；便于再次打出「TF定位成功」等日志。"""
        self.tf_ready = False
        self.tf_fail_count = 0
        self.current_pose = None
        # 记录本次重定位请求的 ROS 时刻（纳秒）。
        # update_robot_pose 只有在拿到时间戳 >= 该值的 TF 帧后，
        # 才允许把 tf_ready 从 False 恢复为 True，防止 tf2 buffer
        # 里换图前的旧缓存帧把 tf_ready 立即拉回 True。
        try:
            self._min_tf_stamp_for_ready_ns: int = self.get_clock().now().nanoseconds
        except Exception:
            self._min_tf_stamp_for_ready_ns = 0
        self.get_logger().info(
            f"{reason}：已清除位姿缓存并将定位判为未就绪，待 map→base_link 再次稳定。"
        )

    def update_robot_pose(self):
        """
        定时更新机器人位姿（从tf获取）
        """
        try:
            # 获取从map到base_link的变换
            transform = self.tf_buffer.lookup_transform(
                'map',  # 目标坐标系
                'base_link',  # 源坐标系
                rclpy.time.Time(),  # 获取最新的变换
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            now = self.get_clock().now()

            # ── 计算 TF 新鲜度 ────────────────────────────────────────
            # 注意：lookup_transform('map','base_link') 在 tf2 内是把
            # map→odom 与 odom→base_link 两段链路合成的；返回的
            # header.stamp 取链路中"最旧那一段的最新时间"。
            # FAST-LIO 通常只在重定位修正时才广播 map→odom（几秒~十几秒一帧），
            # 而 odom→base_link 由底盘里程计 ~50-100Hz 广播；因此合成 stamp
            # 看起来"老"，但 pose 是用 odom→base_link 最新一帧外推得到的，
            # 位姿实际是新鲜的。
            #
            # 这里改用 odom→base_link 的 stamp 作为真正的新鲜度指标；
            # 当该叶子查询失败时回退到合成 stamp，兼容直接 map→base_link
            # 的 TF 树。
            leaf_age_sec: Optional[float] = None
            try:
                leaf_tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
                leaf_stamp_tm = rclpy.time.Time.from_msg(leaf_tf.header.stamp)
                leaf_age_sec = max(0.0, (now - leaf_stamp_tm).nanoseconds * 1e-9)
            except TransformException:
                leaf_age_sec = None

            composed_stamp_tm = rclpy.time.Time.from_msg(transform.header.stamp)
            composed_age_sec = max(0.0, (now - composed_stamp_tm).nanoseconds * 1e-9)

            # 叶子优先；不可用时回退到合成 stamp
            stamp_age_sec = leaf_age_sec if leaf_age_sec is not None else composed_age_sec

            if stamp_age_sec > self._tf_stale_warn_sec:
                mono = time.monotonic()
                if mono - self._last_stale_tf_warn_mono >= 4.0:
                    self._last_stale_tf_warn_mono = mono
                    if leaf_age_sec is not None:
                        self.get_logger().warning(
                            f"TF 新鲜度告警：odom→base_link 时间戳落后约 {leaf_age_sec:.2f}s "
                            f"（map→base_link 合成 stamp 落后 {composed_age_sec:.2f}s）。"
                            "若仅合成 stamp 老、叶子 stamp 新鲜，通常只是 map→odom 长时间未更新，"
                            "不代表定位丢失。"
                        )
                    else:
                        self.get_logger().warning(
                            f"TF 新鲜度告警：map→base_link 合成 stamp 落后约 {composed_age_sec:.2f}s "
                            "（无法读取 odom→base_link 单独 stamp，可能是 TF 链非 map→odom→base_link）。"
                        )

            # 失败阈值同样用叶子 stamp：只有底盘 TF 真的停了才判为定位丢失，
            # 避免 FAST-LIO 修正稀疏时误把"map→odom 沉默"当作定位失败。
            if stamp_age_sec > self._tf_stale_fail_sec:
                if self.tf_ready:
                    self.tf_ready = False
                    self.get_logger().warn(
                        f"✗ TF定位丢失：odom→base_link 时间戳落后 {stamp_age_sec:.2f}s "
                        f"超过失败阈值 {self._tf_stale_fail_sec:.1f}s，进入not ready状态"
                    )
                    self.play_audio("fail.mp3")
                return

            # 将Transform转换为PoseStamped格式（兼容原有代码）
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.header.frame_id = 'map'
            
            # 复制位置信息
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            
            # 复制姿态信息
            pose_stamped.pose.orientation = transform.transform.rotation
            
            # 更新当前位姿
            self.current_pose = pose_stamped
            # print(f"transform:{pose_stamped}")
            # 添加调试日志
            self.get_logger().debug(f"更新机器人位姿: x={pose_stamped.pose.position.x:.3f}, y={pose_stamped.pose.position.y:.3f}")
            
            # 重置失败计数
            self.tf_fail_count = 0
            
            # 检测状态变化：从 not ready 变为 ready
            if not self.tf_ready:
                # 换图后 tf2 buffer 里可能残留旧地图的缓存帧，
                # 用时间戳门控：只接受产生于重定位请求之后的新帧。
                min_ns: int = getattr(self, '_min_tf_stamp_for_ready_ns', 0)
                if min_ns > 0:
                    tf_stamp_ns = (
                        leaf_stamp_tm.nanoseconds
                        if leaf_age_sec is not None
                        else composed_stamp_tm.nanoseconds
                    )
                    if tf_stamp_ns < min_ns:
                        # 这帧产生于重定位请求之前，属于旧地图残留，跳过
                        return
                    # 帧足够新，清除门控
                    self._min_tf_stamp_for_ready_ns = 0
                # 室外 GPS 模式下，map→odom 在收到首个 GPS 前是单位阵占位
                # （map_odom_from_gps 的 publish_identity_until_gps），此时 TF 链
                # 虽连通但 GPS 尚未真正定位。额外要求 GNSS fix 可用，避免在 GPS
                # 收敛前就误报“定位成功”。SLAM 模式不受影响。
                #
                # 另：SLAM→GPS 切换时，旧 SLAM 栈残留/异步续发的 map→odom 会让 TF 链
                # 立刻连通，而 GNSS fix 由传感器持续发布（仿真里恒为可用），二者都不足以
                # 区分新旧栈，会在新室外栈启动完成前误报就绪、把目标发给未就绪的栈而丢失。
                # 故额外要求 campus_planner_ready()：新栈的 route_planner 已订阅
                # /campus_goal_pose。这里原先用 _gps_odom_received（/odometry/gps 收到过），
                # 但那条话题要等航向锁定才发布，会让切换流程站着等到超时——详见
                # campus_planner_ready 的说明。
                if (self.nav_mode == self.NAV_MODE_GPS
                        and not (self.is_gps_available()
                                 and self.campus_planner_ready())):
                    return
                # SLAM 模式：切换后 map_received 被复位，只有新 SLAM 栈重新发布 /map
                # 才会置回 True。以此确保“定位就绪”对应的是新栈（而非旧栈残留 TF），
                # 避免在 SLAM 栈启动完成前就误报就绪、把目标发给未就绪的栈而丢失。
                if (self.nav_mode == self.NAV_MODE_SLAM
                        and not self.map_received):
                    return
                self.tf_ready = True
                self.get_logger().info("✓ TF定位成功，进入ready状态")
                self.play_audio("success.mp3")
            
        # 仅当 map→base_link 在 buffer 中无法求出（缺边、时间外推失败等）时才会抛出，与「缓冲里仍有旧变换」不同
        except TransformException as ex:
            self.get_logger().debug(f"无法获取tf变换: {ex}")
            self.tf_fail_count += 1
            
            # 检测状态变化：从 ready 变为 not ready（连续失败超过阈值）
            if self.tf_ready and self.tf_fail_count >= self.tf_fail_threshold:
                self.tf_ready = False
                self.get_logger().warn("✗ TF定位丢失，进入not ready状态")
                self.play_audio("fail.mp3")
                
        except Exception as e:
            self.get_logger().error(f"更新机器人位姿时出错: {e}")

    def is_map_localized(self) -> bool:
        """map→base_link TF 连续可用且有位姿缓存（与 get_status 中 localized 含义一致）。"""
        return bool(self.tf_ready and self.current_pose is not None)

    # ══════════════════════════════════════════════════════════════════════
    # 导航模式（SLAM / GPS）与室外 GNSS 导航
    # ══════════════════════════════════════════════════════════════════════
    def _publish_nav_mode(self) -> None:
        """把当前导航模式发布到 /nav_mode（供其它节点联动）。"""
        try:
            msg = String()
            msg.data = self.nav_mode
            self.nav_mode_pub.publish(msg)
        except Exception as e:
            self.get_logger().warning(f"发布 /nav_mode 失败: {e}")

    def get_nav_mode(self) -> str:
        """返回当前导航模式：``"slam"`` 或 ``"gps"``。"""
        return self.nav_mode

    def set_nav_mode(self, mode: str) -> Tuple[bool, str]:
        """切换导航模式（SLAM 室内点云定位 ↔ GPS 室外 GNSS 导航）。

        切换时统一做清场，避免旧模式的目标/精调残留把新模式带偏：
          - 停车（发零速），关闭 PID 精调；
          - 取消当前 Nav2 (navigate_to_pose) 目标；
          - 清空室外目标与运行标志；
          - 复位定位就绪判据（新模式的定位需要重新建立）；
          - 向 /nav_mode 通告新模式。

        :param mode: ``"slam"`` 或 ``"gps"``
        :return: (success, message)
        """
        if mode not in (self.NAV_MODE_SLAM, self.NAV_MODE_GPS):
            return False, f"未知导航模式: {mode}（可选 slam / gps）"
        with self._nav_mode_lock:
            if mode == self.nav_mode:
                return True, f"已处于 {mode} 模式"
            old = self.nav_mode
            self._stop_all_navigation(reason=f"切换导航模式 {old} → {mode}")
            self.nav_mode = mode
            # 新模式下定位需重新建立：清掉 SLAM 侧的 tf 就绪标志与位姿缓存，
            # 让上层等待新的 map→base_link / GNSS fix 稳定后再判定“已定位”。
            self.tf_ready = False
            self.tf_fail_count = 0
            self.current_pose = None
            # 关键：清空 tf2 buffer，丢弃旧模式残留的 map→odom / map→base_link 缓存帧。
            # 否则切换后（新栈还在启动）update_robot_pose 会立刻用旧 GPS 的
            # map→odom 合成出 map→base_link，把 tf_ready 拉回 True，使上层
            # (_wait_localized) 误判“已定位”，在新栈就绪前就 publish_goal，导致
            # 目标发给旧/未就绪的栈而丢失。静态变换由 BufferCore 单独保存，clear 不影响。
            try:
                self.tf_buffer.clear()
            except Exception as e:
                self.get_logger().warning(f"清空 tf buffer 失败(忽略): {e}")
            # 时间戳门控：只接受切换时刻之后产生的新 map→base_link 帧再置 ready，
            # 与换图 (_on_relocalization_request) 同机制，双保险。
            try:
                self._min_tf_stamp_for_ready_ns = self.get_clock().now().nanoseconds
            except Exception:
                self._min_tf_stamp_for_ready_ns = 0
            # 切到 SLAM：仅靠 TF 无法区分“旧 GPS 栈仍在发的 map→odom”和“新 SLAM 栈的
            # map→odom”（旧栈被 start_slam.sh 异步 kill 前会继续发新时间戳帧，清 buffer
            # 也会被立刻重新填满）。因此额外要求新栈重新发布过 /map（OccupancyGrid，
            # 仅 SLAM 栈发布）才认定就绪：复位 map_received，令其只能被新栈的 /map 置回
            # True（见 update_robot_pose 的就绪门控）。
            if mode == self.NAV_MODE_SLAM:
                self.map_received = False
                self.current_map = None
            # 切到 GPS：新旧栈的区分现在由 campus_planner_ready()（/campus_goal_pose 有
            # 订阅者）承担，见其说明。这里仍复位 _gps_odom_received / _latest_gps_odom，
            # 但只作为排查信息（navsat 是否已在发 /odometry/gps，即航向是否已锁定），
            # 不再参与就绪判定。
            if mode == self.NAV_MODE_GPS:
                self._gps_odom_received = False
                self._latest_gps_odom = None
            self._publish_nav_mode()
            self.get_logger().info(f"✓ 导航模式已切换: {old} → {mode}")
        return True, f"已切换到 {mode} 模式"

    def _stop_all_navigation(self, reason: str = "") -> None:
        """切换/停止时的统一清场：停车、关精调、取消 Nav2 目标、清室外目标。

        仅做“尽力而为”的本地清理：navigate_to_pose 目标会被取消；室外路网规划
        （navigate_through_poses 由 campus_route_planner 持有）不在本节点句柄内，
        这里通过清空室外目标 + 发零速降低风险，真正的接管由上层重新下发目标完成。
        """
        if reason:
            self.get_logger().info(f"停止当前导航：{reason}")
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        self.cmd_vel_sent = None
        self.stuck_count = 0
        self.pid_control_active = False
        # 取消进行中的 navigate_to_pose 目标（SLAM 侧）
        if self.nav_goal_active and self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"取消当前 Nav2 目标失败: {e}")
        self.nav_goal_active = False
        self.nav_goal_reached = False
        # 清室外目标与运行标志
        self._outdoor_goal = None
        self._outdoor_nav_active = False
        self._outdoor_goal_name = None

    # ── GNSS 定位监控 ─────────────────────────────────────────────────────
    def _gps_fix_callback(self, msg: NavSatFix) -> None:
        """缓存最新 NavSatFix：经纬高 + 定位状态 + 时间戳。"""
        try:
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            fix = {
                "lat": float(msg.latitude),
                "lon": float(msg.longitude),
                "alt": float(msg.altitude),
                "status": int(msg.status.status),
                "service": int(msg.status.service),
                "position_covariance": list(msg.position_covariance),
                "position_covariance_type": int(msg.position_covariance_type),
                "stamp_sec": float(stamp_sec),
                "mono": time.monotonic(),
            }
            with self._gps_lock:
                self._latest_gps_fix = fix
        except Exception as e:
            self.get_logger().error(f"GNSS fix 回调出错: {e}")

    def _gps_odom_callback(self, msg: Odometry) -> None:
        """缓存 navsat_transform 输出的 GPS 里程 (x, y, mono)，仅作健康度参考。"""
        self._latest_gps_odom = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            time.monotonic(),
        )
        # 该话题只有室外栈的 navsat_transform 发布：收到即说明新室外栈已起来。
        self._gps_odom_received = True

    def get_latest_gps_fix(self) -> Optional[Dict[str, Any]]:
        """返回最新一帧 NavSatFix 的字段字典，或 None（还未收到）。"""
        with self._gps_lock:
            return dict(self._latest_gps_fix) if self._latest_gps_fix else None

    def is_gps_available(self, max_age_sec: Optional[float] = None) -> bool:
        """GNSS 是否可用：有定位解（status >= STATUS_FIX）且 fix 足够新鲜。

        :param max_age_sec: 允许的最大 fix 年龄（秒），None 时用 ``_gps_fresh_sec``。
        """
        max_age = self._gps_fresh_sec if max_age_sec is None else max_age_sec
        with self._gps_lock:
            fix = self._latest_gps_fix
        if not fix:
            return False
        if fix["status"] < NavSatStatus.STATUS_FIX:  # STATUS_NO_FIX = -1
            return False
        return (time.monotonic() - fix["mono"]) <= max_age

    def campus_planner_ready(self) -> bool:
        """新室外栈的 campus 路网规划器是否已经能接目标。

        判据是 ``/campus_goal_pose`` 有订阅者。这一个信号同时承担三件事：

        1. **区分新旧栈**——只有 campus_nav 的 route_planner 订阅这个话题，而它只存在于
           室外栈；``start_outdoor.sh`` 又会先 ``tmux kill-session -t outdoor``，所以订阅者
           出现只能是新栈。这替代了原来用 ``_gps_odom_received`` 做的同一件事。
        2. **不依赖航向锁定**——原判据要等 ``/odometry/gps``，而那条话题要等
           gps_heading_align 锁定航向（需走够 3 m）才由 navsat 发布。切换流程里
           ``wait_until_stopped()`` 刚把车停住、之后没有任何东西驱动它，于是站着等锁定，
           直到 ``localize_timeout`` 超时。改用本判据后「边走边锁」才成立：目标先下发
           （``require_heading_lock`` 已为 false），机器人走起来自己把 baseline 攒上。
        3. **目标不会被静默丢弃**——``/campus_goal_pose`` 两端都是 VOLATILE 无 latch 队列，
           而 route_planner 要到室外栈启动后约 9 s 才建立订阅（TimerAction 7 s + 加载 6 万
           节点 OSM 图）。在那之前 publish 的目标会被永久丢掉且两端都不报错。
        """
        return self.campus_goal_pub.get_subscription_count() > 0

    def is_gps_localized(self) -> bool:
        """室外 GPS 定位是否就绪：GNSS fix 新鲜可用，且 map→base_link 位姿已建立。

        与 SLAM 的 ``is_map_localized`` 对应——室外由 navsat_transform +
        map_odom_from_gps 融合出 map→odom→base_link，故仍以 ``current_pose``
        判断底盘位姿是否可用，再叠加 GNSS fix 健康度。

        新旧栈的区分交给 ``campus_planner_ready()``：``current_pose`` 会被旧 SLAM 栈残留的
        map→odom 立刻填上、GNSS fix 又由传感器持续发布，两者都分不清新旧栈。这里原先用
        ``_gps_odom_received``，但它要等航向锁定（见 ``campus_planner_ready`` 的说明），会让
        ``_wait_localized`` 站着等到超时。
        """
        return bool(self.is_gps_available()
                    and self.campus_planner_ready()
                    and self.current_pose is not None)

    def is_localized(self) -> bool:
        """按当前导航模式返回定位是否就绪（SLAM 看点云 TF，GPS 看 GNSS fix）。"""
        if self.nav_mode == self.NAV_MODE_GPS:
            return self.is_gps_localized()
        return self.is_map_localized()

    # ── 室外目标下发（经 campus_nav 路网规划器）─────────────────────────────
    def _lonlat_to_campus_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        """经纬度 → campus_map 平面 (east, north)，等距圆柱投影（同 route_graph.ENU）。"""
        east = math.radians(lon - self.gps_datum_lon) * self._EARTH_R * self._gps_datum_cos0
        north = math.radians(lat - self.gps_datum_lat) * self._EARTH_R
        return east, north

    def _campus_xy_to_lonlat(self, east: float, north: float) -> Tuple[float, float]:
        """campus_map 平面 (east, north) → 经纬度，_lonlat_to_campus_xy 的逆变换。"""
        lat = self.gps_datum_lat + math.degrees(north / self._EARTH_R)
        lon = self.gps_datum_lon + math.degrees(east / (self._EARTH_R * self._gps_datum_cos0))
        return lon, lat

    def send_campus_goal(self, pose: PoseStamped,
                         wait_planner_s: float = 30.0) -> bool:
        """把一个 campus_frame 下的 PoseStamped 目标发给路网规划器（campus_goal_pose）。

        规划器会把该点吸附到最近道路、A* 出道路级路线并分批下发 Nav2。仅记录目标
        与运行标志；实际道路规划/驱动 Nav2 由 campus_route_planner 完成。

        发布前先确认 route_planner 已订阅：话题两端都是 VOLATILE 无 latch 队列，而
        route_planner 要到室外栈启动后约 9 s 才建立订阅（TimerAction 7 s + 加载 6 万节点
        OSM 图）。在那之前 publish 的目标会被**永久丢弃且两端都不报错**，表现为「路径发
        不过去」。这里等到订阅出现再发，超时则明确失败而不是往虚空里扔。
        """
        if not self.campus_planner_ready():
            deadline = time.time() + wait_planner_s
            self.get_logger().info(
                f'campus 路网规划器尚未订阅 {self.campus_goal_pub.topic_name}，'
                f'等待其就绪（最多 {wait_planner_s:.0f}s）…')
            while time.time() < deadline and not self.campus_planner_ready():
                time.sleep(0.2)
            if not self.campus_planner_ready():
                self.get_logger().error(
                    f'等待 {wait_planner_s:.0f}s 后 {self.campus_goal_pub.topic_name} '
                    f'仍无订阅者，放弃下发（若此刻 publish 会被静默丢弃）。'
                    f'请确认室外栈的 campus_route_planner 已启动。')
                return False
            self.get_logger().info(
                f'campus 路网规划器已就绪（等待 '
                f'{wait_planner_s - (deadline - time.time()):.1f}s）')
        try:
            self.campus_goal_pub.publish(pose)
            self._outdoor_goal = pose
            self._outdoor_nav_active = True
            self.get_logger().info(
                f"已下发室外路网目标: frame={pose.header.frame_id}, "
                f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"下发室外路网目标失败: {e}")
            return False

    def set_outdoor_goal_by_xy(
        self, x: float, y: float, frame: Optional[str] = None
    ) -> bool:
        """以 campus_frame（默认）平面坐标下发室外目标。

        :param x, y: 平面坐标（默认 campus_map 系，即 ENU east/north）
        :param frame: 目标坐标系，None 时用 ``self.campus_frame``；规划器会 TF 到
                      campus_map 后再吸附道路，故也可传 ``"map"`` 等可 TF 的系。
        """
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame or self.campus_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0  # 朝向对路网规划无意义，规划器只用 xy 吸附道路
        return self.send_campus_goal(pose)

    def set_outdoor_goal(self, lon: float, lat: float,
                         target: Optional[str] = None,
                         tolerance: float = 1.0) -> Dict[str, Any]:
        """室外目标经 campus_nav 路网规划器下发（``campus_goal_pose``）。

        规划器会根据距离自动选择路网分段或 Nav2 直达，无需在此按距离分支。
        经纬度经 ENU datum 换算成 campus_map 平面点后调用 ``set_outdoor_goal_by_xy``。

        记录 ``_outdoor_goal`` / ``target`` / ``tolerance``，供 ``wait_goal_reached`` 判定到达。

        :return: {ok, channel: 'campus', distance, target, tolerance}
        """
        east, north = self._lonlat_to_campus_xy(lon, lat)

        dist = None
        if self.current_pose is not None:
            mapped = self._transform_xy(east, north, self.campus_frame, "map")
            if mapped is not None:
                gx, gy = mapped
                rx = self.current_pose.pose.position.x
                ry = self.current_pose.pose.position.y
                dist = math.hypot(gx - rx, gy - ry)

        self._outdoor_goal_name = target
        self._outdoor_goal_tol = float(tolerance)
        ok = self.set_outdoor_goal_by_xy(east, north, frame=self.campus_frame)
        dist_str = "未知" if dist is None else f"{dist:.1f}m"
        self.get_logger().info(
            f"室外目标→campus 路网规划器 target={target} dist={dist_str} tol={tolerance}")
        return {"ok": bool(ok), "channel": "campus", "distance": dist,
                "target": target, "tolerance": float(tolerance)}

    def cancel_outdoor_goal(self) -> None:
        """清除本地室外目标与运行标志，并发零速停车（尽力而为）。"""
        self._outdoor_goal = None
        self._outdoor_nav_active = False
        self._outdoor_goal_name = None
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def _transform_xy(
        self, x: float, y: float, from_frame: str, to_frame: str
    ) -> Optional[Tuple[float, float]]:
        """把 from_frame 下的平面点 (x,y) 变换到 to_frame，失败返回 None。"""
        if from_frame == to_frame:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        cy, sy = math.cos(yaw), math.sin(yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        return tx + cy * x - sy * y, ty + sy * x + cy * y

    def is_outdoor_goal_reached(self, tolerance: float = 3.0) -> bool:
        """室外目标是否到达：把 campus_frame 目标变换到 map 后与当前位姿比距离。

        campus_route_planner 不发布“路线完成”话题，这里用几何距离近似判定；
        ``tolerance`` 默认与规划器的 ``nav2_reach_tol_m`` 相近（3m）。
        """
        if self._outdoor_goal is None or self.current_pose is None:
            return False
        goal = self._outdoor_goal
        mapped = self._transform_xy(
            goal.pose.position.x, goal.pose.position.y,
            goal.header.frame_id or self.campus_frame, "map",
        )
        if mapped is None:
            return False
        gx, gy = mapped
        dx = gx - self.current_pose.pose.position.x
        dy = gy - self.current_pose.pose.position.y
        if math.hypot(dx, dy) <= tolerance:
            self._outdoor_nav_active = False
            return True
        return False

    def get_outdoor_goal_name(self) -> Optional[str]:
        """当前室外目标关联的语义点名称（未设置返回 None）。"""
        return self._outdoor_goal_name

    def get_outdoor_goal_tolerance(self) -> float:
        """当前室外目标的到达判定距离阈值（米）。"""
        return self._outdoor_goal_tol

    def set_slam_goal_tolerance(self, xy_tolerance: float, yaw_tolerance: float) -> None:
        """记录 SLAM 目标到达判定容差（下发目标时设定，供 wait_goal_reached 取回）。"""
        self._slam_goal_xy_tol = float(xy_tolerance)
        self._slam_goal_yaw_tol = float(yaw_tolerance)

    def get_slam_goal_tolerance(self) -> Tuple[float, float]:
        """当前 SLAM 目标的到达判定容差 (xy 米, yaw 弧度)。"""
        return self._slam_goal_xy_tol, self._slam_goal_yaw_tol

    def chassis_speed_callback(self, msg: TwistStamped):
        """
        底盘速度反馈回调函数
        
        参数:
        msg: TwistStamped消息，包含底盘实际速度
        """
        self.chassis_speed = msg.twist

    def _odom_callback(self, msg: Odometry):
        """缓存最新 /odom 位姿 (x, y, yaw_rad, stamp_sec)。

        用 ZYX-yaw 提取（chassis 几乎水平时数值稳定），无需 tf2 依赖。
        写入一个 tuple 即可，GIL 让外部读取拿到的是要么旧 tuple 要么完整新
        tuple，不会拿到半新半旧的字段。

        节流：底盘 50-100Hz 触发回调，但 VPR 运动门控只需要 ~10Hz 的累计位移/转角
        信息。早期 return 避免在 SingleThreadedExecutor 上挤占其他回调（特别是
        update_robot_pose 的 TF 心跳）的 GIL 时间。
        """
        mono = time.monotonic()
        if mono - self._odom_last_keep_mono < self._odom_min_keep_interval_sec:
            return
        self._odom_last_keep_mono = mono

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
            float(stamp_sec),
        )

    def get_latest_odom_pose(self) -> Optional[Tuple[float, float, float, float]]:
        """返回最新一帧 /odom 位姿 ``(x_m, y_m, yaw_rad, stamp_sec)`` 或 None。

        供 VprRelocator.collect_consensus 做"运动门控"——比较相邻两次共识
        采样之间的位移/转角增量，决定要不要等到下一周期再采。
        """
        return self._latest_odom_pose

    def _elevator_moving_callback(self, msg: Bool):
        """
        /elevator_moving 回调：true 表示电梯正在升降。

        语义：
          - 升降开始（False -> True）：暂停 Nav2 与 PID 精调，发零速并取消当前 goal；
            记录"曾因电梯而暂停"的标记，恢复时自动重发 last_goal。
          - 升降结束（True -> False）：清除标记，若曾暂停 Nav2 则重发 last_goal；
            PID 精调由 pid_control_step 内的 elevator_moving 判断自动恢复。
        """
        try:
            new_state = bool(msg.data)
            if new_state == self.elevator_moving:
                return  # 状态未变化

            if new_state:
                # 进入升降：先停车，再视情况取消 Nav2 当前目标
                self.elevator_moving = True
                self.get_logger().info("/elevator_moving=true：电梯升降开始，暂停导航与精调")
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                self.cmd_vel_sent = None
                self.stuck_count = 0

                if self.nav_goal_active and self.current_goal_handle is not None:
                    self._elevator_paused_nav = True
                    self._elevator_cancel_in_flight = True
                    try:
                        self.current_goal_handle.cancel_goal_async()
                        self.get_logger().info("已请求取消 Nav2 当前目标（电梯升降暂停）")
                    except Exception as e:
                        self.get_logger().error(f"电梯暂停取消 Nav2 目标失败: {e}")
                        self._elevator_cancel_in_flight = False
            else:
                # 升降结束：恢复
                self.elevator_moving = False
                # 给底盘一段唤醒宽限期 + 清掉旧的速度反馈/命令缓存，避免被误判卡住
                self._elevator_resume_grace_until = time.monotonic() + self._elevator_resume_grace_sec
                self.chassis_speed = None
                self.cmd_vel_sent = None
                self.stuck_count = 0
                self.get_logger().info(
                    f"/elevator_moving=false：电梯升降结束，恢复导航与精调（卡住检测宽限 {self._elevator_resume_grace_sec:.1f}s）"
                )
                if self._elevator_paused_nav and self.last_goal is not None:
                    self._elevator_paused_nav = False
                    try:
                        self.publish_goal(self.last_goal, target="elevator_resume")
                    except Exception as e:
                        self.get_logger().error(f"电梯恢复重发 Nav2 目标失败: {e}")
                else:
                    self._elevator_paused_nav = False
        except Exception as e:
            self.get_logger().error(f"处理 /elevator_moving 回调出错: {e}")

    def is_robot_stuck(self) -> bool:
        """
        检测机器人是否被卡住
        
        返回:
        bool: True表示机器人被卡住，False表示正常
        """
        return False
        # 电梯升降刚结束的宽限期内不做卡住检测，避免把底盘唤醒延迟误判为卡死
        if self._elevator_resume_grace_until and time.monotonic() < self._elevator_resume_grace_until:
            self.stuck_count = 0
            return False

        # 如果没有发送过速度命令或没有底盘反馈，不认为卡住
        if self.cmd_vel_sent is None or self.chassis_speed is None:
            return False
        
        # 计算发送的速度幅值
        cmd_linear = (self.cmd_vel_sent.linear.x**2 + self.cmd_vel_sent.linear.y**2) ** 0.5
        cmd_angular = abs(self.cmd_vel_sent.angular.z)
        
        # 计算反馈的速度幅值
        feedback_linear = (self.chassis_speed.linear.x**2 + self.chassis_speed.linear.y**2) ** 0.5
        feedback_angular = abs(self.chassis_speed.angular.z)
        
        # 如果发送了速度但反馈接近0，认为可能卡住
        cmd_magnitude = cmd_linear + cmd_angular
        feedback_magnitude = feedback_linear + feedback_angular
        
        if cmd_magnitude > self.stuck_check_threshold and feedback_magnitude < self.stuck_check_threshold:
            self.stuck_count += 1
            if self.stuck_count >= self.stuck_count_threshold:
                self.get_logger().warn(f"检测到机器人卡住: 发送速度={cmd_magnitude:.3f}, 反馈速度={feedback_magnitude:.3f}")
                return True
        else:
            # 速度正常，重置计数
            self.stuck_count = 0
        
        return False

    def play_audio(self, filename):
        """
        播放音频文件（异步，不阻塞主线程）
        
        参数:
        filename: 音频文件名（如 success.mp3 或 fail.mp3）
        """
        audio_path = os.path.join(self.audio_dir, filename)
        if not os.path.exists(audio_path):
            self.get_logger().warn(f"音频文件不存在: {audio_path}")
            return
        
        def play():
            try:
                subprocess.run(
                        ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', audio_path],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
            except Exception as e:
                self.get_logger().debug(f"播放音频失败: {e}")
        
        # 在后台线程播放，避免阻塞ROS回调
        threading.Thread(target=play, daemon=True).start()

    def goal_status_callback(self, future):
        """
        Action反馈回调函数 - 监听导航状态变化
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('导航目标未被接受')
                self.nav_goal_active = False
                self.nav_goal_reached = False
                self._nav_terminal_message = "导航目标未被接受"
                return
            self.current_goal_handle = goal_handle
            self.get_logger().info('导航目标已被接受，开始执行...')

            epoch = self._plan_epoch
            goal_handle.get_result_async().add_done_callback(
                lambda f, ep=epoch: self._nav_result_callback(f, ep)
            )
        except Exception as e:
            self.get_logger().error(f"状态回调出错: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def _nav_result_callback(self, future, epoch):
        try:
            result = future.result().result
            status = future.result().status
            status_names = {
                GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
                GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
                GoalStatus.STATUS_EXECUTING: "EXECUTING",
                GoalStatus.STATUS_CANCELING: "CANCELING",
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED"
            }
            status_name = status_names.get(status, f"UNKNOWN({status})")

            if epoch != self._plan_epoch:
                self.get_logger().info(f"忽略旧目标的结果回调: {status_name} (epoch {epoch} != {self._plan_epoch})")
                return

            self.get_logger().info(f"导航状态: {status_name}")
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('✓ Navigation2导航成功完成！进入PID控制状态')
                self.nav_goal_reached = True
                self.nav_goal_active = False
                self.pid_control_active = True
                self.pid_rotation_phase = True
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('✗ Navigation2导航被中止')
                self.nav_goal_reached = False
                self.nav_goal_active = False
                self.current_goal_handle = None
                self.pid_control_active = False
                self._nav_terminal_message = "Navigation2 导航被中止"
            elif status == GoalStatus.STATUS_CANCELED:
                if self._elevator_cancel_in_flight:
                    # 电梯升降触发的取消：不视为失败，恢复后由 _elevator_moving_callback 重发 goal
                    self.get_logger().info('Navigation2导航被取消（电梯升降暂停）')
                    self.nav_goal_reached = False
                    self.nav_goal_active = False
                    self.current_goal_handle = None
                    self.pid_control_active = False
                    self._elevator_cancel_in_flight = False
                else:
                    self.get_logger().info('Navigation2导航被取消')
                    self.nav_goal_reached = False
                    self.nav_goal_active = False
                    self.current_goal_handle = None
                    self.pid_control_active = False
                    self._nav_terminal_message = "Navigation2 导航已取消"
            else:
                self.nav_goal_reached = False
                self.nav_goal_active = False
                self.current_goal_handle = None
                self.pid_control_active = False
                self._nav_terminal_message = f"导航结束状态异常: {status_name}"
        except Exception as e:
            self.get_logger().error(f"导航结果回调出错: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def publish_goal(self, pose: PoseStamped, target):
        """
        通过navigate_to_pose action client发布导航目标点
        """
        try:
            # 重置状态
            self.nav_goal_reached = False
            self.nav_goal_active = True
            self.pid_control_active = False
            self.current_goal_handle = None
            self._nav_terminal_message = None
            # 新 goal：清空路径缓存并递增代次，忽略旧 goal 的迟到 /plan
            with self._plan_lock:
                self._plan_epoch += 1
                self._latest_plan = None

            # 构造action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            # 记录最后发布的目标点
            self.last_goal = pose

            self.get_logger().info(f"发布目标点:name={target}, x={pose.pose.position.x}, y={pose.pose.position.y}")
            self.get_logger().info("等待导航服务器可用...")
            # run wait_for_server and send_goal_async inside ROS thread
            def do_send():
                self.nav_action_client.wait_for_server()
                fut = self.nav_action_client.send_goal_async(goal_msg)
                fut.add_done_callback(self.goal_status_callback)
                return True

            return self.schedule_sync_call(do_send, timeout=10.0)
        except Exception as e:
            self.get_logger().error(f"发布目标点失败: {e}")
            self.nav_goal_active = False
            self.nav_goal_reached = False
            self._nav_terminal_message = f"发布导航目标失败: {e}"
            return False

    def pause_navigation(self):
        """暂停导航（通过cancel当前goal实现）"""
        if self.current_goal_handle:
            self.get_logger().info("暂停导航...")
            def do_cancel():
                fut = self.current_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self.get_logger().info("导航已暂停（已取消当前目标）"))
                return True
            try:
                return self.schedule_sync_call(do_cancel, timeout=5.0)
            except Exception as e:
                self.get_logger().error(f"暂停导航失败: {e}")
                return False
        else:
            self.get_logger().warn("没有活跃的导航目标可暂停")

    def resume_navigation(self):
        """继续导航（重新发送上一个目标）"""
        if self.last_goal:
            self.get_logger().info("继续导航...")
            try:
                # publish_goal 内部已通过 schedule_sync_call(do_send) 自行调度到
                # executor 线程；不能再套一层 schedule_sync_call，否则外层占住
                # _cb_group_queue 的 executor 线程，内层永远无法被 _process_call_queue
                # 调度 → 嵌套死锁，表现为 10s 超时。
                return self.publish_goal(self.last_goal, target="resume")
            except Exception as e:
                self.get_logger().error(f"继续导航失败: {e}")
                return False
        else:
            self.get_logger().warn("没有可继续的目标")

    def cancel_navigation(self):
        """取消导航"""
        if self.current_goal_handle:
            self.get_logger().info("取消导航...")
            def do_cancel():
                fut = self.current_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda f: self.get_logger().info("导航已取消"))
                return True
            try:
                return self.schedule_sync_call(do_cancel, timeout=5.0)
            except Exception as e:
                self.get_logger().error(f"取消导航失败: {e}")
                return False
        else:
            self.get_logger().warn("没有活跃的导航目标可取消")

    def cancel_all_nav_goals(self, timeout_sec: float = 3.0) -> bool:
        """取消 ``navigate_to_pose`` action server 上的**全部**目标。

        与 ``cancel_navigation`` 不同：后者只能取消本节点持有 handle 的目标；而室外
        近距目标是别处发到 /goal_pose 再由 Nav2 执行的，本节点没有其 goal handle，
        必须直接调用 action 底层的 CancelGoal 服务。空请求（goal_id/stamp 全 0）按
        action 语义即“取消全部目标”，从而真正让室外 controller 停止驱动 /cmd_vel。
        """
        try:
            client = self._get_cached_client(
                CancelGoal, '/navigate_to_pose/_action/cancel_goal')
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warning(
                    '取消服务不可用: /navigate_to_pose/_action/cancel_goal')
                return False
            request = CancelGoal.Request()  # 全 0 → 取消全部目标
            future = client.call_async(request)
            start = time.time()
            while not future.done():
                if time.time() - start > timeout_sec:
                    self.get_logger().warning('取消全部导航目标超时')
                    return False
                time.sleep(0.05)
            resp = future.result()
            n = len(resp.goals_canceling) if resp is not None else 0
            self.get_logger().info(f'已请求取消全部导航目标（{n} 个正在取消）')
            return True
        except Exception as e:
            self.get_logger().error(f'取消全部导航目标失败: {e}')
            return False

    def wait_until_stopped(self, timeout: float = 4.0,
                           linear_tol: float = 0.05,
                           angular_tol: float = 0.05,
                           settle: float = 0.3,
                           poll: float = 0.05) -> Dict[str, Any]:
        """等底盘**真的**停下，而不是假设「取消目标 = 已停车」。

        取消目标只让 Nav2 开始减速：零速要经
        ``controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
        -> collision_monitor -> /cmd_vel``
        三跳才到底盘，且 velocity_smoother 按 ``max_decel``（-2.5 m/s²）**渐变**下来，
        0.8 m/s 要 ~0.3s 才归零。谁在这中间拆栈，最后一条 /cmd_vel 就是非零值，
        底盘会保持它继续溜车。

        所以这里直接盯 ``/motion_control/chassis_speed`` 的真实反馈。

        :param settle: 达标后再确认这么久，避免刚好采到减速过零的瞬间。
        :return: {stopped: True|False|None, waited: 秒, speed: 最后测得线速度}
                 ``stopped=None`` 表示没有底盘反馈可判（仿真/驱动未起），
                 此时退化为固定延时。
        """
        t0 = time.time()

        def _speed():
            cs = self.chassis_speed
            if cs is None:
                return None
            return (math.hypot(cs.linear.x, cs.linear.y), abs(cs.angular.z))

        if _speed() is None:
            # 没有反馈就无法确认，只能给足够时间让减速走完（velocity_smoother 的
            # velocity_timeout 是 1.0s，加上减速斜坡）。
            time.sleep(min(timeout, 1.5))
            self.get_logger().warning(
                '无底盘速度反馈(/motion_control/chassis_speed)，无法确认已停车，'
                f'按固定 {min(timeout, 1.5):.1f}s 延时处理')
            return {'stopped': None, 'waited': time.time() - t0, 'speed': None}

        ok_since = None
        last = None
        while time.time() - t0 < timeout:
            sp = _speed()
            if sp is None:
                break
            last = sp[0]
            if sp[0] <= linear_tol and sp[1] <= angular_tol:
                if ok_since is None:
                    ok_since = time.time()
                elif time.time() - ok_since >= settle:
                    waited = time.time() - t0
                    self.get_logger().info(
                        f'底盘已停车（线速度 {sp[0]:.3f} m/s，'
                        f'角速度 {sp[1]:.3f} rad/s），耗时 {waited:.2f}s')
                    return {'stopped': True, 'waited': waited, 'speed': sp[0]}
            else:
                ok_since = None
            time.sleep(poll)

        # 超时：Nav2 没把车停下来，自己补发零速（此时目标已取消，不会和谁抢）。
        self.cmd_vel_pub.publish(Twist())
        self.cmd_vel_sent = None
        waited = time.time() - t0
        self.get_logger().warning(
            f'等待停车超时（{timeout:.1f}s，最后线速度 '
            f'{"未知" if last is None else f"{last:.3f} m/s"}），已直接下发零速')
        return {'stopped': False, 'waited': waited, 'speed': last}

    def quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角
        
        参数:
        x, y, z, w: 四元数分量
        
        返回:
        yaw: 偏航角（弧度）
        """
        # 计算yaw角度
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        将角度标准化到[-pi, pi]范围
        
        参数:
        angle: 输入角度（弧度）
        
        返回:
        标准化后的角度
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def pid_control_step(self, yaw_tolerance=0.1, tolerance=0.5):
        """
        执行一步PID控制
        
        参数:
        yaw_tolerance: yaw角度容差（弧度）
        tolerance: 距离容差（米）
        
        返回:
        bool: True表示已到达目标，False表示仍在调整
        """
        if not self.pid_control_active:
            return False

        # 电梯升降中：暂停精调，发零速保证机器人不动，等 /elevator_moving=false 再继续
        if self.elevator_moving:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.cmd_vel_sent = None
            self.stuck_count = 0
            return False

        timestamp = self.get_clock().now().to_msg()

        if not (self.current_pose and self.last_goal):
            return False

        # 计算位置误差
        dx = self.last_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.last_goal.pose.position.y - self.current_pose.pose.position.y
        dist = (dx**2 + dy**2) ** 0.5

        if(dist > 3.0):
            self.get_logger().warn(f"当前位置与目标点距离过大，跳过PID控制步骤: dist={dist}")
            return False
        # 计算当前yaw角和目标yaw角
        current_yaw = self.quaternion_to_euler(
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )
        target_yaw = self.quaternion_to_euler(
            self.last_goal.pose.orientation.x,
            self.last_goal.pose.orientation.y,
            self.last_goal.pose.orientation.z,
            self.last_goal.pose.orientation.w
        )
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # 创建速度控制消息
        cmd_vel = Twist()

        if self.pid_rotation_phase:
            # 旋转阶段：调整yaw角度
            if abs(yaw_error) < yaw_tolerance:
                self.get_logger().debug(f"current quaternion: {self.current_pose.pose.orientation}")
                self.get_logger().warning(f"yaw角度调整完成: {current_yaw} -> {target_yaw}, error={yaw_error}, tolerance={yaw_tolerance}")
                self.pid_rotation_phase = False
                self.get_logger().info("yaw角度调整完成，切换到移动阶段")
                cmd_vel.angular.z = 0.0
            else:
                if( abs(yaw_error) > 0.1):
                    kp_yaw = 0.9
                elif( abs(yaw_error) > 0.08):
                    kp_yaw = 1.5
                else:
                    kp_yaw = 2.0
                cmd_vel.angular.z = kp_yaw * yaw_error
                if cmd_vel.angular.z > 0.5:
                    cmd_vel.angular.z = 0.5
                elif cmd_vel.angular.z < -0.5:
                    cmd_vel.angular.z = -0.5
                self.get_logger().warning(f"yaw_error={yaw_error},cmd={cmd_vel.angular.z},ts={timestamp.sec}.{timestamp.nanosec}")
        else:
            # 移动阶段：调整xy位置
            if dist < tolerance:
                # 位置达到要求，完成调整
                self.get_logger().warning(f"位置调整完成: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, dist={dist}, tolerance={tolerance}")
                self.pid_control_active = False
                self.get_logger().info("位置调整完成，到达目标")
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                return True
            else:
                # 继续调整位置
                kp_linear = 0.7  # 线性速度的比例控制系数
                x_local = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
                y_local = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
                cmd_vel.linear.x = kp_linear * x_local
                cmd_vel.linear.y = kp_linear * y_local
                self.get_logger().warning(f"dx={dx},dy={dy},xlocal={x_local},ylocal={y_local},cmdx={cmd_vel.linear.x},cmdy={cmd_vel.linear.y}")
                # 限制线性速度
                max_vel = 0.3
                if abs(cmd_vel.linear.x) > max_vel:
                    cmd_vel.linear.x = max_vel if cmd_vel.linear.x > 0 else -max_vel
                if abs(cmd_vel.linear.y) > max_vel:
                    cmd_vel.linear.y = max_vel if cmd_vel.linear.y > 0 else -max_vel

        # 发布速度控制指令
        self.cmd_vel_pub.publish(cmd_vel)
        # 记录发送的速度命令（用于卡住检测）
        self.cmd_vel_sent = cmd_vel
        
        # 检测是否卡住
        if self.is_robot_stuck():
            self.get_logger().error("机器人被卡住，停止PID控制")
            # 发送停止命令
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.cmd_vel_sent = None
            self.stuck_count = 0
            self.pid_control_active = False
            self.nav_goal_reached = False
            self._nav_terminal_message = "精调阶段检测到机器人卡住"
            return False

        # # 设置10ms后停止的定时器
        # if self.stop_timer is not None:
        #     self.stop_timer.cancel()  # 取消之前的定时器
            
        # self.stop_timer = self.create_timer(0.1, self.stop_robot_once)  # 10ms = 0.01s
            
        return False

    def stop_robot_once(self):
        """发送停止命令并取消定时器"""
        stop_cmd = Twist()  # 所有速度都为0
        self.cmd_vel_pub.publish(stop_cmd)
        
        # 取消定时器，防止重复执行
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
            
        self.get_logger().debug("机器人已停止（10ms后）")

    def check_goal_reached(self, tolerance=0.5, yaw_tolerance=0.1) -> Tuple[str, Optional[str]]:
        """
        检查是否到达目标点（或导航已失败）。

        参数:
        tolerance: 到达判断的距离阈值（米）
        yaw_tolerance: yaw角度容差（弧度），默认0.1

        返回:
        (状态, 失败原因):
            - (GOAL_CHECK_REACHED, None): 已到达
            - (GOAL_CHECK_PENDING, None): 仍在导航或精调中
            - (GOAL_CHECK_FAILED, msg): 导航失败/取消等，msg 为简短说明，上层应停止轮询
        """
        if self._nav_terminal_message:
            return (NavNode.GOAL_CHECK_FAILED, self._nav_terminal_message)

        # 确保有当前位置数据和目标点数据
        if not (self.current_pose and self.last_goal):
            return (NavNode.GOAL_CHECK_PENDING, None)

        # 如果已经在PID控制状态中，执行PID控制步骤
        if self.pid_control_active:
            # self.get_logger().info("pid_control_step")
            done = self.pid_control_step(yaw_tolerance, tolerance)
            if done:
                return (NavNode.GOAL_CHECK_REACHED, None)
            if self._nav_terminal_message:
                return (NavNode.GOAL_CHECK_FAILED, self._nav_terminal_message)
            return (NavNode.GOAL_CHECK_PENDING, None)

        # 计算当前位置与目标点的距离误差
        dx = self.last_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.last_goal.pose.position.y - self.current_pose.pose.position.y
        dist = (dx**2 + dy**2) ** 0.5

        # 计算当前yaw角和目标yaw角
        current_yaw = self.quaternion_to_euler(
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )
        target_yaw = self.quaternion_to_euler(
            self.last_goal.pose.orientation.x,
            self.last_goal.pose.orientation.y,
            self.last_goal.pose.orientation.z,
            self.last_goal.pose.orientation.w
        )
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # 检查是否同时满足距离和角度要求
        if dist < tolerance and abs(yaw_error) < yaw_tolerance:
            self.get_logger().info(f"目标已到达 - 距离: {dist:.3f}m, yaw误差: {abs(yaw_error):.3f}rad")
            return (NavNode.GOAL_CHECK_REACHED, None)

        # 检查Navigation2是否成功但精度不够
        if self.nav_goal_reached and not self.pid_control_active:
            self.get_logger().info("Navigation2已到达目标但精度不够，进入PID调整状态")
            self.pid_control_active = True
            self.pid_rotation_phase = True
            return (NavNode.GOAL_CHECK_PENDING, None)

        # 目标已结束且未进入/未完成成功路径：导航失败（兜底，防止回调未写入 _nav_terminal_message）
        if not self.nav_goal_active and not self.nav_goal_reached and not self.pid_control_active:
            self.get_logger().warning("导航目标不活跃且未到达")
            return (NavNode.GOAL_CHECK_FAILED, "导航未成功完成")

        # 导航仍在进行中
        return (NavNode.GOAL_CHECK_PENDING, None)

    def _get_cached_client(self, srv_type, srv_name: str):
        """按 ``(srv_type, srv_name)`` 复用 rclpy Client，避免每次调用都
        ``create_client`` 引发的 DDS endpoint 注册/销毁抖动。

        线程安全；同一节点生命周期内同 service 只创建一次。Node 销毁时
        rclpy 会随节点回收所有 client，无需在此手动 destroy。
        """
        key = (srv_type, srv_name)
        with self._svc_client_cache_lock:
            cli = self._svc_client_cache.get(key)
            if cli is None:
                cli = self.create_client(srv_type, srv_name)
                self._svc_client_cache[key] = cli
        return cli

    def get_current_map_path(self, timeout_sec: float = 2.0) -> str:
        """
        从 /grid_map_pcl_loader_node 节点获取 map_file_path 参数
        
        参数:
        timeout_sec: 服务调用超时时间（秒）
        
        返回:
        str: 当前地图文件路径，如果获取失败返回空字符串
        """
        try:
            client = self._get_cached_client(
                GetParameters,
                '/grid_map_pcl_loader_node/get_parameters',
            )

            # 等待服务可用
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warning('参数服务不可用: /grid_map_pcl_loader_node/get_parameters')
                return ''
            
            # 创建请求
            request = GetParameters.Request()
            request.names = ['map_file_path']
            
            # 发送请求
            future = client.call_async(request)
            
            # 使用轮询等待，避免与主 executor 冲突
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().warning('获取参数超时')
                    return ''
                time.sleep(0.05)
            
            if future.result() is not None:
                result = future.result()
                if result.values and len(result.values) > 0:
                    map_path = result.values[0].string_value
                    self.get_logger().info(f'当前地图路径: {map_path}')
                    return map_path
                else:
                    self.get_logger().warning('未找到 map_file_path 参数')
                    return ''
            else:
                self.get_logger().error('获取参数失败')
                return ''
                
        except Exception as e:
            self.get_logger().error(f'获取当前地图路径失败: {e}')
            return ''

    def change_map(self, map_file_path: str) -> bool:
        """
        切换地图，向 /change_map 主题发送新地图路径
        /grid_map_pcl_loader_node 和 /laser_mapping 节点会接收并处理
        
        参数:
        map_file_path: 新地图文件的完整路径
        
        返回:
        bool: 发送成功返回 True，失败返回 False
        """
        try:
            msg = String()
            msg.data = map_file_path
            self.change_map_pub.publish(msg)
            self.get_logger().info(f'已发送地图切换请求: {map_file_path}')
            # 此时地图尚未切换，FAST-LIO 仍在用旧地图正常输出 TF，不重置定位标志；
            # 定位标志的重置由 publish_pose_in_target 延迟 2s 触发。
            return True
        except Exception as e:
            self.get_logger().error(f'发送地图切换请求失败: {e}')
            return False

    def publish_pose_in_target(self, pose_dict: dict) -> bool:
        """
        发布切换地图后的初始位姿到 /pose_in_target 主题
        
        参数:
        pose_dict: 位姿字典，格式: {position: {x, y, z}, orientation: {x, y, z, w}}
        
        返回:
        bool: 发送成功返回 True，失败返回 False
        """
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            
            pos = pose_dict.get('position', {})
            ori = pose_dict.get('orientation', {})
            
            msg.pose.position.x = float(pos.get('x', 0.0))
            msg.pose.position.y = float(pos.get('y', 0.0))
            msg.pose.position.z = float(pos.get('z', 0.0))
            msg.pose.orientation.x = float(ori.get('x', 0.0))
            msg.pose.orientation.y = float(ori.get('y', 0.0))
            msg.pose.orientation.z = float(ori.get('z', 0.0))
            msg.pose.orientation.w = float(ori.get('w', 1.0))
            
            self.pose_in_target_pub.publish(msg)
            self.get_logger().info(f'已发布目标地图初始位姿: x={pos.get("x")}, y={pos.get("y")}')
            # 延迟 2s 再重置定位标志：此刻 FAST-LIO 刚收到 pose_in_target，
            # 还需要几秒完成重定位；立即置 False 会被 update_robot_pose 用旧 TF
            # 立即拉回 True，导致 server.py 的等待定位逻辑误判已就绪。
            prev = getattr(self, '_localization_reset_timer', None)
            if prev is not None:
                prev.cancel()
            def _do_reset():
                self._on_relocalization_request("已发布 pose_in_target（2s 后重置定位标志）")
                self._localization_reset_timer = None
            self._localization_reset_timer = threading.Timer(2.0, _do_reset)
            self._localization_reset_timer.daemon = True
            self._localization_reset_timer.start()
            return True
        except Exception as e:
            self.get_logger().error(f'发布目标地图初始位姿失败: {e}')
            return False

    def set_elevator_mode(self, enter: bool, timeout: float = 5.0) -> Tuple[bool, str]:
        """
        进入/退出电梯模式：调用 LIO 的 /elevator_mode 服务，进入后锁 Z 轴、切换历史帧累积；检测到升降时自动锁 XY。

        :param enter: True=进入电梯模式（锁 Z），False=退出（解锁 Z）
        :param timeout: 等待服务响应的超时（秒）
        :return: (success, message)
        """
        srv_name = '/elevator_mode'
        cli = self._get_cached_client(SetBool, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            msg = f'服务 {srv_name} 不可用'
            self.get_logger().error(msg)
            return False, msg

        req = SetBool.Request()
        req.data = enter
        future = cli.call_async(req)

        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                msg = f'调用 {srv_name} 超时'
                self.get_logger().error(msg)
                return False, msg
            time.sleep(0.05)

        result = future.result()
        if result is None:
            msg = f'调用 {srv_name} 失败（无结果）'
            self.get_logger().error(msg)
            return False, msg

        action = "进入" if enter else "退出"
        self.get_logger().info(f'{action}电梯模式: success={result.success}, {result.message}')
        return result.success, result.message

    def set_remote_parameter(
        self,
        node_name: str,
        param_name: str,
        value,
        timeout: float = 3.0,
    ) -> Tuple[bool, str]:
        """
        设置远程节点的 ROS2 参数。

        :param node_name: 目标节点全名，如 '/controller_server'
        :param param_name: 参数名，如 'general_goal_checker.xy_goal_tolerance'
        :param value: int / float / bool / str
        :param timeout: 超时秒数
        :return: (success, message)
        """
        srv = f'{node_name}/set_parameters'
        cli = self._get_cached_client(SetParameters, srv)
        if not cli.wait_for_service(timeout_sec=2.0):
            msg = f'服务 {srv} 不可用'
            self.get_logger().error(msg)
            return False, msg

        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        else:
            return False, f'不支持的参数类型: {type(value)}'

        param_msg = ParameterMsg()
        param_msg.name = param_name
        param_msg.value = pv

        req = SetParameters.Request()
        req.parameters = [param_msg]
        future = cli.call_async(req)

        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return False, f'调用 {srv} 超时'
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, f'调用 {srv} 失败（无结果）'

        r = result.results[0]
        if r.successful:
            self.get_logger().info(f'已设置 {node_name} 参数 {param_name} = {value}')
            return True, f'{param_name} = {value}'
        else:
            return False, f'设置失败: {r.reason}'