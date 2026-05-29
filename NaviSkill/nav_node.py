# ROS导航节点类
import rclpy  # ROS2 Python客户端库
from rclpy.node import Node  # ROS节点基类
from rcl_interfaces.srv import GetParameters, SetParameters  # 参数服务
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from rclpy.executors import SingleThreadedExecutor  # 单线程执行器
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # 回调组（MultiThreadedExecutor 并行调度）
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped  # 位姿消息类型和速度控制消息类型
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # 占据网格、里程计、全局路径
from sensor_msgs.msg import CompressedImage  # 压缩图像（用于 VPR 引导）
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

    def __init__(self, camera_image_topic: str = "/hdas/camera_head/left_raw/image_raw_color/compressed"):
        """初始化ROS节点和相关发布者/订阅者

        :param camera_image_topic: 用于 VPR 引导/重定位的相机 CompressedImage 话题，
                                    默认 ``/hdas/camera_head/left_raw/image_raw_color/compressed``
                                    （部署机 408 项目使用 hdas head 左目 raw 压缩图像）。
                                    注意这与 vpr_preprocess.py 解析离线 rosbag 时
                                    使用的 ``/camera_agent/img_left/compressed`` 不同；
                                    数据库本身只关心特征空间，不要求 live 话题与 bag
                                    话题同名，只要相机参数与去畸变配置一致即可。
        """
        # 先确保rclpy已经初始化
        if not rclpy.ok():
            rclpy.init()
        
        # 调用父类构造函数，设置节点名称
        super().__init__('nav_node')

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
                    'odom', 'base_link',
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
                return self.schedule_sync_call(lambda: self.publish_goal(self.last_goal, target="resume"), timeout=10.0)
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
                self.get_logger().debug(f"yaw_error={yaw_error},cmd={cmd_vel.angular.z},ts={timestamp.sec}.{timestamp.nanosec}")
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
                self.get_logger().debug(f"dx={dx},dy={dy},xlocal={x_local},ylocal={y_local},cmdx={cmd_vel.linear.x},cmdy={cmd_vel.linear.y}")
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