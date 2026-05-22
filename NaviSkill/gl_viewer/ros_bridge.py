"""GL Viewer 独立 ROS 桥（用 rclpy 在后台 spin 线程订阅）。

- ``/cloud_registered`` (PointCloud2) 或 livox CustomMsg 作为实时 lidar
- TF ``map`` → ``base_link`` 作为机器人位姿（submap-local）

Viewer 主线程不直接调 Qt 之外的事；通过 ``get_state()`` / ``get_latest_lidar()``
拿快照。这跟 NaviSkill.nav_node / NaviSkill.server 完全独立。
"""

from __future__ import annotations
import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger("gl_viewer.ros_bridge")


@dataclass
class RobotState:
    """机器人在 root 系下的当前位姿（已分解为 submap-local + submap_id）。"""
    submap_id: str
    local_pos: np.ndarray     # (3,) float32
    local_quat: np.ndarray    # (4,) float32 xyzw
    stamp_ns: int             # ros 时间戳，给 viewer 判"有没有更新"


@dataclass
class LidarFrame:
    """一帧已处理好的 lidar（spin 线程预先 downsample + z-着色完毕）。"""
    submap_id: str            # 该帧逻辑上属于哪个 submap（用 robot 当前 submap）
    interleaved: np.ndarray   # (N, 6) float32  pos3 + color3，可直接 glBufferSubData
    seq: int                  # 单调递增，给 viewer 判"是否需要重传 VBO"


class RosBridge:
    """rclpy 后台 spin 线程 + 一份 lidar / 机器人状态快照。

    Usage:
        bridge = RosBridge(map_manager, lidar_topic="/cloud_registered")
        bridge.start()
        ...
        state = bridge.get_robot_state()
        lidar = bridge.get_latest_lidar()
        ...
        bridge.stop()
    """

    def __init__(self, map_manager, lidar_topic: str = "/cloud_registered",
                 tf_lookup_hz: float = 2.0,
                 lidar_max_hz: float = 5.0,
                 max_lidar_points: int = 20_000):
        self._map_manager = map_manager
        self._lidar_topic = lidar_topic
        self._tf_lookup_interval = 1.0 / max(0.5, tf_lookup_hz)
        self._lidar_min_interval = 1.0 / max(0.5, lidar_max_hz)
        self._max_lidar_points = int(max_lidar_points)

        self._lock = threading.Lock()
        # lidar
        self._lidar_xyz: Optional[np.ndarray] = None         # 兼容字段，不一定用
        self._lidar_interleaved: Optional[np.ndarray] = None  # 预处理好的 (N, 6) f32
        self._latest_lidar_seq: int = 0
        self._latest_lidar_stamp_ns: int = 0
        self._last_lidar_kept_t: float = 0.0
        self._submap_for_lidar: str = ""

        # robot
        self._robot: Optional[RobotState] = None
        self._robot_event_seq: int = 0  # 只在真"动了"时才 +1，避免重绘空转

        # 性能计数
        self._cnt_lidar: int = 0
        self._cnt_tf: int = 0

        self._rclpy_thread: Optional[threading.Thread] = None
        self._tf_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._node = None
        self._executor = None
        self._lidar_sub = None

    # ── 启动 / 停止 ─────────────────────────────────────────────────

    def start(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        class _ViewerNode(Node):
            def __init__(self_inner):
                super().__init__("gl_viewer_bridge")

        self._node = _ViewerNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        try:
            self._subscribe_lidar(self._node, self._lidar_topic)
        except Exception:
            logger.warning("订阅 lidar topic=%s 失败（可能尚未发布），继续运行", self._lidar_topic)

        # tf
        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer(cache_time=None)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)

        self._rclpy_thread = threading.Thread(
            target=self._spin_loop, name="gl_viewer_rclpy", daemon=True)
        self._rclpy_thread.start()
        self._tf_thread = threading.Thread(
            target=self._tf_loop, name="gl_viewer_tf", daemon=True)
        self._tf_thread.start()

        logger.info("gl_viewer ROS bridge 启动完毕 (lidar=%s)", self._lidar_topic)

    def stop(self) -> None:
        self._stop_evt.set()
        try:
            if self._executor:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass

    # ── 订阅 lidar ──────────────────────────────────────────────────

    def _subscribe_lidar(self, node, topic: str) -> None:
        """根据 topic publisher 类型（PointCloud2 / Livox CustomMsg）二选一订阅。"""
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        # 等到 publisher 出现，识别消息类型
        topic_type = None
        for _ in range(20):
            infos = node.get_publishers_info_by_topic(topic)
            if infos:
                tnames = [i.topic_type for i in infos]
                # 优先 PointCloud2，再 CustomMsg
                if any("PointCloud2" in t for t in tnames):
                    topic_type = "sensor_msgs/msg/PointCloud2"
                    break
                if any("CustomMsg" in t for t in tnames):
                    topic_type = tnames[0]
                    break
            time.sleep(0.5)
        if not topic_type:
            logger.warning("topic %s 暂未发现 publisher (5s)，先按 PointCloud2 订阅", topic)
            topic_type = "sensor_msgs/msg/PointCloud2"

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if topic_type.endswith("/PointCloud2"):
            from sensor_msgs.msg import PointCloud2
            def cb(msg):
                self._on_lidar(msg, kind="pc2")
            self._lidar_sub = node.create_subscription(PointCloud2, topic, cb, qos)
            logger.info("订阅 %s 为 PointCloud2 (BEST_EFFORT/KEEP_LAST 1, throttle %.1fHz)",
                        topic, 1.0 / self._lidar_min_interval)
        else:
            # livox custom msg
            try:
                mod = __import__("livox_ros_driver2.msg", fromlist=["CustomMsg"])
            except Exception:
                mod = __import__("livox_ros_driver", fromlist=["CustomMsg"])
            CustomMsg = getattr(mod, "CustomMsg")
            def cb2(msg):
                self._on_lidar(msg, kind="livox_custom")
            self._lidar_sub = node.create_subscription(CustomMsg, topic, cb2, qos)
            logger.info("订阅 %s 为 livox CustomMsg (throttle %.1fHz)",
                        topic, 1.0 / self._lidar_min_interval)

    def _on_lidar(self, msg, kind: str) -> None:
        # throttle
        now = time.monotonic()
        if now - self._last_lidar_kept_t < self._lidar_min_interval:
            return
        self._last_lidar_kept_t = now

        try:
            if kind == "pc2":
                from sensor_msgs_py import point_cloud2
                arr = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True)
                if arr.size == 0:
                    return
                xyz = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
            else:  # livox CustomMsg
                points = msg.points
                n = len(points)
                if n == 0:
                    return
                xyz = np.empty((n, 3), dtype=np.float32)
                for i, p in enumerate(points):
                    xyz[i, 0] = p.x; xyz[i, 1] = p.y; xyz[i, 2] = p.z
        except Exception:
            logger.exception("lidar 解析失败")
            return

        # stride 下采样
        n = len(xyz)
        if n > self._max_lidar_points:
            stride = int(np.ceil(n / self._max_lidar_points))
            xyz = xyz[::stride]
            n = len(xyz)

        # z-彩虹着色（5 段 jet：蓝→青→绿→黄→红，比单调 rainbow 更鲜艳）
        z = xyz[:, 2]
        z_lo, z_hi = float(np.percentile(z, 5.0)), float(np.percentile(z, 95.0))
        denom = max(0.3, z_hi - z_lo)
        t = np.clip((z - z_lo) / denom, 0.0, 1.0).astype(np.float32)
        # Jet colormap 解析公式（值域 [0, 1]，饱和度高）：
        r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0)
        g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0)
        b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0)
        rgb = np.stack([r, g, b], axis=1).astype(np.float32)

        interleaved = np.concatenate([xyz, rgb], axis=1).astype(np.float32, copy=False)

        # 时间戳
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        with self._lock:
            self._lidar_xyz = xyz
            self._lidar_interleaved = interleaved
            self._latest_lidar_seq += 1
            self._latest_lidar_stamp_ns = stamp_ns
            # lidar 默认归属：当前 robot submap，没有就 map_manager 的 current
            self._submap_for_lidar = (
                self._robot.submap_id if self._robot
                else (self._map_manager.get_current_submap() or "")
            )
            self._cnt_lidar += 1

    # ── 后台 spin ───────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        try:
            while not self._stop_evt.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:
            logger.exception("rclpy spin 线程异常退出")

    # ── tf_lookup_hz 周期查询 map → base_link，归到 current submap ──

    def _tf_loop(self) -> None:
        """
        周期查询 TF，把 map→base_link 拆成「当前 submap + submap-local」。
        只在位姿真正变化时 bump robot_event_seq；这样静止时 viewer 不会被持续触发重绘。
        如果 TF 暂时不可用（换图时常见），保留上一帧 stamp 即可（viewer 不会以此判定改变）。
        """
        from rclpy.time import Time
        from rclpy.duration import Duration
        pos_eps = 0.005   # 5 mm
        quat_eps = 0.005  # ~0.3 deg

        while not self._stop_evt.is_set():
            try:
                t = self._tf_buffer.lookup_transform(
                    "map", "base_link", Time(),
                    timeout=Duration(seconds=0.05))
                transform = t.transform
                pos = np.array([
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ], dtype=np.float32)
                quat = np.array([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ], dtype=np.float32)
                sm_id = self._infer_current_submap(pos)
                if sm_id:
                    stamp_ns = int(t.header.stamp.sec) * 1_000_000_000 + int(t.header.stamp.nanosec)
                    with self._lock:
                        prev = self._robot
                        changed = (
                            prev is None
                            or prev.submap_id != sm_id
                            or float(np.linalg.norm(prev.local_pos - pos)) > pos_eps
                            or float(np.linalg.norm(prev.local_quat - quat)) > quat_eps
                        )
                        self._robot = RobotState(
                            submap_id=sm_id,
                            local_pos=pos,
                            local_quat=quat,
                            stamp_ns=stamp_ns,
                        )
                        if changed:
                            self._robot_event_seq += 1
                        self._cnt_tf += 1
            except Exception:
                pass
            self._stop_evt.wait(self._tf_lookup_interval)

    def _infer_current_submap(self, pos: np.ndarray) -> str:
        """
        优先 map_manager.get_current_submap()。
        换图后 server.py 会 set_current_submap 同步；这里只兜底：找最近 submap 原点。
        """
        cur = self._map_manager.get_current_submap()
        if cur:
            return cur
        # fallback：到 submap 原点最近的
        best, best_d = "", float("inf")
        for sm_id in self._map_manager.get_submap_list():
            tf = self._map_manager.get_submap_transform(sm_id) or {}
            t = tf.get("translation", {"x": 0.0, "y": 0.0, "z": 0.0})
            origin = np.array([t["x"], t["y"], t["z"]], dtype=np.float32)
            d = float(np.linalg.norm(origin - pos))
            if d < best_d:
                best_d, best = d, sm_id
        return best

    # ── 公开快照 API ────────────────────────────────────────────────

    def get_robot_state(self) -> Optional[RobotState]:
        with self._lock:
            return self._robot

    def peek_lidar_seq(self) -> int:
        """让 viewer O(1) 判断 lidar 是否有新数据（无需 copy numpy）。"""
        with self._lock:
            return self._latest_lidar_seq

    def peek_robot_stamp(self) -> int:
        with self._lock:
            return self._robot_event_seq

    def get_counters(self, reset: bool = False) -> Tuple[int, int]:
        """返回 (lidar_count, tf_count)。reset=True 时复位（用于算 FPS 等）。"""
        with self._lock:
            ld, tf = self._cnt_lidar, self._cnt_tf
            if reset:
                self._cnt_lidar = 0
                self._cnt_tf = 0
        return ld, tf

    def get_latest_lidar(self) -> Optional[LidarFrame]:
        """O(1)：直接返回 spin 线程已 interleaved 好的快照。"""
        with self._lock:
            if self._lidar_interleaved is None:
                return None
            return LidarFrame(
                submap_id=self._submap_for_lidar,
                interleaved=self._lidar_interleaved,
                seq=self._latest_lidar_seq,
            )
