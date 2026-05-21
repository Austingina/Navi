"""NaviSkill 可视化模块（基于 rerun-sdk）。

在 server.py 启动时通过 ``-v / --viz`` 开启。功能：

  1. 加载所有 submap 的点云，按 ``MapManager.get_submap_transform`` 对齐到 root 系
  2. 实时机器人位姿（``nav_node.current_pose`` × ``current_submap → root``）
  3. 实时雷达点云（默认 ``/cloud_registered``，FAST_LIO 输出的已去畸变+已在 world/odom
     系的 PointCloud2；也可用 ``--viz_lidar_topic /livox/lidar`` 切到原始传感器系点云）
  4. 所有 routes 的 waypoint 折线（高亮 default_route）；可调 ``highlight_route(name)``
  5. 2D 栅格地图 ``/map`` + Nav2 全局路径 ``/plan`` + 当前机器人位置

设计要点
--------
* rerun-sdk 是可选依赖（同名的 ``rerun`` 包是无关工具，需要 ``rerun-sdk``）。
  未安装时模块仍可 import，仅在 ``NaviRerunViz.start()`` 时抛出明确错误。
* 静态层（点云、route）只 log 一次；换 submap 时机器人 frame 自动跟新变换。
* 一个后台 worker 线程做动态层刷新（10 Hz），不阻塞 server 主路径。
* lidar→base_link 外参做 lazy lookup 并 cache；查不到时退化为直接挂到 robot frame。
* 单 submap 28M 点会卡 viewer，统一下采样到 ``max_submap_points``（默认 40 万）。

Entity path 约定
----------------
``/world``                    根 frame（root submap 系），右手 Z 上
``/world/submaps/<id>``       Transform3D：submap → root
``/world/submaps/<id>/points`` 该 submap 点云（每 submap 配独立颜色）
``/world/submaps/<id>/axes``  0.5m 三轴指示
``/world/robot``              Transform3D：base_link → root，每 100ms 更新
``/world/robot/box``          机器人外形（半透 box + 朝向箭头）
``/world/robot/lidar``        当前 LiDAR 帧（Points3D），跟随 robot 自动到正确位置
``/world/routes/<name>``      LineStrips3D + Points3D（waypoints）
``/map2d``                    2D 栅格地图（rerun 2D image）
``/map2d/plan``               Nav2 全局路径（LineStrips2D）
``/map2d/robot``              机器人在栅格图上的当前位置（Points2D）
"""

from __future__ import annotations

import io
import logging
import os
import re
import struct
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


logger = logging.getLogger("navi.viz")


# ═══════════════════════════════════════════════════════════════════════
#  rerun 懒加载（同名包 rerun 是无关工具，需 rerun-sdk）
# ═══════════════════════════════════════════════════════════════════════

def _import_rerun():
    try:
        import rerun as rr
    except ImportError as e:
        raise ImportError(
            "未安装 rerun-sdk。请在 server.py 的 Python 环境中执行：\n"
            "    pip3 install --user rerun-sdk\n"
            "注意：不是同名的 'rerun'（那是一个无关的文件变化检测器）。"
        ) from e
    # 同名包 rerun 不提供 Points3D/Transform3D，作语义检查
    if not hasattr(rr, "Points3D") or not hasattr(rr, "Transform3D"):
        raise ImportError(
            "检测到的 'rerun' 不是 rerun-sdk。请：\n"
            "    pip3 uninstall rerun -y\n"
            "    pip3 install --user rerun-sdk"
        )
    return rr


def _rr_connect(rr, addr: str) -> None:
    """rerun-sdk 不同版本里 connect 的函数名/签名略有差异，做一层兼容。"""
    host, _, port = addr.partition(":")
    port = int(port) if port else 9876
    if hasattr(rr, "connect_tcp"):
        rr.connect_tcp(f"{host}:{port}")
    elif hasattr(rr, "connect"):
        rr.connect(addr=f"{host}:{port}")
    else:
        raise RuntimeError("rerun-sdk 既无 connect_tcp 也无 connect 方法")


def _rr_serve(rr, addr: str) -> Dict[str, Any]:
    """启动 SDK server，让 viewer 主动 connect 过来。

    返回一个 dict 描述实际启的 server 类型，用于给用户提示正确的连接命令：
        { "scheme": "tcp"|"grpc"|"ws", "host": <显示用 IP>, "port": <int>,
          "web_port": Optional[int] }   (web_port 仅 ws 老 API 有)
    """
    host, _, port = addr.partition(":")
    port = int(port) if port else 9876
    show_host = host if host not in ("", "0.0.0.0", "::") else _detect_local_ip()
    if hasattr(rr, "serve_grpc"):
        rr.serve_grpc(bind_ip=host, grpc_port=port)
        return {"scheme": "grpc", "host": show_host, "port": port, "web_port": None}
    if hasattr(rr, "serve_tcp"):
        rr.serve_tcp(bind_ip=host, port=port)
        return {"scheme": "tcp", "host": show_host, "port": port, "web_port": None}
    if hasattr(rr, "serve"):
        # 老 API（rerun-sdk 0.21 等）：启 WebSocket + 内置 web viewer
        web_port = 8080
        rr.serve(open_browser=False, web_port=web_port, ws_port=port)
        return {"scheme": "ws", "host": show_host, "port": port, "web_port": web_port}
    raise RuntimeError("rerun-sdk 未提供 serve_* API")


def _detect_local_ip() -> str:
    """尽力探测本机一个对外可访问的 IPv4，用于日志提示。失败回退到 hostname。"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 8.8.8.8 不真的发包，只是触发路由表选择；UDP 不会 connect 失败
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "<本机IP>"


# ═══════════════════════════════════════════════════════════════════════
#  PCD 解析（优先 open3d；否则手写 binary/ascii 解析，仅取 xyz）
# ═══════════════════════════════════════════════════════════════════════

_PCD_TYPE_TO_NUMPY = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
    ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
}


def _parse_pcd_xyz(path: str) -> Optional[np.ndarray]:
    """读取 PCD 的 xyz 三列；ASCII / binary（不支持 binary_compressed）均支持。"""
    try:
        import open3d as o3d  # type: ignore
        pcd = o3d.io.read_point_cloud(path)
        if pcd.has_points():
            return np.asarray(pcd.points, dtype=np.float32)
        return None
    except ImportError:
        pass

    # ── 手写 fallback ───────────────────────────────────────────────
    try:
        with open(path, "rb") as f:
            header_bytes = io.BytesIO()
            while True:
                line = f.readline()
                if not line:
                    return None
                header_bytes.write(line)
                if line.startswith(b"DATA"):
                    break
            header = header_bytes.getvalue().decode("ascii", errors="ignore")

            def _re(key, default=None):
                m = re.search(rf"^\s*{key}\s+(.+)$", header, re.MULTILINE)
                return m.group(1).strip() if m else default

            fields = (_re("FIELDS") or "").split()
            sizes  = list(map(int, (_re("SIZE")  or "").split()))
            types  = (_re("TYPE")  or "").split()
            counts = list(map(int, (_re("COUNT") or "1 " * len(fields)).split()))
            points = int(_re("POINTS") or 0)
            data_kind = (_re("DATA") or "ascii").lower()

            try:
                xi = fields.index("x"); yi = fields.index("y"); zi = fields.index("z")
            except ValueError:
                return None
            if points <= 0:
                return None

            if data_kind == "ascii":
                # 用 numpy.loadtxt 读 xyz 三列
                arr = np.loadtxt(f, dtype=np.float32, usecols=(xi, yi, zi))
                return arr.reshape(-1, 3) if arr.ndim == 1 else arr
            if data_kind == "binary":
                dtype_fields = []
                for name, tp, sz, c in zip(fields, types, sizes, counts):
                    np_tp = _PCD_TYPE_TO_NUMPY.get((tp, sz))
                    if np_tp is None:
                        return None
                    if c == 1:
                        dtype_fields.append((name, np_tp))
                    else:
                        dtype_fields.append((name, np_tp, c))
                dt = np.dtype(dtype_fields)
                buf = np.fromfile(f, dtype=dt, count=points)
                if buf.size == 0:
                    return None
                return np.column_stack([buf["x"], buf["y"], buf["z"]]).astype(np.float32)
            # binary_compressed 不支持
            logger.warning(f"viz: PCD 数据类型 {data_kind} 不支持: {path}")
            return None
    except Exception as e:
        logger.warning(f"viz: PCD 解析失败 {path}: {e}")
        return None


# ═══════════════════════════════════════════════════════════════════════
#  四元数 / Pose 数学（轻量，避免拉 scipy）
# ═══════════════════════════════════════════════════════════════════════

def _quat_xyzw(q_dict: Dict[str, float]) -> np.ndarray:
    """{x,y,z,w} → np.array([x,y,z,w]) float32"""
    return np.array([q_dict.get("x", 0.0), q_dict.get("y", 0.0),
                     q_dict.get("z", 0.0), q_dict.get("w", 1.0)], dtype=np.float32)


def _quat_to_mat3(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y + z*z),     s*(x*y - z*w),     s*(x*z + y*w)],
        [    s*(x*y + z*w), 1 - s*(x*x + z*z),     s*(y*z - x*w)],
        [    s*(x*z - y*w),     s*(y*z + x*w), 1 - s*(x*x + y*y)],
    ], dtype=np.float32)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """q1 ⊗ q2，xyzw 表示"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=np.float32)


def _apply_tf(translation: Dict[str, float], rotation: Dict[str, float],
              pts: np.ndarray) -> np.ndarray:
    """对 (N,3) 点应用 T = (t, q_xyzw)：p' = R*p + t"""
    R = _quat_to_mat3(_quat_xyzw(rotation))
    t = np.array([translation.get("x", 0.0), translation.get("y", 0.0),
                  translation.get("z", 0.0)], dtype=np.float32)
    return pts @ R.T + t


# ═══════════════════════════════════════════════════════════════════════
#  调色板
# ═══════════════════════════════════════════════════════════════════════

_SUBMAP_PALETTE = [
    (240, 170, 80),  (120, 200, 255), (180, 240, 180), (255, 180, 210),
    (220, 220, 220), (255, 240, 140), (160, 220, 220), (255, 150, 120),
]
_ROUTE_PALETTE = [
    (255, 220, 60),  (80, 220, 255),  (255, 140, 220), (180, 255, 140),
    (255, 180, 80),  (140, 200, 255), (200, 255, 200), (255, 200, 200),
]


# ═══════════════════════════════════════════════════════════════════════
#  主类
# ═══════════════════════════════════════════════════════════════════════

class NaviRerunViz:
    """rerun 可视化编排：静态层 log 一次，动态层 worker 线程定时刷新。"""

    def __init__(
        self,
        *,
        nav_node,
        map_manager,
        app_id: str = "NaviSkill",
        mode: str = "spawn",
        connect_addr: str = "127.0.0.1:9876",
        serve_addr: str = "0.0.0.0:9876",
        save_path: Optional[str] = None,
        recording_id: Optional[str] = None,
        lidar_topic: str = "/cloud_registered",
        update_hz: float = 10.0,
        lidar_hz: float = 5.0,
        map_hz: float = 1.0,
        max_submap_points: int = 400_000,
        max_lidar_points: int = 80_000,
    ):
        """
        :param mode: 连接模式
            - ``spawn``  : 在本机自动起一个 rerun viewer 子进程并 TCP connect
                          （需要有效的 X11 / Wayland 显示；headless 机会失败）
            - ``connect``: 连接已经在跑的 rerun viewer，``connect_addr`` 形如 ``host:port``
            - ``serve``  : 不起 viewer，仅起 SDK TCP server 监听 ``serve_addr``，
                          让你从别的机器 ``rerun --connect tcp://<jetson_ip>:9876`` 连过来
            - ``save``   : 不联网，把所有数据写到 ``save_path`` .rrd 文件，事后离线看
        """
        self._nav_node = nav_node
        self._map_manager = map_manager
        self._app_id = app_id
        self._mode = mode
        self._connect_addr = connect_addr
        self._serve_addr = serve_addr
        self._save_path = save_path
        self._recording_id = recording_id
        self._lidar_topic = lidar_topic
        self._update_interval = 1.0 / max(1.0, update_hz)
        self._lidar_interval = 1.0 / max(0.5, lidar_hz)
        self._map_interval = 1.0 / max(0.1, map_hz)
        self._max_pts = int(max_submap_points)
        self._max_lidar_pts = int(max_lidar_points)

        self._rr = None
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._started = False

        # 缓存：上次已 log 过点云的 submap 集合（避免无谓重传）
        self._submap_clouds_logged: set = set()
        # 机器人外观（box / heading / axes）只 log 一次
        self._robot_visual_logged: bool = False
        # lidar→base_link 外参 cache
        self._lidar_base_tf: Optional[dict] = None
        self._lidar_base_tf_tried_mono: float = 0.0
        # 上次发栅格地图的标记，避免大幅刷新
        self._last_map_stamp: Tuple[int, int] = (0, 0)
        self._last_map_t: float = 0.0
        # lidar 限频 / 跳采状态
        self._last_lidar_seq: int = -1
        self._next_lidar_t: float = 0.0
        # 当前高亮 route 名（None = 不高亮）
        self._active_route: Optional[str] = None
        self._route_meta: Dict[str, Dict[str, Any]] = {}

    # ── 生命周期 ────────────────────────────────────────────────────

    def start(self) -> None:
        if self._started:
            return
        rr = _import_rerun()
        self._rr = rr

        # 1) init recording（不带 spawn，避免阻塞 + 失败行为不可控）
        init_kwargs: Dict[str, Any] = {"spawn": False}
        if self._recording_id:
            init_kwargs["recording_id"] = self._recording_id
        rr.init(self._app_id, **init_kwargs)

        # 2) 按 mode 选数据传输方式
        mode = (self._mode or "spawn").lower()
        if mode == "spawn":
            try:
                rr.spawn()  # 起 viewer 子进程 + 隐式 connect
                logger.info("viz: 已 spawn rerun viewer 子进程（如果窗口未出现，"
                            "通常是 wayland/X11 显示环境问题——试试 --viz_mode serve "
                            "或在桌面 terminal 里运行）")
            except Exception as e:
                logger.error(
                    "viz: spawn viewer 失败: %s\n"
                    "  → 自动降级为 serve 模式（不画窗口，等远端 viewer 连接）", e
                )
                mode = "serve"
        if mode == "connect":
            try:
                _rr_connect(rr, self._connect_addr)
                logger.info("viz: 已 connect 已有 rerun viewer @ %s", self._connect_addr)
            except Exception as e:
                logger.error("viz: connect %s 失败: %s", self._connect_addr, e)
        elif mode == "serve":
            try:
                info = _rr_serve(rr, self._serve_addr)
                scheme = info["scheme"]
                show_host = info["host"]
                port = info["port"]
                if scheme == "ws":
                    web_url = f"http://{show_host}:{info['web_port']}?url=ws://{show_host}:{port}"
                    native_cmd = f"rerun ws://{show_host}:{port}"
                    logger.info(
                        "viz: 已启动 rerun WebSocket server @ %s:%d (老 API)\n"
                        "  → 浏览器直接看：%s\n"
                        "  → 或在 PC 上：    %s",
                        show_host, port, web_url, native_cmd,
                    )
                elif scheme == "grpc":
                    logger.info(
                        "viz: 已启动 rerun gRPC server @ %s:%d\n"
                        "  → 在 PC 上：  rerun --connect rerun+http://%s:%d/proxy",
                        show_host, port, show_host, port,
                    )
                else:  # tcp
                    logger.info(
                        "viz: 已启动 rerun SDK TCP server @ %s:%d\n"
                        "  → 在 PC 上：  rerun --connect tcp://%s:%d",
                        show_host, port, show_host, port,
                    )
            except Exception as e:
                logger.error("viz: serve %s 失败: %s", self._serve_addr, e)
        elif mode == "save":
            path = self._save_path or "/tmp/navi_rerun.rrd"
            try:
                rr.save(path)
                logger.info("viz: 数据写入 %s（事后用 `rerun %s` 离线查看）", path, path)
            except Exception as e:
                logger.error("viz: save %s 失败: %s", path, e)

        # 全局坐标系：右手 Z 上（ROS 习惯）
        rr.log("/world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self._log_global_axes()

        # 静态层
        try:
            self._log_submap_clouds()
        except Exception:
            logger.exception("viz: 加载 submap 点云失败（继续运行其他层）")
        try:
            self._log_routes()
        except Exception:
            logger.exception("viz: 加载 routes 失败")

        # 按需启用 lidar 订阅（NavNode 内部 cache 最新一帧）
        try:
            ok = self._nav_node.enable_lidar_subscription_for_viz(self._lidar_topic)
            if not ok:
                logger.warning("viz: lidar 订阅未启用（详情见日志），雷达视图将为空")
        except Exception as e:
            logger.warning(f"viz: 启用 lidar 订阅出错: {e}")

        # worker 线程
        self._worker = threading.Thread(target=self._loop, name="navi-rerun-viz", daemon=True)
        self._worker.start()
        self._started = True
        logger.info(
            "viz: rerun 已启动 (app_id=%s, mode=%s, lidar=%s, robot_hz=%.1f, lidar_hz=%.1f)",
            self._app_id, self._mode, self._lidar_topic,
            1.0 / self._update_interval, 1.0 / self._lidar_interval,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._worker:
            self._worker.join(timeout=2.0)
        # rerun 自身的资源由 atexit 处理

    # ── 公共 API ─────────────────────────────────────────────────────

    def highlight_route(self, route_name: Optional[str]) -> None:
        """将某条 route 设为高亮（粗线 + 满色）；传 None 则取消高亮。"""
        if self._rr is None:
            return
        self._active_route = route_name
        # 重新 log 所有 routes 以更新颜色/线宽
        try:
            self._log_routes()
        except Exception:
            logger.exception("viz: 高亮 route 时重 log 失败")

    def trigger_reload_static(self) -> None:
        """供外部在地图重载后调用：清空已 log 标记并重灌静态层。"""
        self._submap_clouds_logged.clear()
        self._log_submap_clouds()
        self._log_routes()

    # ── 静态层 ─────────────────────────────────────────────────────

    def _log_global_axes(self) -> None:
        rr = self._rr
        rr.log(
            "/world/axes",
            rr.Arrows3D(
                vectors=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                labels=["X", "Y", "Z"],
            ),
            static=True,
        )

    def _log_submap_clouds(self) -> None:
        rr = self._rr
        if self._map_manager is None:
            return

        for idx, sm_id in enumerate(self._map_manager.get_submap_list()):
            if sm_id in self._submap_clouds_logged:
                continue
            try:
                pcd_path = self._map_manager.get_submap_full_map_path(sm_id)
            except Exception:
                pcd_path = None
            if not pcd_path or not Path(pcd_path).exists():
                logger.warning(f"viz: submap {sm_id} 找不到点云文件: {pcd_path}")
                continue

            pts = _parse_pcd_xyz(pcd_path)
            if pts is None or len(pts) == 0:
                logger.warning(f"viz: submap {sm_id} 点云为空: {pcd_path}")
                continue
            n0 = len(pts)
            if n0 > self._max_pts:
                rng = np.random.default_rng(seed=42)
                idx_keep = rng.choice(n0, size=self._max_pts, replace=False)
                pts = pts[idx_keep]

            tf = self._map_manager.get_submap_transform(sm_id) or {}
            t = tf.get("translation", {"x": 0.0, "y": 0.0, "z": 0.0})
            r = tf.get("rotation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
            entity = f"/world/submaps/{sm_id}"
            rr.log(
                entity,
                rr.Transform3D(
                    translation=[float(t["x"]), float(t["y"]), float(t["z"])],
                    rotation=rr.Quaternion(xyzw=_quat_xyzw(r).tolist()),
                ),
                static=True,
            )
            color = _SUBMAP_PALETTE[idx % len(_SUBMAP_PALETTE)]
            rr.log(
                f"{entity}/points",
                rr.Points3D(positions=pts, colors=[color], radii=0.02),
                static=True,
            )
            # 局部三轴：每个 submap 原点 0.5 m
            rr.log(
                f"{entity}/axes",
                rr.Arrows3D(
                    vectors=[[0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.5]],
                    colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
                ),
                static=True,
            )
            self._submap_clouds_logged.add(sm_id)
            logger.info(
                f"viz: 已加载 submap {sm_id}: 原始 {n0} 点 → 渲染 {len(pts)} 点  ({pcd_path})"
            )

    def _resolve_waypoint_world(self, sub_id: str, pt_name: str) -> Optional[np.ndarray]:
        """waypoint → root 系 (3,) float32；查不到返回 None。"""
        if self._map_manager is None:
            return None
        info = self._map_manager.get_pose_in_submap(pt_name, sub_id)
        if not info:
            return None
        pos = info.get("pose", {}).get("position") if isinstance(info, dict) else None
        if pos is None:
            # 兜底：直接拿 info.position
            pos = info.get("position") if isinstance(info, dict) else None
        if not pos:
            return None
        local = np.array([[float(pos.get("x", 0.0)),
                           float(pos.get("y", 0.0)),
                           float(pos.get("z", 0.0))]], dtype=np.float32)
        tf = self._map_manager.get_submap_transform(sub_id) or {}
        t = tf.get("translation", {})
        r = tf.get("rotation", {})
        return _apply_tf(t, r, local)[0]

    def _log_routes(self) -> None:
        rr = self._rr
        if self._map_manager is None:
            return
        try:
            routes = self._map_manager.get_route_list() or []
            default_name = self._map_manager.get_default_route()
        except Exception:
            routes = []
            default_name = None

        # 先把之前 log 过的 routes "清掉"（避免上次 highlight 后残留颜色）
        for name in list(self._route_meta.keys()):
            try:
                rr.log(f"/world/routes/{name}", rr.Clear(recursive=True))
            except Exception:
                pass
        self._route_meta.clear()

        for i, r in enumerate(routes):
            name = r.get("name")
            if not name:
                continue
            full = self._map_manager.get_route_by_name(name)
            if not full:
                continue
            wps = full.get("waypoints", [])
            world_pts: List[np.ndarray] = []
            for wp in wps:
                sub_id = wp.get("submap")
                pt_name = wp.get("semantic_point")
                if not sub_id or not pt_name:
                    continue
                p = self._resolve_waypoint_world(sub_id, pt_name)
                if p is None:
                    continue
                world_pts.append(p)
            if len(world_pts) < 1:
                continue
            world_pts = np.stack(world_pts, axis=0)

            base_color = _ROUTE_PALETTE[i % len(_ROUTE_PALETTE)]
            is_active = (self._active_route == name)
            is_default = (default_name == name)
            # 高亮：active > default > 普通
            if is_active:
                color = tuple(min(255, int(c * 1.2)) for c in base_color)
                line_radius = 0.10
                pt_radius = 0.22
                alpha = 255
            elif is_default:
                color = base_color
                line_radius = 0.07
                pt_radius = 0.18
                alpha = 255
            else:
                color = base_color
                line_radius = 0.04
                pt_radius = 0.13
                alpha = 150
            rgba = (*color, alpha)

            entity = f"/world/routes/{name}"
            if len(world_pts) >= 2:
                rr.log(
                    f"{entity}/line",
                    rr.LineStrips3D(strips=[world_pts.tolist()],
                                    colors=[rgba], radii=line_radius),
                    static=True,
                )
            rr.log(
                f"{entity}/waypoints",
                rr.Points3D(positions=world_pts, colors=[rgba], radii=pt_radius),
                static=True,
            )
            self._route_meta[name] = {
                "active": is_active, "default": is_default, "count": len(world_pts),
            }
        if routes:
            logger.info(
                "viz: routes 已加载 %d 条 (default=%s, active=%s)",
                len(routes), default_name, self._active_route,
            )

    # ── 动态层 (worker) ─────────────────────────────────────────────

    def _loop(self) -> None:
        next_map = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._publish_robot_pose()
            except Exception:
                logger.exception("viz: 发布机器人位姿失败")

            # lidar 限频：默认 5 Hz；同 seq 的帧不重复发
            if t0 >= self._next_lidar_t:
                try:
                    self._publish_lidar()
                except Exception:
                    logger.exception("viz: 发布雷达失败")
                self._next_lidar_t = t0 + self._lidar_interval

            # 2D 栅格 + plan 更低频
            if t0 >= next_map:
                try:
                    self._publish_grid_and_plan()
                except Exception:
                    logger.exception("viz: 发布栅格地图失败")
                next_map = t0 + self._map_interval

            elapsed = time.monotonic() - t0
            sleep = self._update_interval - elapsed
            if sleep > 0:
                self._stop.wait(sleep)

    # ── 机器人位姿（base_link → root） ─────────────────────────────

    def _publish_robot_pose(self) -> None:
        rr = self._rr
        pose = getattr(self._nav_node, "current_pose", None)
        if pose is None or self._map_manager is None:
            return
        cur_submap = self._map_manager.get_current_submap()
        if not cur_submap:
            return

        # pose 是当前 submap 内 map 系下的 PoseStamped
        local_pos = np.array([[float(pose.pose.position.x),
                               float(pose.pose.position.y),
                               float(pose.pose.position.z)]], dtype=np.float32)
        local_quat = np.array([float(pose.pose.orientation.x),
                               float(pose.pose.orientation.y),
                               float(pose.pose.orientation.z),
                               float(pose.pose.orientation.w)], dtype=np.float32)

        tf = self._map_manager.get_submap_transform(cur_submap) or {}
        t = tf.get("translation", {})
        r = tf.get("rotation", {})
        # 平移
        world_pos = _apply_tf(t, r, local_pos)[0]
        # 旋转：q_world = q_submap_to_root ⊗ q_local
        world_quat = _quat_mul(_quat_xyzw(r), local_quat)

        # 时间戳：用机器人 ros time（保证 viewer time-cursor 对齐）
        try:
            stamp_ns = int(pose.header.stamp.sec) * 1_000_000_000 + int(pose.header.stamp.nanosec)
            if stamp_ns > 0:
                rr.set_time_nanos("ros_time", stamp_ns)
        except Exception:
            pass

        rr.log(
            "/world/robot",
            rr.Transform3D(
                translation=world_pos.tolist(),
                rotation=rr.Quaternion(xyzw=world_quat.tolist()),
            ),
        )
        # 静态外观：第一次发位姿时一并 log（之后 robot frame 跟着 transform 动）
        if not self._robot_visual_logged:
            rr.log(
                "/world/robot/box",
                rr.Boxes3D(
                    centers=[[0.0, 0.0, 0.15]],
                    half_sizes=[[0.30, 0.25, 0.18]],
                    colors=[[80, 220, 80, 180]],
                ),
                static=True,
            )
            rr.log(
                "/world/robot/heading",
                rr.Arrows3D(
                    origins=[[0.0, 0.0, 0.15]],
                    vectors=[[0.6, 0.0, 0.0]],
                    colors=[[255, 50, 50]],
                ),
                static=True,
            )
            rr.log(
                "/world/robot/axes",
                rr.Arrows3D(
                    vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
                    colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
                ),
                static=True,
            )
            self._robot_visual_logged = True

    # ── 雷达（PointCloud2 / Livox CustomMsg） ──────────────────────

    # z 彩虹 LUT（低→高: 蓝→青→绿→黄→红），用于让实时雷达点云"跳出"
    # 浅色的静态 submap 背景；4 ms / 80k 点。
    _LIDAR_Z_STOPS = np.array(
        [
            (40,  80, 255),   # 蓝
            (60, 200, 230),   # 青
            (120, 230, 100),  # 绿
            (255, 220, 80),   # 黄
            (255,  80, 60),   # 红
        ],
        dtype=np.float32,
    )

    @classmethod
    def _color_by_z(cls, xyz: np.ndarray) -> np.ndarray:
        """按 z 高度做 5-stop 渐变，返回 (N,3) uint8。"""
        z = xyz[:, 2]
        # 用 2/98 分位避免单个离群点把色阶拉死
        z_lo = float(np.percentile(z, 2.0))
        z_hi = float(np.percentile(z, 98.0))
        denom = max(1e-6, z_hi - z_lo)
        t = np.clip((z - z_lo) / denom, 0.0, 1.0)
        stops = cls._LIDAR_Z_STOPS
        seg = t * (len(stops) - 1)
        i0 = np.clip(seg.astype(np.int32), 0, len(stops) - 2)
        f = (seg - i0).reshape(-1, 1)
        return (stops[i0] * (1.0 - f) + stops[i0 + 1] * f).astype(np.uint8)

    # 已在 world/odom 系下的点云 topic 关键字（不做 lidar→base_link 外参变换，
    # 不挂在机器人 Transform3D 下，否则会被位姿再变换一次造成双重抖动）
    _WORLD_FRAME_TOPIC_HINTS = (
        "/cloud_registered",      # FAST_LIO 输出（去畸变 + 已在 world/odom）
        "/cloud_registered_body", # FAST_LIO 同帧 body frame 版本（也已是 world？看版本，保险按 world 处理）
        "/lio_sam/mapping/cloud_registered",
        "/Laser_map",
        "/map_points",
    )

    @classmethod
    def _topic_is_world_frame(cls, topic: str) -> bool:
        topic = (topic or "").rstrip("/")
        return any(topic.endswith(h) or h in topic for h in cls._WORLD_FRAME_TOPIC_HINTS)

    def _publish_lidar(self) -> None:
        rr = self._rr
        if not hasattr(self._nav_node, "get_latest_lidar_xyz"):
            return
        ret = self._nav_node.get_latest_lidar_xyz()
        if len(ret) == 4:
            xyz, frame_id, stamp_ns, seq = ret
        else:
            xyz, frame_id, stamp_ns = ret
            seq = -1
        if xyz is None or len(xyz) == 0:
            return
        if seq >= 0 and seq == self._last_lidar_seq:
            return
        self._last_lidar_seq = seq

        # 抽稀
        if len(xyz) > self._max_lidar_pts:
            rng = np.random.default_rng()
            xyz = xyz[rng.choice(len(xyz), self._max_lidar_pts, replace=False)]

        is_world = self._topic_is_world_frame(self._lidar_topic)
        if not is_world:
            # ─ 传感器系点云（如 /livox/lidar）：做 lidar→base_link，再挂到 robot 节点下
            if self._lidar_base_tf is None:
                now = time.monotonic()
                if now - self._lidar_base_tf_tried_mono > 5.0 and frame_id:
                    self._lidar_base_tf_tried_mono = now
                    if hasattr(self._nav_node, "lookup_static_transform"):
                        self._lidar_base_tf = self._nav_node.lookup_static_transform(
                            "base_link", frame_id, timeout_sec=0.3
                        )
                        if self._lidar_base_tf is not None:
                            logger.info(
                                "viz: 已 cache lidar→base_link 外参 (frame=%s, t=%s)",
                                frame_id, self._lidar_base_tf["translation"],
                            )
                        else:
                            logger.warning(
                                "viz: 暂时拿不到 base_link←%s 的 TF；雷达点云先按"
                                " lidar 即 base_link 假设挂载，后续重试",
                                frame_id,
                            )
            if self._lidar_base_tf is not None:
                xyz = _apply_tf(
                    self._lidar_base_tf["translation"],
                    self._lidar_base_tf["rotation"],
                    xyz,
                )
            entity = "/world/robot/lidar"
        else:
            # ─ 世界系点云（如 /cloud_registered）：已是 world/odom，直接 log
            entity = "/world/lidar"

        if stamp_ns > 0:
            try:
                rr.set_time_nanos("ros_time", stamp_ns)
            except Exception:
                pass

        # 醒目化：每点按 z 染色 + 加大点径，与浅色 submap 形成强对比
        colors = self._color_by_z(xyz)
        rr.log(
            entity,
            rr.Points3D(positions=xyz, colors=colors, radii=0.06),
        )

    # ── 栅格地图 + 全局路径 + 机器人在 2D 视图上的位置 ─────────────

    def _publish_grid_and_plan(self) -> None:
        rr = self._rr
        ros_map = getattr(self._nav_node, "current_map", None)
        if ros_map is None:
            return
        info = ros_map.info
        res = float(info.resolution)
        w = int(info.width)
        h = int(info.height)
        if w <= 0 or h <= 0 or res <= 0:
            return
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        cur_stamp = (int(ros_map.header.stamp.sec), int(ros_map.header.stamp.nanosec))

        # 仅当 map header stamp 变化或久未刷新时才上传新图（避免大图反复转）
        now = time.monotonic()
        if cur_stamp == self._last_map_stamp and (now - self._last_map_t) < 5.0:
            need_send_image = False
        else:
            need_send_image = True
            self._last_map_stamp = cur_stamp
            self._last_map_t = now

        if need_send_image:
            # ROS OccupancyGrid: int8 [-1 unknown, 0 free, 100 occupied]
            raw = np.asarray(ros_map.data, dtype=np.int8).reshape(h, w)
            # ROS map 第一行对应世界 y=origin_y（下方），rerun image 坐标 y 朝下；
            # 为了视觉一致（"图像上方=世界 +y"），把行翻转一下
            img = np.empty((h, w), dtype=np.uint8)
            img[raw == -1] = 128       # unknown 灰
            img[raw == 0]  = 240       # free  浅灰
            occ = raw > 0
            img[occ] = 30              # occupied 深
            img = np.flipud(img)
            rr.log("/map2d/grid", rr.Image(img), static=False)

        # 把世界 (mx, my) 转到 image 像素 (px, py)
        def _world_to_px(mx: float, my: float) -> Tuple[float, float]:
            px = (mx - origin_x) / res
            py = h - 1 - (my - origin_y) / res
            return (px, py)

        # 当前机器人位置（用 nav_node 在 map frame 下的 current_pose 直接转）
        pose = getattr(self._nav_node, "current_pose", None)
        if pose is not None:
            rx, ry = _world_to_px(pose.pose.position.x, pose.pose.position.y)
            # 把 heading 也画成一条短线
            # 取四元数 z/w 算 yaw（平面）
            q = pose.pose.orientation
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            arrow_len_m = 0.8
            ex_m = pose.pose.position.x + arrow_len_m * float(np.cos(yaw))
            ey_m = pose.pose.position.y + arrow_len_m * float(np.sin(yaw))
            ex, ey = _world_to_px(ex_m, ey_m)
            rr.log(
                "/map2d/robot",
                rr.Points2D(positions=[[rx, ry]], colors=[[80, 220, 80]], radii=6.0),
            )
            rr.log(
                "/map2d/robot_heading",
                rr.LineStrips2D(strips=[[[rx, ry], [ex, ey]]],
                                colors=[[255, 60, 60]], radii=2.0),
            )

        # Nav2 全局路径（poses 在 map frame）
        latest_plan = getattr(self._nav_node, "_latest_plan", None)
        if latest_plan is not None and getattr(latest_plan, "poses", None):
            pts2d = []
            for p in latest_plan.poses:
                px, py = _world_to_px(p.pose.position.x, p.pose.position.y)
                pts2d.append([px, py])
            if len(pts2d) >= 2:
                rr.log(
                    "/map2d/plan",
                    rr.LineStrips2D(strips=[pts2d],
                                    colors=[[255, 220, 60]], radii=2.5),
                )
        else:
            # plan 已被清空（新 goal / 取消）→ 清除画面残留
            rr.log("/map2d/plan", rr.Clear(recursive=False))
