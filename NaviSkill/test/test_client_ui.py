#!/usr/bin/env python3
"""
带 UI 的 WebSocket 巡检/导航测试客户端

对接 server.py 的 /ws 接口（JSON-RPC 风格：{id, method, params} / {id, result}）。

功能：
1. 显示栅格地图
2. 实时显示机器人位置
3. 路线巡检（开始/暂停/继续/停止）
4. 单点导航（set_goal_by_name / set_goal_by_xy）
"""

import argparse
import asyncio
import json
import logging
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Any, Dict, List, Optional

import matplotlib
import numpy as np
import websockets

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)
logging.getLogger("websockets").setLevel(logging.WARNING)


# ---------------------------------------------------------------------------
# WebSocket 客户端：单连接、串行请求、自动重连
# ---------------------------------------------------------------------------
class WsNavClient:
    """与 server.py 的 /ws 通信的轻量客户端。

    - 一条长连接复用所有请求（避免 1Hz 状态轮询时反复握手）。
    - 内部 lock 保证 send/recv 一一对应，避免响应错乱。
    - 失败时仅在"连接状态切换"或"长时间未连接"时打印日志，避免刷屏。
    - 提供 on_state_change 回调，便于 UI 显示连接状态。
    """

    # 连接失败后的最小重试间隔（秒），避免对死端口高速重连
    RECONNECT_MIN_INTERVAL = 2.0

    def __init__(self, url: str):
        self.url = self._normalize(url)
        self._ws: Any = None
        self._req_id = 0
        self._lock = asyncio.Lock()

        # 连接状态：True=已连上，False=未连上
        self._connected = False
        # 上次失败时间戳，用于节流重连
        self._last_fail_ts = 0.0
        # 上次失败原因，用于变更日志
        self._last_fail_msg: Optional[str] = None
        # 连接状态变更回调：fn(connected: bool, msg: Optional[str])
        self.on_state_change = None

    @staticmethod
    def _normalize(url: str) -> str:
        url = url.rstrip("/")
        if url.startswith("http://"):
            url = "ws://" + url[7:]
        elif url.startswith("https://"):
            url = "wss://" + url[8:]
        if not (url.startswith("ws://") or url.startswith("wss://")):
            url = "ws://" + url
        if not url.endswith("/ws"):
            url = url + "/ws"
        return url

    @property
    def connected(self) -> bool:
        return self._connected

    def _emit_state(self, connected: bool, msg: Optional[str] = None) -> None:
        if self.on_state_change is None:
            return
        try:
            self.on_state_change(connected, msg)
        except Exception:
            logger.debug("on_state_change 回调异常", exc_info=True)

    def _set_connected(self) -> None:
        was = self._connected
        self._connected = True
        self._last_fail_msg = None
        if not was:
            logger.info("✅ 已连接到服务端: %s", self.url)
            self._emit_state(True, None)

    def _set_disconnected(self, msg: str) -> None:
        was = self._connected
        self._connected = False
        self._last_fail_ts = time.monotonic()
        # 仅在状态切换 或 失败原因变化时打印，其它静默
        if was:
            logger.warning("❌ 与服务端断开 (%s): %s", self.url, msg)
            self._emit_state(False, msg)
        elif msg != self._last_fail_msg:
            logger.warning("⏳ 仍未连接到服务端 (%s): %s", self.url, msg)
            self._emit_state(False, msg)
        self._last_fail_msg = msg

    async def _ensure_connected(self) -> None:
        if self._ws is not None:
            return
        # 节流：连接失败后 RECONNECT_MIN_INTERVAL 秒内不重试
        if not self._connected:
            since = time.monotonic() - self._last_fail_ts
            if since < self.RECONNECT_MIN_INTERVAL:
                raise ConnectionError(
                    f"连接冷却中 (剩余 {self.RECONNECT_MIN_INTERVAL - since:.1f}s)"
                )
        self._ws = await websockets.connect(
            self.url, ping_interval=20, ping_timeout=60, max_size=None
        )
        self._set_connected()

    async def close(self) -> None:
        async with self._lock:
            if self._ws is not None:
                try:
                    await self._ws.close()
                except Exception:
                    pass
                self._ws = None

    async def _drop_ws(self) -> None:
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

    async def call(
        self,
        method: str,
        params: Optional[Dict[str, Any]] = None,
        timeout: float = 30.0,
    ) -> Dict[str, Any]:
        """发送一次 RPC 调用并等待响应。返回 dict（保证含 success 字段）。

        失败时不再无脑双倍重试：第一次出现连接错误 → 丢连接重连一次；仍失败则直接返回。
        """
        last_err: Optional[str] = None
        for attempt in range(2):
            try:
                async with self._lock:
                    await self._ensure_connected()
                    self._req_id += 1
                    rid = self._req_id
                    payload = {"id": rid, "method": method, "params": params or {}}
                    await self._ws.send(json.dumps(payload, ensure_ascii=False))
                    raw = await asyncio.wait_for(self._ws.recv(), timeout=timeout)
                data = json.loads(raw)
                if data.get("error"):
                    return {"success": False, "msg": str(data["error"])}
                if "result" in data:
                    r = data["result"]
                    if isinstance(r, dict):
                        return r
                    return {"success": True, "data": r}
                return {"success": False, "msg": f"未知响应: {data}"}
            except (websockets.ConnectionClosed, asyncio.TimeoutError, OSError) as e:
                last_err = str(e) or e.__class__.__name__
                async with self._lock:
                    await self._drop_ws()
                self._set_disconnected(last_err)
                # 已经在 _ensure_connected 里冷却失败的，没必要再走第二次
                if isinstance(e, OSError) and not self._connected:
                    break
            except ConnectionError as e:
                # 来自 _ensure_connected 的冷却拒绝，直接返回
                return {"success": False, "msg": str(e)}
            except Exception as e:
                return {"success": False, "msg": f"RPC 异常: {e}"}
        return {"success": False, "msg": f"连接异常: {last_err}"}


# ---------------------------------------------------------------------------
# 主 UI
# ---------------------------------------------------------------------------
class NavClientUI:
    def __init__(self, ws_url: str = "ws://localhost:9999/ws"):
        self.ws_url = ws_url
        self.client: Optional[WsNavClient] = None

        # 地图数据
        self.map_data: Optional[np.ndarray] = None
        self.map_info: Optional[Dict[str, Any]] = None
        self.map_bound: Optional[Dict[str, Any]] = None

        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.localized = False

        # 点位 / 路线
        self.pose_list: List[str] = []
        self.route_names: List[str] = []
        self.route_waypoints: List[Any] = []  # 元素可能是 str 或 dict
        self.current_waypoint_idx = 0

        # 巡检/暂停状态
        self.is_patrolling = False  # True 表示巡检循环正在运行
        self.is_paused = False      # True 表示已发出 pause_navigation 且未 resume

        # 地图选点模式
        self.xy_select_mode = False

        # 异步事件循环（独立线程）
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.async_thread: Optional[threading.Thread] = None

        # 静态数据是否已加载（连接成功后惰性补拉）
        self._static_loaded = False

        self.create_ui()
        self.start_async_loop()

        self.client = WsNavClient(self.ws_url)
        self.client.on_state_change = self._on_conn_state_change
        self.schedule_async(self.init_data())
        self.start_status_update()

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------
    def create_ui(self) -> None:
        self.root = tk.Tk()
        self.root.title("Navigation Test Client (WebSocket)")
        self.root.geometry("1280x820")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(main_frame, width=320)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_frame.pack_propagate(False)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # === 路线巡检 ===
        patrol_frame = ttk.LabelFrame(left_frame, text="路线巡检", padding=10)
        patrol_frame.pack(fill=tk.X, pady=(0, 10))

        route_select_frame = ttk.Frame(patrol_frame)
        route_select_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(route_select_frame, text="路线:").pack(side=tk.LEFT)
        self.route_var = tk.StringVar()
        self.route_combo = ttk.Combobox(
            route_select_frame, textvariable=self.route_var, values=[], width=18, state="readonly"
        )
        self.route_combo.pack(side=tk.LEFT, padx=(5, 0))
        self.route_combo.bind("<<ComboboxSelected>>", lambda _e: self.schedule_async(self._load_route_preview()))

        btn_frame = ttk.Frame(patrol_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        self.patrol_btn = ttk.Button(btn_frame, text="开始巡检", command=self.toggle_patrol)
        self.patrol_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.stop_patrol_btn = ttk.Button(
            btn_frame, text="停止", command=self.stop_patrol, state=tk.DISABLED
        )
        self.stop_patrol_btn.pack(side=tk.LEFT)

        ttk.Label(patrol_frame, text="巡检点位:").pack(anchor=tk.W)
        self.waypoint_text = scrolledtext.ScrolledText(
            patrol_frame, height=10, width=34, state=tk.DISABLED
        )
        self.waypoint_text.pack(fill=tk.X, pady=(5, 0))

        self.patrol_status_var = tk.StringVar(value="状态: 空闲")
        ttk.Label(patrol_frame, textvariable=self.patrol_status_var).pack(anchor=tk.W, pady=(5, 0))

        # === API 导航 ===
        nav_frame = ttk.LabelFrame(left_frame, text="API 导航", padding=10)
        nav_frame.pack(fill=tk.BOTH, expand=True)

        goal_frame = ttk.Frame(nav_frame)
        goal_frame.pack(fill=tk.X, pady=(0, 10))
        self.set_goal_btn = ttk.Button(
            goal_frame, text="set_goal_by_name", command=self.on_set_goal, width=18
        )
        self.set_goal_btn.pack(side=tk.LEFT)
        self.goal_var = tk.StringVar()
        self.goal_combo = ttk.Combobox(goal_frame, textvariable=self.goal_var, width=14)
        self.goal_combo.pack(side=tk.LEFT, padx=(5, 0))

        xy_frame = ttk.Frame(nav_frame)
        xy_frame.pack(fill=tk.X, pady=(0, 10))
        self.set_goal_xy_btn = ttk.Button(
            xy_frame, text="set_goal_by_xy", command=self.on_set_goal_xy, width=18
        )
        self.set_goal_xy_btn.pack(side=tk.LEFT)
        self.xy_status_var = tk.StringVar(value="点击按钮后在地图选点")
        ttk.Label(xy_frame, textvariable=self.xy_status_var, foreground="gray").pack(
            side=tk.LEFT, padx=(5, 0)
        )

        cancel_frame = ttk.Frame(nav_frame)
        cancel_frame.pack(fill=tk.X, pady=(0, 10))
        self.cancel_btn = ttk.Button(
            cancel_frame, text="cancel_navigation", command=self.on_cancel_navigation, width=18
        )
        self.cancel_btn.pack(side=tk.LEFT)

        ttk.Separator(nav_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        self.nav_status_var = tk.StringVar(value="导航状态: 空闲")
        ttk.Label(nav_frame, textvariable=self.nav_status_var).pack(anchor=tk.W)

        self.robot_pos_var = tk.StringVar(value="位置: (-, -)  yaw: -")
        ttk.Label(nav_frame, textvariable=self.robot_pos_var).pack(anchor=tk.W, pady=(5, 0))

        self.localized_var = tk.StringVar(value="定位: 未知")
        ttk.Label(nav_frame, textvariable=self.localized_var).pack(anchor=tk.W, pady=(2, 0))

        self.conn_var = tk.StringVar(value=f"连接: 未连接 ({self.ws_url})")
        self.conn_label = ttk.Label(nav_frame, textvariable=self.conn_var, foreground="orange")
        self.conn_label.pack(anchor=tk.W, pady=(2, 0))

        # === 右侧地图 ===
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self._init_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_map_click)

        self.robot_marker = None
        self.robot_arrow = None
        self.target_marker = None

    def _init_axes(self) -> None:
        self.ax.set_title("Map View")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # 异步线程
    # ------------------------------------------------------------------
    def start_async_loop(self) -> None:
        loop_ready = threading.Event()

        def run_loop() -> None:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            loop_ready.set()
            self.loop.run_forever()

        self.async_thread = threading.Thread(target=run_loop, daemon=True)
        self.async_thread.start()
        loop_ready.wait()

    def schedule_async(self, coro) -> None:
        if self.loop:
            asyncio.run_coroutine_threadsafe(coro, self.loop)

    # ------------------------------------------------------------------
    # 连接状态
    # ------------------------------------------------------------------
    def _on_conn_state_change(self, connected: bool, msg: Optional[str]) -> None:
        """WsNavClient 连接状态变化回调（在异步线程里被调）。"""
        if connected:
            self.root.after(0, lambda: self.conn_var.set(f"连接: 已连接 ({self.ws_url})"))
            self.root.after(0, lambda: self.conn_label.configure(foreground="green"))
            # 连上后惰性补拉一次静态数据
            if not self._static_loaded:
                self.schedule_async(self._load_static_data())
        else:
            text = f"连接: 未连接 ({self.ws_url})"
            if msg:
                text += f"  [{msg}]"
            self.root.after(0, lambda t=text: self.conn_var.set(t))
            self.root.after(0, lambda: self.conn_label.configure(foreground="red"))

    # ------------------------------------------------------------------
    # 初始化数据
    # ------------------------------------------------------------------
    async def init_data(self) -> None:
        await self._load_static_data()

    async def _load_static_data(self) -> None:
        """拉取地图 / 点位列表 / 路线列表。任意一项失败都不会抛出，等下次连上再试。"""
        ok_all = True

        if self.map_data is None:
            map_resp = await self.client.call("get_map", timeout=60.0)
            if map_resp.get("success"):
                try:
                    self.map_info = map_resp["info"]
                    self.map_bound = map_resp.get("bound")
                    self.map_data = np.array(map_resp["data"]).reshape(
                        self.map_info["height"], self.map_info["width"]
                    )
                    logger.info(
                        "地图加载成功: %dx%d",
                        self.map_info["width"],
                        self.map_info["height"],
                    )
                    self.root.after(0, self.update_map_display)
                except Exception as e:
                    logger.exception("解析地图数据失败: %s", e)
                    ok_all = False
            else:
                ok_all = False

        if not self.pose_list:
            pose_resp = await self.client.call("get_pose_list_in_current_submap")
            if pose_resp.get("success"):
                self.pose_list = list(pose_resp.get("keys") or [])
                logger.info("点位列表 (%d): %s", len(self.pose_list), self.pose_list)
                self.root.after(0, lambda: self.goal_combo.configure(values=self.pose_list))
                if self.pose_list:
                    self.root.after(0, lambda: self.goal_var.set(self.pose_list[0]))
            else:
                ok_all = False

        if not self.route_names:
            routes_resp = await self.client.call("get_route_names")
            if routes_resp.get("success"):
                self.route_names = list(routes_resp.get("route_names") or [])
                logger.info("路线列表 (%d): %s", len(self.route_names), self.route_names)
                self.root.after(0, lambda: self.route_combo.configure(values=self.route_names))

                default_route = None
                dr_resp = await self.client.call("get_default_route")
                if dr_resp.get("success"):
                    default_route = dr_resp.get("default_route")

                pick = default_route if default_route in self.route_names else (
                    self.route_names[0] if self.route_names else ""
                )
                if pick:
                    self.root.after(0, lambda r=pick: self.route_var.set(r))
                    self.schedule_async(self._load_route_preview())
            else:
                ok_all = False

        if ok_all and self.map_data is not None and self.pose_list and self.route_names:
            self._static_loaded = True

    async def _load_route_preview(self) -> None:
        """根据当前选中的路线名拉取并显示 waypoints（不开始导航）。"""
        name = self.route_var.get()
        if not name:
            return
        resp = await self.client.call("get_route", {"route_name": name})
        if not resp.get("success"):
            logger.warning("获取路线 %s 失败: %s", name, resp.get("msg"))
            return
        wps = list(resp.get("waypoints") or [])
        # 仅在未巡检时刷新预览，避免巡检过程被覆盖
        if not self.is_patrolling:
            self.route_waypoints = wps
            self.current_waypoint_idx = 0
            self.root.after(0, lambda: self._update_waypoint_text(self._format_waypoints(wps)))

    # ------------------------------------------------------------------
    # 地图与机器人显示
    # ------------------------------------------------------------------
    def update_map_display(self) -> None:
        if self.map_data is None or self.map_info is None:
            return

        self.ax.clear()
        origin_x = self.map_info["origin"]["x"]
        origin_y = self.map_info["origin"]["y"]
        resolution = self.map_info["resolution"]
        width = self.map_info["width"]
        height = self.map_info["height"]
        extent = [origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution]

        display_map = np.copy(self.map_data).astype(float)
        display_map[display_map == -1] = 50  # 未知 → 中灰
        self.ax.imshow(
            display_map, cmap="gray_r", origin="lower", extent=extent, vmin=0, vmax=100, alpha=0.85
        )
        self._init_axes()

        # 重置缓存的图元（已被 ax.clear 清掉了）
        self.robot_marker = None
        self.robot_arrow = None
        self.target_marker = None

        self.draw_robot()
        self.canvas.draw()

    def draw_robot(self) -> None:
        if self.map_data is None:
            return

        if self.robot_marker is not None:
            try:
                self.robot_marker.remove()
            except Exception:
                pass
            self.robot_marker = None
        if self.robot_arrow is not None:
            try:
                self.robot_arrow.remove()
            except Exception:
                pass
            self.robot_arrow = None

        (self.robot_marker,) = self.ax.plot(
            self.robot_x, self.robot_y, "ro", markersize=10, label="Robot"
        )
        arrow_len = 0.5
        dx = arrow_len * np.cos(self.robot_yaw)
        dy = arrow_len * np.sin(self.robot_yaw)
        self.robot_arrow = self.ax.arrow(
            self.robot_x,
            self.robot_y,
            dx,
            dy,
            head_width=0.2,
            head_length=0.1,
            fc="red",
            ec="red",
        )
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # 状态轮询
    # ------------------------------------------------------------------
    def start_status_update(self) -> None:
        self.schedule_async(self._async_update_status())
        self.root.after(1000, self.start_status_update)

    async def _async_update_status(self) -> None:
        try:
            data = await self.client.call("get_status", timeout=5.0)
            if not data.get("success"):
                return
            pos = data["position"]
            ori = data["orientation"]
            self.robot_x = pos["x"]
            self.robot_y = pos["y"]
            self.robot_yaw = float(
                np.arctan2(
                    2.0 * (ori["w"] * ori["z"] + ori["x"] * ori["y"]),
                    1.0 - 2.0 * (ori["y"] ** 2 + ori["z"] ** 2),
                )
            )
            self.localized = bool(data.get("localized", False))

            self.root.after(
                0,
                lambda: self.robot_pos_var.set(
                    f"位置: ({self.robot_x:.2f}, {self.robot_y:.2f})  yaw: {self.robot_yaw:.2f}"
                ),
            )
            self.root.after(
                0,
                lambda: self.localized_var.set(
                    f"定位: {'已定位' if self.localized else '未定位'}"
                ),
            )
            self.root.after(0, self.draw_robot)
        except Exception as e:
            logger.debug("获取状态失败: %s", e)

    # ------------------------------------------------------------------
    # 路线巡检
    # ------------------------------------------------------------------
    @staticmethod
    def _wp_name(wp: Any) -> str:
        if isinstance(wp, dict):
            sub = wp.get("submap")
            name = wp.get("semantic_point") or wp.get("point") or wp.get("name") or ""
            return f"{sub}::{name}" if sub else str(name)
        return str(wp)

    def _format_waypoints(self, wps: List[Any]) -> str:
        return "\n".join(f"{i + 1}. {self._wp_name(wp)}" for i, wp in enumerate(wps))

    def toggle_patrol(self) -> None:
        if not self.is_patrolling and not self.is_paused:
            self.start_patrol()
        elif self.is_patrolling and not self.is_paused:
            self.pause_patrol()
        else:
            # is_paused == True
            self.resume_patrol()

    def start_patrol(self) -> None:
        if not self.route_var.get():
            messagebox.showwarning("提示", "请先选择路线")
            return
        logger.info("开始巡检")
        self.is_patrolling = True
        self.is_paused = False
        self.current_waypoint_idx = 0
        self.patrol_btn.configure(text="暂停")
        self.stop_patrol_btn.configure(state=tk.NORMAL)
        self.schedule_async(self._async_patrol(start_idx=0))

    def pause_patrol(self) -> None:
        logger.info("暂停巡检")
        self.is_patrolling = False
        self.patrol_btn.configure(text="继续")
        self.schedule_async(self._async_pause_navigation())

    def resume_patrol(self) -> None:
        logger.info("继续巡检 from idx=%d", self.current_waypoint_idx)
        self.patrol_btn.configure(text="暂停")
        self.schedule_async(self._async_resume_then_patrol(self.current_waypoint_idx))

    async def _async_pause_navigation(self) -> None:
        resp = await self.client.call("pause_navigation")
        if resp.get("success"):
            self.is_paused = True
            self.root.after(0, lambda: self.patrol_status_var.set("状态: 已暂停"))
        else:
            logger.error("暂停导航失败: %s", resp.get("msg"))
            self.root.after(0, lambda: self.patrol_status_var.set(f"状态: 暂停失败 - {resp.get('msg')}"))

    async def _async_resume_then_patrol(self, start_idx: int) -> None:
        resp = await self.client.call("resume_navigation")
        if not resp.get("success"):
            logger.warning("resume_navigation 返回: %s", resp.get("msg"))
        self.is_paused = False
        self.is_patrolling = True
        await self._async_patrol(start_idx=start_idx)

    def stop_patrol(self) -> None:
        self.is_patrolling = False
        self.is_paused = False
        self.current_waypoint_idx = 0
        self.patrol_btn.configure(text="开始巡检")
        self.stop_patrol_btn.configure(state=tk.DISABLED)
        self.schedule_async(self._async_cancel_navigation())
        self.root.after(0, lambda: self.patrol_status_var.set("状态: 已停止"))

    async def _async_cancel_navigation(self) -> None:
        resp = await self.client.call("cancel_navigation")
        if not resp.get("success"):
            logger.warning("cancel_navigation 失败: %s", resp.get("msg"))

    def on_cancel_navigation(self) -> None:
        self.schedule_async(self._async_cancel_navigation())
        self.root.after(0, lambda: self.nav_status_var.set("导航状态: 已取消"))

    async def _async_patrol(self, start_idx: int = 0) -> None:
        """巡检主循环。可从 start_idx 续航。"""
        try:
            route_name = self.route_var.get()
            self.root.after(
                0, lambda r=route_name: self.patrol_status_var.set(f"状态: 加载路线 {r}...")
            )

            # 仅在 start_idx==0 时重新拉取路线（首次开始）；续航时复用上一次的 waypoints
            if start_idx == 0:
                resp = await self.client.call("get_route", {"route_name": route_name})
                if not resp.get("success"):
                    self.root.after(
                        0,
                        lambda m=resp.get("msg"): messagebox.showerror(
                            "错误", f"获取路线失败: {m}"
                        ),
                    )
                    self.root.after(0, self.stop_patrol)
                    return
                self.route_waypoints = list(resp.get("waypoints") or [])
                if not self.route_waypoints:
                    self.root.after(0, lambda: messagebox.showerror("错误", "路线无可用 waypoint"))
                    self.root.after(0, self.stop_patrol)
                    return
                self.root.after(
                    0,
                    lambda txt=self._format_waypoints(self.route_waypoints):
                    self._update_waypoint_text(txt),
                )

            total = len(self.route_waypoints)
            logger.info("巡检路线 %s, 共 %d 个点位, 从第 %d 个开始", route_name, total, start_idx + 1)

            success_count = 0
            total_arrive_time = 0.0
            start_total = time.time()

            self.current_waypoint_idx = start_idx

            while self.current_waypoint_idx < total and self.is_patrolling:
                waypoint = self.route_waypoints[self.current_waypoint_idx]
                target_name = self._wp_target_name(waypoint)
                if not target_name:
                    logger.error("路线点格式不正确: %s", waypoint)
                    self.current_waypoint_idx += 1
                    continue

                idx = self.current_waypoint_idx + 1
                self.root.after(
                    0,
                    lambda w=self._wp_name(waypoint), i=idx, t=total: self.patrol_status_var.set(
                        f"状态: 前往 {w} ({i}/{t})"
                    ),
                )
                logger.info("巡检点 %d/%d: %s", idx, total, self._wp_name(waypoint))

                start_time = time.time()
                goal_resp = await self.client.call(
                    "set_goal_by_name", {"target": target_name}
                )
                if not goal_resp.get("success"):
                    logger.error("set_goal_by_name 失败: %s", goal_resp.get("msg"))
                    self.current_waypoint_idx += 1
                    continue

                arrived = await self._wait_until_arrived(target_name)
                if arrived:
                    duration = time.time() - start_time
                    success_count += 1
                    total_arrive_time += duration
                    logger.info("✓ 到达 %s, 耗时 %.1fs", self._wp_name(waypoint), duration)
                    self.current_waypoint_idx += 1
                    if self.is_patrolling and self.current_waypoint_idx < total:
                        await asyncio.sleep(2.0)
                else:
                    # 被暂停或终止：停在当前 idx，由 resume/stop 决定下一步
                    logger.info("⏸/✋ 等待中断于点 %s (idx=%d)", target_name, self.current_waypoint_idx)
                    return

            # 巡检完成
            total_duration = time.time() - start_total
            self.root.after(0, self.stop_patrol)

            avg = total_arrive_time / max(success_count, 1)
            report = (
                f"\n{'=' * 50}\n巡检报告\n{'=' * 50}\n"
                f"路线: {route_name}\n"
                f"总点位: {total}\n"
                f"成功: {success_count}\n"
                f"失败: {total - success_count}\n"
                f"成功率: {success_count / max(total, 1) * 100:.1f}%\n"
                f"总耗时: {total_duration:.1f}秒\n"
                f"平均每点: {avg:.1f}秒\n"
                f"{'=' * 50}"
            )
            logger.info(report)
            print(report)

        except Exception as e:
            logger.exception("巡检执行失败: %s", e)
            self.root.after(0, self.stop_patrol)

    @staticmethod
    def _wp_target_name(wp: Any) -> Optional[str]:
        if isinstance(wp, dict):
            return wp.get("semantic_point") or wp.get("point") or wp.get("name")
        if isinstance(wp, str):
            return wp
        return None

    async def _wait_until_arrived(self, target_name: str) -> bool:
        """轮询 wait_goal_reached 直到成功 / 巡检被暂停或停止。"""
        while self.is_patrolling:
            await asyncio.sleep(1.0)
            if not self.is_patrolling:
                break
            wait_resp = await self.client.call(
                "wait_goal_reached",
                {
                    "target": target_name,
                    "timeout": 1.0,
                    "xy_tolerance": 0.6,
                    "yaw_tolerance": 3.14,
                },
                timeout=10.0,
            )
            if wait_resp.get("success"):
                return True
            if wait_resp.get("nav_failed"):
                logger.error("导航失败: %s", wait_resp.get("msg"))
                return False
        return False

    def _update_waypoint_text(self, text: str) -> None:
        self.waypoint_text.configure(state=tk.NORMAL)
        self.waypoint_text.delete(1.0, tk.END)
        self.waypoint_text.insert(tk.END, text)
        self.waypoint_text.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # 单点导航
    # ------------------------------------------------------------------
    def on_set_goal(self) -> None:
        target = self.goal_var.get()
        if not target:
            messagebox.showwarning("警告", "请选择目标点")
            return
        self.schedule_async(self._async_set_goal(target))

    async def _async_set_goal(self, target: str) -> None:
        self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 前往 {target}..."))
        resp = await self.client.call("set_goal_by_name", {"target": target})
        if resp.get("success"):
            logger.info("set_goal_by_name 成功: %s", target)
            self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 正在前往 {target}"))
        else:
            logger.error("set_goal_by_name 失败: %s", resp.get("msg"))
            self.root.after(
                0, lambda m=resp.get("msg"): self.nav_status_var.set(f"导航状态: 失败 - {m}")
            )

    def on_set_goal_xy(self) -> None:
        if self.xy_select_mode:
            self.xy_select_mode = False
            self.xy_status_var.set("点击按钮后在地图选点")
            self.set_goal_xy_btn.configure(text="set_goal_by_xy")
            return
        self.xy_select_mode = True
        self.xy_status_var.set("请在地图上点击选择目标点...")
        self.set_goal_xy_btn.configure(text="取消选点")

    def on_map_click(self, event) -> None:
        if not self.xy_select_mode:
            return
        if event.xdata is None or event.ydata is None:
            return

        x, y = float(event.xdata), float(event.ydata)
        logger.info("选择目标点: (%.2f, %.2f)", x, y)

        self.xy_select_mode = False
        self.xy_status_var.set("点击按钮后在地图选点")
        self.set_goal_xy_btn.configure(text="set_goal_by_xy")

        if self.target_marker is not None:
            try:
                self.target_marker.remove()
            except Exception:
                pass
        (self.target_marker,) = self.ax.plot(x, y, "g*", markersize=15, label="Target")
        self.canvas.draw()

        self.schedule_async(self._async_set_goal_by_xy(x, y))

    async def _async_set_goal_by_xy(self, x: float, y: float) -> None:
        self.root.after(
            0, lambda: self.nav_status_var.set(f"导航状态: 前往 ({x:.2f}, {y:.2f})...")
        )
        resp = await self.client.call(
            "set_goal_by_xy", {"x": x, "y": y, "distance": 0.5}
        )
        if resp.get("success"):
            logger.info("set_goal_by_xy 成功: (%.2f, %.2f)", x, y)
            self.root.after(
                0, lambda: self.nav_status_var.set(f"导航状态: 正在前往 ({x:.2f}, {y:.2f})")
            )
        else:
            logger.error("set_goal_by_xy 失败: %s", resp.get("msg"))
            self.root.after(
                0, lambda m=resp.get("msg"): self.nav_status_var.set(f"导航状态: 失败 - {m}")
            )

    # ------------------------------------------------------------------
    # 关闭
    # ------------------------------------------------------------------
    def on_close(self) -> None:
        try:
            if self.client is not None:
                fut = asyncio.run_coroutine_threadsafe(self.client.close(), self.loop)
                try:
                    fut.result(timeout=2.0)
                except Exception:
                    pass
        finally:
            try:
                if self.loop and self.loop.is_running():
                    self.loop.call_soon_threadsafe(self.loop.stop)
            except Exception:
                pass
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="WebSocket 巡检/导航测试 UI 客户端")
    parser.add_argument(
        "--ws",
        default="ws://localhost:9999/ws",
        help="WebSocket URL（默认 ws://localhost:9999/ws）",
    )
    # 兼容旧参数 --url
    parser.add_argument("--url", default=None, help="兼容旧参数：等价于 --ws")
    args = parser.parse_args()

    ws_url = args.url if args.url else args.ws
    app = NavClientUI(ws_url=ws_url)
    app.run()


if __name__ == "__main__":
    main()
