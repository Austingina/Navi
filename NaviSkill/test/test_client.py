#!/usr/bin/env python3
"""
WebSocket 客户端测试脚本 — 对接 server_fast.py 的 /ws 接口，测试巡检点位 / 路线。
"""

import argparse
import asyncio
import json
import logging
import os
import select
import sys
import termios
import time
import tty
from typing import Any, Dict, List, Optional

import websockets

# 设置日志
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


class WsNavClient:
    """与 Navigation WebSocket API（server_fast.py）通信的简单客户端。"""

    def __init__(self, url: str):
        self.url = url.rstrip("/")
        if self.url.startswith("http://"):
            self.url = "ws://" + self.url[7:]
        elif self.url.startswith("https://"):
            self.url = "wss://" + self.url[8:]
        self._ws: Any = None  # websockets ClientConnection (版本间类型不同，用 Any)
        self._req_id = 0

    async def connect(self) -> None:
        self._ws = await websockets.connect(self.url, ping_interval=20, ping_timeout=60)

    async def close(self) -> None:
        if self._ws is not None:
            await self._ws.close()
            self._ws = None

    async def __aenter__(self) -> "WsNavClient":
        await self.connect()
        return self

    async def __aexit__(self, *args: Any) -> None:
        await self.close()

    async def call(self, method: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if self._ws is None:
            raise RuntimeError("WebSocket 未连接，请先 connect() 或使用 async with")

        self._req_id += 1
        rid = self._req_id
        payload = {"id": rid, "method": method, "params": params or {}}
        await self._ws.send(json.dumps(payload, ensure_ascii=False))

        raw = await self._ws.recv()
        data = json.loads(raw)

        if data.get("error"):
            return {"success": False, "msg": str(data["error"])}
        if "result" in data:
            r = data["result"]
            if isinstance(r, dict):
                return r
            return {"success": True, "data": r}
        return {"success": False, "msg": f"未知响应: {data}"}


class KeyboardHandler:
    """非阻塞键盘输入处理器"""

    def __init__(self) -> None:
        self.old_settings = None

    def __enter__(self) -> "KeyboardHandler":
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass
        return self

    def __exit__(self, *args: Any) -> None:
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass

    def check_key(self) -> str:
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except Exception:
            pass
        return ""


class SimplePatrolClient:
    def __init__(
        self,
        ws_url: str = "ws://localhost:9000/ws",
        route_name: Optional[str] = None,
        patrol_sequence: Optional[List[str]] = None,
        continue_tolerance: float = -1.0,
        auto_confirm: bool = False,
        route_start_index: int = 0,
    ):
        self.ws_url = ws_url
        self.pose_list: list = []
        self.auto_confirm = auto_confirm
        self.is_paused = False
        self.continue_tolerance = continue_tolerance
        self.route_start_index = route_start_index

        self.patrol_sequence: list = list(patrol_sequence) if patrol_sequence else []

        _r = route_name.strip() if route_name else ""
        self.route_cli: Optional[str] = _r if _r else None
        self.route_name: str = ""

    async def print_status(self, client) -> Optional[dict]:
        """打印当前机器人状态（导航模式 / 地图 / 位置）。

        - 打印当前 nav_mode：
          * SLAM 模式：打印 slam_map id、当前 submap id、平面位置及子地图列表；
          * 室外 GPS 模式：打印平面位置与最新 GNSS fix；
        - 打印最接近的预设点。

        :return: 当前平面位置 dict（``{'x','y','z'}``）；暂无实时位姿时返回 None。
        """
        # 当前导航状态
        logger.info("🔍 检查服务器连接和机器人状态...")
        status_data = await client.call("get_status")
        nav_mode = status_data.get("nav_mode")
        if nav_mode:
            logger.info(f"🧭 当前导航模式: {nav_mode}")

        if not status_data.get("success"):
            # 无实时位姿并非致命：新架构下定位/导航栈可能按需（首个目标触发
            # _ensure_nav_mode）才拉起，此时尚无 map→base_link。继续下发目标即可。
            logger.warning(
                "⚠️  暂无机器人实时位姿（%s）；定位/导航栈可能尚未就绪，"
                "仍继续下发目标（服务端会按需切换/拉起对应导航栈）",
                status_data.get("msg"),
            )
            return None

        current_pos = status_data.get("position")
        if nav_mode == "slam":
            slam_map = status_data.get("slam_map")
            cur_map = await client.call("get_current_map_id")
            submap_id = cur_map.get("map_id") if cur_map.get("success") else None
            logger.info(f"🗺️  SLAM 定位：slam_map={slam_map}, submap={submap_id}")
            if current_pos:
                logger.info(
                    f"📍 当前位置: x={current_pos['x']:.3f}, y={current_pos['y']:.3f}"
                )

            # 子地图列表（仅 SLAM 模式下有意义）
            submap_list_data = await client.call("get_submap_list")
            if submap_list_data.get("success"):
                submap_ids = submap_list_data.get("submaps", [])
                logger.info(f"🗺️  子地图列表 ({len(submap_ids)} 个): {submap_ids}")
                for sid in submap_ids:
                    info_data = await client.call("get_submap_info", {"submap_id": sid})
                    if info_data.get("success"):
                        info = info_data.get("info", {})
                        sp_count = len(info.get("semantic_points", {}))
                        portal_count = len(info.get("submap_portals", []))
                        logger.info(
                            f"   [{sid}] name={info.get('name', '')}, "
                            f"desc={info.get('description', '')}, "
                            f"map_file={info.get('map_file', '')}, "
                            f"points={sp_count}, portals={portal_count}"
                        )
        else:
            # 室外 GPS 模式
            if current_pos:
                logger.info(
                    f"📍 当前位置(map): x={current_pos['x']:.3f}, y={current_pos['y']:.3f}"
                )
            gps_fix = status_data.get("gps_fix")
            if gps_fix:
                logger.info(
                    f"🛰️  GNSS fix: lon={gps_fix.get('lon')}, "
                    f"lat={gps_fix.get('lat')}, status={gps_fix.get('status')}"
                )

        nearest_data = await client.call("get_nearest_point")
        if nearest_data.get("success"):
            logger.info(
                f"📍 最接近的预设点: {nearest_data['nearest_point']} "
                f"(距离: {nearest_data['distance']}m)"
            )

        return current_pos

    async def print_routes(self, client) -> bool:
        """确定并打印本次巡检路线，结果写入 ``self.patrol_sequence`` / ``self.route_name``。

        优先级：``-p`` 指定序列 > ``-r`` 指定路线 > default_route > 第一条路线；
        再按 ``route_start_index`` 截断。返回 False 表示无可用路线（应终止）。
        """
        if self.patrol_sequence:
            logger.info(
                "📌 使用 -p 指定的巡检序列 (%d 个目的地): %s",
                len(self.patrol_sequence),
                self.patrol_sequence,
            )
        else:
            routes_data = await client.call("get_route_names")
            if not routes_data.get("success"):
                logger.error(
                    "❌ get_route_names 失败: %s",
                    routes_data.get("msg"),
                )
                return False

            names: List[str] = list(routes_data.get("route_names") or [])
            cnt = routes_data.get("count", len(names))
            logger.info(f"📋 服务器全部路线 ({cnt} 条): {names}")
            if not names:
                logger.error("❌ 服务器未配置任何路线")
                return False

            if self.route_cli:
                if self.route_cli not in names:
                    logger.error(
                        "❌ 路线 %r 不存在，可用: %s",
                        self.route_cli,
                        names,
                    )
                    return False
                self.route_name = self.route_cli
                logger.info("📌 使用 -r 指定路线: %r", self.route_name)
            else:
                default_route_data = await client.call("get_default_route")
                dr = default_route_data.get("default_route") if default_route_data.get("success") else None
                if dr and dr in names:
                    self.route_name = dr
                    logger.info("📌 使用 default_route: %r", self.route_name)
                else:
                    self.route_name = names[0]
                    logger.info("📌 未指定 -r 且无 default_route，使用第一条路线: %r", self.route_name)

            logger.info(f"📌 拉取路线详情: {self.route_name!r}")
            try:
                route_data = await client.call(
                    "get_route", {"route_name": self.route_name}
                )
                if route_data.get("success") and route_data.get("waypoints"):
                    self.patrol_sequence = route_data["waypoints"]
                    logger.info(
                        f"✅ 路线 {self.route_name} 共 {len(self.patrol_sequence)} 个途经点"
                    )
                else:
                    logger.warning(
                        "无法获取路线 %r: %s",
                        self.route_name,
                        route_data.get("msg"),
                    )
            except Exception as e:
                logger.warning(f"获取路线异常: {e}")

            if not self.patrol_sequence:
                logger.error(
                    "❌ 未得到任何路线途经点（请检查地图 multifloor 是否配置 routes）"
                )
                return False

        if self.route_start_index > 0:
            total = len(self.patrol_sequence)
            if self.route_start_index >= total:
                logger.error(
                    f"❌ 起始序号 {self.route_start_index + 1} 超出路线点位总数 {total}"
                )
                return False
            self.patrol_sequence = self.patrol_sequence[self.route_start_index:]
            logger.info(
                f"📌 从第 {self.route_start_index + 1}/{total} 个点位开始，"
                f"剩余 {len(self.patrol_sequence)} 个点"
            )

        logger.info(
            f"📋 本次测试点位序列 ({len(self.patrol_sequence)}): "
            f"{self.patrol_sequence}"
        )
        return True

    async def start_patrol_test(self) -> bool:
        """开始巡检测试"""
        logger.info("🚀 开始巡检路径测试 (WebSocket)")
        logger.info(f"📡 {self.ws_url}")

        async with WsNavClient(self.ws_url) as client:
            current_pos = await self.print_status(client)

            if not await self.print_routes(client):
                return False

            if self.continue_tolerance >= 0 and current_pos and len(self.patrol_sequence) > 1:
                tol = self.continue_tolerance
                cur_x = current_pos['x']
                cur_y = current_pos['y']
                best_idx = 0
                best_dist = float('inf')
                for idx, wp in enumerate(self.patrol_sequence):
                    wp_name = wp.get('name') if isinstance(wp, dict) else wp
                    if not wp_name:
                        continue
                    pi = await client.call("get_pose_in_submap", {"pose_name": wp_name})
                    if not pi.get('success'):
                        continue
                    pose = pi.get('point_info', {}).get('pose')
                    if not pose:
                        continue
                    dx = cur_x - pose['position']['x']
                    dy = cur_y - pose['position']['y']
                    d = (dx * dx + dy * dy) ** 0.5
                    if d <= tol:
                        best_idx = idx
                        best_dist = d
                    elif d < best_dist:
                        best_dist = d
                        best_idx = idx
                self.patrol_sequence = self.patrol_sequence[best_idx:]
                logger.info(
                    f"📌 --continue 模式 (tolerance={tol:.1f}m)：从第 {best_idx + 1} 个途经点开始"
                    f"（距离 {best_dist:.2f}m），剩余 {len(self.patrol_sequence)} 个点"
                )

            success_count = 0
            total_time = 0.0

            for i, waypoint in enumerate(self.patrol_sequence):
                logger.info(f"\n{'=' * 50}")
                logger.info(f"📍 测试点位 {i + 1}/{len(self.patrol_sequence)}: {waypoint}")

                is_route_dict = isinstance(waypoint, dict)

                # waypoint 类型完全以统一的 type 字段为准（服务端已标注 slam/gps/instruction）：
                #   slam = 某 SLAM 地图 submap 的语义点；gps = 场景/经纬度点；
                #   instruction = VLN 指令（本简单客户端不支持）。
                # -p 传入的裸字符串点名按 slam 处理。
                wp_type = waypoint.get("type", "slam") if is_route_dict else "slam"

                start_time = time.time()

                # ── VLN 指令：SimplePatrolClient 不支持，跳过 ──
                if wp_type == "instruction":
                    logger.warning(
                        "⏭️  跳过 instruction(VLN) 途经点（简单巡检客户端不支持）: %s",
                        waypoint,
                    )
                    continue

                # ── 室外 GPS（场景语义点 / 裸经纬度）：走 set_goal_by_outdoor ──
                if wp_type == "gps":
                    gname = waypoint.get("name")
                    if gname:
                        outdoor_params = {"target": gname}
                    else:
                        gname = "tmp"
                        outdoor_params = {
                            "target": "tmp",
                            "lon": waypoint.get("lon"),
                            "lat": waypoint.get("lat"),
                        }
                    logger.info("🌍 室外目标(%s): %s -> set_goal_by_outdoor", wp_type, gname)
                    goal_data = await client.call("set_goal_by_outdoor", outdoor_params)
                    if not goal_data.get("success"):
                        logger.error("❌ 下发室外目标失败: %s", goal_data.get("msg"))
                        continue
                    logger.info("✅ 室外目标下发成功: %s", goal_data.get("msg"))
                    logger.info("⏳ 等待到达室外目标 (超时: 600秒)... [按空格键暂停/恢复]")
                    wait_data = await self.wait_goal_with_pause(
                        client, outdoor_params["target"], timeout=600.0
                    )
                    duration = time.time() - start_time
                    if wait_data.get("success"):
                        logger.info("🎉 成功到达 %s (耗时: %.2f秒)", gname, duration)
                        success_count += 1
                        total_time += duration
                    else:
                        logger.error("❌ 未能到达 %s: %s", gname, wait_data.get("msg"))
                    if i < len(self.patrol_sequence) - 1:
                        await asyncio.sleep(2)
                    continue

                # ── SLAM（map 类型 / 裸字符串点名）：解析目标点与所在 submap ──
                target_name = None
                target_submap = None
                if is_route_dict:
                    target_submap = waypoint.get("submap")
                    target_name = waypoint.get("name")
                    if not target_name:
                        logger.error(f"❌ 路线点格式不正确: {waypoint}")
                        continue
                else:
                    target_name = waypoint

                if not is_route_dict and target_name not in self.pose_list:
                    logger.error(f"❌ 点位 {target_name} 不存在于服务器点位列表中")
                    continue

                pre_status_data = await client.call("get_status")
                if pre_status_data.get("success"):
                    pre_pos = pre_status_data["position"]
                    logger.info(f"📍 下发前位置: x={pre_pos['x']:.3f}, y={pre_pos['y']:.3f}")

                # 跨子图切换完全由服务端处理：只要下发 set_goal_by_slam 时带上目标
                # submap，服务端内部（_ensure_submap）会先导航到当前→目标 submap 的
                # 传送门、到达后换图，客户端无需自行判断/取传送门/换图。

                point_meta = await client.call("get_pose_in_submap", {"pose_name": target_name})
                pt_type = point_meta.get("point_info", {}).get("type") if point_meta.get("success") else None
                logger.info(
                    f"📋 点位详细信息 [{target_name}]: "
                    f"{json.dumps(point_meta.get('point_info'), ensure_ascii=False, indent=2)}"
                )
                if pt_type in {"elevator_wait", "elevator_out", "elevator_in"}:
                    if pt_type == "elevator_wait":
                        prompt_msg = "确认呼叫了电梯，等待电梯到达？(Y/n): "
                        logger.info(
                            f"🛗 目标 {target_name} 是 elevator_wait 类型，"
                            "按回车确认继续，输入 n 跳过："
                        )
                    elif pt_type == "elevator_out":
                        prompt_msg = "确认出电梯？(Y/n): "
                        logger.info(
                            f"🛗 目标 {target_name} 是 elevator_out 类型，"
                            "请确认电梯已到达目标楼层并开门后按回车继续："
                        )
                    else:
                        prompt_msg = "确认进电梯？(Y/n): "
                        logger.info(
                            f"🛗 目标 {target_name} 是 elevator_in 类型，"
                            "请确认电梯已到达且门已打开、可安全进梯后按回车继续："
                        )

                    if self.auto_confirm:
                        logger.info("🤖 -Y 自动确认，继续执行")
                    else:
                        confirm = await asyncio.get_event_loop().run_in_executor(
                            None, lambda: input(prompt_msg).strip().lower()
                        )
                        if confirm == "n":
                            logger.info(f"⏭️  用户取消，跳过 {target_name}")
                            continue

                pinfo = point_meta.get("point_info") or {}
                pose_ = pinfo.get("pose") or {}
                if not pose_ and pinfo.get("poses"):
                    pose_ = (pinfo["poses"] or [None])[0] or {}
                pos = pose_.get("position") or {}
                if pos:
                    logger.info(
                        f"🎯 下发目标点: {target_name}, type: {pt_type}, "
                        f"pos: x={pos.get('x', 0.0):.3f} y={pos.get('y', 0.0):.3f} z={pos.get('z', 0.0):.3f}"
                    )
                else:
                    logger.info(
                        f"🎯 下发目标点: {target_name}, type: {pt_type} (无 pose 坐标)"
                    )
                goal_params = {"target": target_name}
                if is_route_dict and target_submap:
                    # 携带目标 submap：若与当前不一致，服务端会在下发前自动换图。
                    goal_params["submap"] = target_submap
                goal_data = await client.call("set_goal_by_slam", goal_params)

                if not goal_data.get("success"):
                    logger.error(f"❌ 下发目标失败: {goal_data.get('msg')}")
                    continue

                logger.info(f"✅ 目标点下发成功: {goal_data.get('msg')}")

                logger.info("⏳ 等待导航系统响应...")
                await asyncio.sleep(2)

                post_status_data = await client.call("get_status")
                if post_status_data.get("success"):
                    post_pos = post_status_data["position"]
                    logger.info(f"📍 响应后位置: x={post_pos['x']:.3f}, y={post_pos['y']:.3f}")

                logger.info(
                    "⏳ 等待到达目标点 (超时: 120秒)... [按空格键暂停/恢复]"
                )
                wait_data = await self.wait_goal_with_pause(
                    client, target_name, timeout=120.0, xy_tolerance=0.04, yaw_tolerance=0.03
                )

                duration = time.time() - start_time

                if wait_data.get("success"):
                    logger.info(f"🎉 成功到达 {waypoint} (耗时: {duration:.2f}秒)")
                    success_count += 1
                    total_time += duration

                    current_status_data = await client.call("get_status")
                    if current_status_data.get("success"):
                        pos = current_status_data["position"]
                        logger.info(f"📍 到达后位置: x={pos['x']:.3f}, y={pos['y']:.3f}")

                    if i < len(self.patrol_sequence) - 1:
                        logger.info("⏸️  暂停2秒后发送下一个目标...")
                        await asyncio.sleep(2)
                else:
                    logger.error(f"❌ 未能到达 {waypoint}: {wait_data.get('msg')}")

                    current_status_data = await client.call("get_status")
                    if current_status_data.get("success"):
                        pos = current_status_data["position"]
                        logger.info(f"📍 失败时位置: x={pos['x']:.3f}, y={pos['y']:.3f}")

                    logger.error(f"🛑 由于未能到达 {waypoint}，测试中止")
                    logger.error("💡 请检查机器人导航系统和路径规划")
                    break

            self.generate_report(success_count, len(self.patrol_sequence), total_time)
            return success_count == len(self.patrol_sequence)

    async def wait_goal_with_pause(
        self,
        client: WsNavClient,
        waypoint: str,
        timeout: float = 120.0,
        xy_tolerance: float = 0.6,
        yaw_tolerance: float = 3.14,
    ) -> Dict[str, Any]:
        """等待到达目标点，支持空格键暂停/恢复。单轮超时后自动再等待一轮（共最多 2 轮）。"""
        last_result: Dict[str, Any] = {"success": False, "msg": "等待超时或未到达"}

        for attempt in range(2):
            # 修复2：第2轮开始前，若上一轮在暂停状态下超时（导航已被 cancel），
            # 必须先调 resume_navigation 重新下发 goal；否则 _nav_terminal_message
            # 仍为"导航已取消"，wait_goal_reached 会立即返回 nav_failed。
            if attempt > 0 and self.is_paused:
                logger.info("▶️  重试轮次：上轮在暂停中超时，重新恢复导航...")
                resume_data = await client.call("resume_navigation")
                if not resume_data.get("success"):
                    logger.warning(f"重试恢复失败: {resume_data.get('msg')}")

            start_time = time.time()
            paused_duration = 0.0
            pause_start: Optional[float] = None
            self.is_paused = False

            with KeyboardHandler() as kb:
                while True:
                    # 修复1：elapsed 需同时减去"已完成暂停时长"和"当前正在进行中的暂停时长"，
                    # 否则暂停期间超时计时仍在流逝，会比预期提前触发超时。
                    ongoing = (time.time() - pause_start) if (self.is_paused and pause_start is not None) else 0.0
                    elapsed = time.time() - start_time - paused_duration - ongoing
                    if elapsed >= timeout:
                        break

                    key = kb.check_key()
                    if key == " ":
                        if not self.is_paused:
                            self.is_paused = True
                            pause_start = time.time()
                            logger.info("⏸️  [空格键] 导航已暂停，再次按空格键继续...")
                            pause_data = await client.call("pause_navigation")
                            if not pause_data.get("success"):
                                logger.warning(f"暂停失败: {pause_data.get('msg')}")
                        else:
                            self.is_paused = False
                            if pause_start is not None:
                                paused_duration += time.time() - pause_start
                                pause_start = None
                            logger.info("▶️  [空格键] 导航已恢复...")
                            resume_data = await client.call("resume_navigation")
                            if not resume_data.get("success"):
                                logger.warning(f"恢复失败: {resume_data.get('msg')}")

                    if self.is_paused:
                        await asyncio.sleep(0.2)
                        continue

                    try:
                        wait_data = await client.call(
                            "wait_goal_reached",
                            {
                                "target": waypoint,
                                "timeout": 0.5,
                                "xy_tolerance": xy_tolerance,
                                "yaw_tolerance": yaw_tolerance,
                            },
                        )
                        if wait_data.get("success"):
                            return wait_data
                        if wait_data.get("nav_failed"):
                            return wait_data
                    except Exception as e:
                        logger.debug(f"等待检查异常: {e}")

                    await asyncio.sleep(0.1)

            last_result = {"success": False, "msg": "等待超时或未到达"}
            if attempt == 0:
                logger.warning(
                    "⏳ 目标 %s 本轮等待超时，重试一轮 (第 2/2 轮，每轮 %.0fs)...",
                    waypoint,
                    timeout,
                )

        return last_result

    def generate_report(self, success_count: int, total_count: int, total_time: float) -> None:
        logger.info(f"\n{'=' * 60}")
        logger.info("📊 巡检测试报告")
        logger.info(f"{'=' * 60}")

        if total_count == 0:
            logger.info("无巡检点位，跳过统计。")
            return

        success_rate = (success_count / total_count) * 100

        logger.info("📈 测试结果:")
        logger.info(f"   • 总点位数量: {total_count}")
        logger.info(f"   • 成功点位: {success_count}")
        logger.info(f"   • 失败点位: {total_count - success_count}")
        logger.info(f"   • 成功率: {success_rate:.1f}%")
        logger.info(f"   • 总耗时: {total_time:.2f}秒")
        if success_count > 0:
            logger.info(f"   • 平均每点耗时: {total_time / success_count:.2f}秒")


async def main() -> int:
    parser = argparse.ArgumentParser(description="WebSocket 巡检测试客户端 (server_fast.py)")
    parser.add_argument(
        "--ws",
        default=os.environ.get("NAV_WS_URL", "ws://localhost:9000/ws"),
        help="WebSocket URL，默认 ws://localhost:9000/ws（与 server -t 端口一致）",
    )
    parser.add_argument(
        "-r",
        "--route",
        nargs="+",
        default=None,
        metavar=("NAME", "START_INDEX"),
        help="路线名称 [起始点位序号]；省略序号则从第 1 个开始；序号从 1 起",
    )
    parser.add_argument(
        "-p",
        "--patrol",
        nargs="*",
        default=None,
        metavar="DEST",
        help="启动巡检模式；可选指定目的地序列（不指定则使用 -r 路线或默认路线）",
    )
    parser.add_argument(
        "-c",
        "--continue",
        dest="continue_tolerance",
        type=float,
        default=-1.0,
        metavar="TOL",
        help="从路线中距当前位置最近的途经点继续巡检；TOL 为容差（米），"
             "容差范围内有多个点时优先选靠后的（尽快走完路线）",
    )
    parser.add_argument(
        "-Y",
        "--yes",
        action="store_true",
        default=False,
        help="自动确认所有电梯相关提示（elevator_wait/elevator_in/elevator_out），不再等待用户输入",
    )
    parser.add_argument(
        "--set-xy",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="测试 set_goal_by_slam(target=tmp): 提供 X Y 坐标",
    )
    args = parser.parse_args()

    try:
        # If --set-xy provided, run a minimal test for set_goal_by_slam(tmp) + get_navigation_plan
        if args.set_xy:
            x, y = args.set_xy
            logger.info(f"🔬 Running set_goal_by_slam(target=tmp) test to x={x}, y={y} on {args.ws}")
            async with WsNavClient(args.ws) as client:
                res = await client.call("set_goal_by_slam", {"target": "tmp", "x": float(x), "y": float(y), "distance": 0.05})
                logger.info(f"set_goal_by_slam response: {res}")
                if not res.get("success", True):
                    return 1

                logger.info("获取 /plan 全局路径（get_navigation_plan, timeout=30s）...")
                plan_res = await client.call("get_navigation_plan", {"timeout": 30.0})
                logger.info(f"get_navigation_plan response: {plan_res}")
                if plan_res.get("success"):
                    plan = plan_res.get("plan") or {}
                    logger.info(
                        "   frame_id=%s pose_count=%s",
                        plan.get("frame_id"),
                        plan.get("pose_count"),
                    )
                return 0 if plan_res.get("success") else 1

        # -p/--patrol 存在时才执行巡检（args.patrol 为 None 表示未传 -p）
        if args.patrol is not None:
            patrol_seq = args.patrol if args.patrol else None
            route_name = None
            route_start_index = 0
            if args.route:
                route_name = args.route[0]
                if len(args.route) >= 2:
                    try:
                        route_start_index = int(args.route[1]) - 1
                        if route_start_index < 0:
                            route_start_index = 0
                    except ValueError:
                        logger.error("❌ -r 的第二个参数（起始序号）必须为正整数")
                        return 1
            client = SimplePatrolClient(
                ws_url=args.ws,
                route_name=route_name,
                patrol_sequence=patrol_seq,
                continue_tolerance=args.continue_tolerance,
                auto_confirm=args.yes,
                route_start_index=route_start_index,
            )
            success = await client.start_patrol_test()

            if success:
                logger.info("\n🎉 所有巡检点位测试通过！")
                return 0
            logger.error("\n⚠️  部分巡检点位测试失败")
            return 1

        logger.info("未指定操作（-p 巡检 / --set-xy 坐标导航），退出。")
        return 0

    except KeyboardInterrupt:
        logger.info("\n⏹️  用户中断测试")
        return 2
    except Exception as e:
        logger.exception("\n💥 测试运行异常: %s", e)
        return 3


if __name__ == "__main__":
    exit(asyncio.run(main()))
