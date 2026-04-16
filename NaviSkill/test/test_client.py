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


class SubmapClient:
    """封装子地图（submap）查询与切换操作。"""

    def __init__(self, ws_client: WsNavClient):
        self._client = ws_client

    async def get_current_submap(self) -> Optional[str]:
        """获取当前加载的子地图 ID，失败返回 None。"""
        data = await self._client.call("get_current_map_id")
        if data.get("success"):
            return data.get("map_id")
        logger.warning("获取当前子地图失败: %s", data.get("msg"))
        return None

    async def get_submap_list(self) -> List[str]:
        """获取服务器已配置的所有子地图 ID 列表。"""
        data = await self._client.call("get_submap_list")
        if data.get("success"):
            return list(data.get("submaps") or [])
        logger.warning("获取子地图列表失败: %s", data.get("msg"))
        return []

    async def ensure_submap(self, target_submap: str, timeout: float = 60.0) -> bool:
        """
        确保当前子地图为 *target_submap*。

        如果已在目标子地图则立即返回 True；
        否则调用 change_map 切换，成功返回 True，失败返回 False。
        """
        current = await self.get_current_submap()
        if current == target_submap:
            logger.info("🗺️  当前已在目标子地图: %s", target_submap)
            return True

        available = await self.get_submap_list()
        if target_submap not in available:
            logger.error(
                "❌ 目标子地图 %r 不存在，可用: %s", target_submap, available
            )
            return False

        logger.info("🔁 切换子地图: %s → %s", current, target_submap)
        data = await self._client.call(
            "change_map", {"map_id": target_submap, "timeout": timeout}
        )
        if data.get("success"):
            logger.info("✅ 子地图切换成功: %s", target_submap)
            init_pose = data.get("init_pose")
            if init_pose:
                logger.info("📍 切换后初始位姿: %s", init_pose)
            return True

        logger.error("❌ 子地图切换失败: %s", data.get("msg"))
        return False


class SimplePatrolClient:
    def __init__(
        self,
        ws_url: str = "ws://localhost:9999/ws",
        route_name: Optional[str] = None,
        patrol_sequence: Optional[List[str]] = None,
        continue_tolerance: float = -1.0,
        auto_confirm: bool = False,
    ):
        self.ws_url = ws_url
        self.pose_list: list = []
        self.auto_confirm = auto_confirm
        self.is_paused = False
        self.continue_tolerance = continue_tolerance

        self.patrol_sequence: list = list(patrol_sequence) if patrol_sequence else []

        _r = route_name.strip() if route_name else ""
        self.route_cli: Optional[str] = _r if _r else None
        self.route_name: str = ""

    async def start_patrol_test(self) -> bool:
        """开始巡检测试"""
        logger.info("🚀 开始巡检路径测试 (WebSocket)")
        logger.info(f"📡 {self.ws_url}")

        async with WsNavClient(self.ws_url) as client:
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

            logger.info(
                f"📋 本次测试点位序列 ({len(self.patrol_sequence)}): "
                f"{self.patrol_sequence}"
            )

            logger.info("🔍 检查服务器连接和机器人状态...")
            status_data = await client.call("get_status")
            if not status_data.get("success"):
                logger.error("❌ 无法获取机器人状态")
                return False

            current_pos = status_data["position"]
            logger.info(f"📍 当前位置: x={current_pos['x']:.3f}, y={current_pos['y']:.3f}")

            nearest_data = await client.call("get_nearest_point")
            if nearest_data.get("success"):
                logger.info(
                    f"📍 最接近的预设点: {nearest_data['nearest_point']} "
                    f"(距离: {nearest_data['distance']}m)"
                )

            if self.continue_tolerance >= 0 and len(self.patrol_sequence) > 1:
                tol = self.continue_tolerance
                cur_x = current_pos['x']
                cur_y = current_pos['y']
                best_idx = 0
                best_dist = float('inf')
                for idx, wp in enumerate(self.patrol_sequence):
                    wp_name = (wp.get('semantic_point') or wp.get('name')) if isinstance(wp, dict) else wp
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
                target_name = None
                target_submap = None

                if is_route_dict:
                    target_submap = waypoint.get("submap")
                    target_name = (
                        waypoint.get("semantic_point")
                        or waypoint.get("point")
                        or waypoint.get("name")
                    )
                    if not target_name:
                        logger.error(f"❌ 路线点格式不正确: {waypoint}")
                        continue
                else:
                    target_name = waypoint

                if not is_route_dict and target_name not in self.pose_list:
                    logger.error(f"❌ 点位 {target_name} 不存在于服务器点位列表中")
                    continue

                start_time = time.time()

                pre_status_data = await client.call("get_status")
                if pre_status_data.get("success"):
                    pre_pos = pre_status_data["position"]
                    logger.info(f"📍 下发前位置: x={pre_pos['x']:.3f}, y={pre_pos['y']:.3f}")

                if is_route_dict and target_submap:
                    cur_map_data = await client.call("get_current_map_id")
                    current_map_id = cur_map_data.get("map_id")

                    if current_map_id != target_submap:
                        logger.info(
                            f"🔁 目标 {target_name} 在子地图 {target_submap}, "
                            f"当前子地图为 {current_map_id}, 需要切换"
                        )
                        portal_data = await client.call(
                            "get_portal_to_submap", {"target_submap": target_submap}
                        )
                        if portal_data.get("success") and portal_data.get("portal_pose"):
                            portal_pose = portal_data["portal_pose"]
                            px = portal_pose["position"]["x"]
                            py = portal_pose["position"]["y"]
                            logger.info(
                                f"🚪 导航到传送门 {portal_data.get('portal')} at x={px}, y={py}"
                            )
                            goal_data = await client.call(
                                "set_goal_by_xy", {"x": px, "y": py}
                            )
                            if not goal_data.get("success"):
                                logger.error(f"❌ 前往传送门失败: {goal_data.get('msg')}")
                                continue

                            logger.info("⏳ 等待到达传送门...")
                            wait_portal = await self.wait_goal_with_pause(
                                client, f"portal_{i}", timeout=120.0
                            )
                            if not wait_portal.get("success"):
                                logger.error("❌ 到达传送门超时或失败，跳过此目标")
                                continue

                            change_data = await client.call(
                                "change_map", {"map_id": target_submap}
                            )
                            if not change_data.get("success"):
                                logger.error(f"❌ 切换地图失败: {change_data.get('msg')}")
                                continue
                            logger.info(f"✅ 切换地图成功: {target_submap}")
                            
                            init_pose = change_data.get("init_pose")
                            if init_pose:
                                logger.info(f"📍 切换后初始位姿: {init_pose}")
                            logger.info("⏳ 切换地图后等待 3s...")
                            await asyncio.sleep(3.0)
                        else:
                            logger.error(
                                f"❌ 未找到从 {current_map_id} 到 {target_submap} 的传送门: "
                                f"{portal_data.get('msg')}"
                            )
                            continue

                point_meta = await client.call("get_pose_in_submap", {"pose_name": target_name})
                pt_type = point_meta.get("point_info", {}).get("type") if point_meta.get("success") else None
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
                goal_data = await client.call("set_goal_by_name", {"target": target_name})

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
            start_time = time.time()
            paused_duration = 0.0
            pause_start: Optional[float] = None

            self.is_paused = False

            with KeyboardHandler() as kb:
                while True:
                    elapsed = time.time() - start_time - paused_duration
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
        default=os.environ.get("NAV_WS_URL", "ws://localhost:9999/ws"),
        help="WebSocket URL，默认 ws://localhost:9999/ws（与 server -t 端口一致）",
    )
    parser.add_argument(
        "-r",
        "--route",
        default=None,
        metavar="NAME",
        help="路线名称；省略则使用服务器返回的第一条路线（当 -p 未指定时生效）",
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
        "-s",
        "--submap",
        default=None,
        metavar="ID",
        help="目标子地图 ID；若当前不在该子地图则先切换",
    )
    parser.add_argument(
        "--set-xy",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="测试 set_goal_by_xy: 提供 X Y 坐标",
    )
    args = parser.parse_args()

    try:
        # --submap: 先切换到目标子地图（适用于所有后续操作）
        if args.submap:
            async with WsNavClient(args.ws) as client:
                sc = SubmapClient(client)
                if not await sc.ensure_submap(args.submap):
                    return 1

        # If --set-xy provided, run a minimal test for set_goal_by_xy + get_navigation_plan
        if args.set_xy:
            x, y = args.set_xy
            logger.info(f"🔬 Running set_goal_by_xy test to x={x}, y={y} on {args.ws}")
            async with WsNavClient(args.ws) as client:
                res = await client.call("set_goal_by_xy", {"x": float(x), "y": float(y), "distance": 0.05})
                logger.info(f"set_goal_by_xy response: {res}")
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
            client = SimplePatrolClient(
                ws_url=args.ws,
                route_name=args.route,
                patrol_sequence=patrol_seq,
                continue_tolerance=args.continue_tolerance,
                auto_confirm=args.yes,
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
