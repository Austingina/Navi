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
from typing import Any, Dict, Optional

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
        self._ws = await websockets.connect(self.url, ping_interval=20, ping_timeout=20)

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
    def __init__(self, ws_url: str = "ws://localhost:9999/ws"):
        self.ws_url = ws_url
        self.pose_list: list = []
        self.is_paused = False

        self.patrol_sequence: list = []

        self.route_name = os.environ.get("ROUTE_NAME", "线路A")

    async def start_patrol_test(self) -> bool:
        """开始巡检测试"""
        logger.info("🚀 开始巡检路径测试 (WebSocket)")
        logger.info(f"📡 {self.ws_url}")
        logger.info(f"📋 测试点位: {' -> '.join(map(str, self.patrol_sequence))}")

        async with WsNavClient(self.ws_url) as client:
            pose_list_data = await client.call("get_pose_list_in_current_submap")
            if pose_list_data.get("success"):
                self.pose_list = pose_list_data["keys"]
                logger.info(f"📋 目的地列表: {self.pose_list}")
            else:
                logger.error(f"❌ 获取目的地列表失败: {pose_list_data.get('msg')}")
                return False

            route_waypoints = None
            if self.route_name:
                logger.info(f"📌 配置了路线名称: {self.route_name}, 尝试从服务器获取对应路线")
                try:
                    route_data = await client.call(
                        "get_route", {"route_name": self.route_name}
                    )
                    if route_data.get("success") and route_data.get("waypoints"):
                        route_waypoints = route_data.get("waypoints")
                        logger.info(f"✅ 获取到路线 {self.route_name}, waypoints={len(route_waypoints)}")
                    else:
                        logger.warning(f"无法获取路线 {self.route_name}, 将使用默认点位列表")
                except Exception as e:
                    logger.warning(f"获取路线异常: {e}")

            if route_waypoints:
                self.patrol_sequence = route_waypoints
                logger.info(f"📋 使用服务器提供的路线点位: {self.patrol_sequence}")

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
                                client, f"portal_{i}", timeout=1200.0
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
                        else:
                            logger.error(
                                f"❌ 未找到从 {current_map_id} 到 {target_submap} 的传送门: "
                                f"{portal_data.get('msg')}"
                            )
                            continue

                logger.info(f"🎯 下发目标点: {target_name}")
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
                    "⏳ 等待到达目标点 (超时: 1200秒)... [按空格键暂停/恢复]"
                )
                wait_data = await self.wait_goal_with_pause(
                    client, target_name, timeout=1200.0, xy_tolerance=0.6, yaw_tolerance=3.14
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
        """等待到达目标点，支持空格键暂停/恢复"""
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
                except Exception as e:
                    logger.debug(f"等待检查异常: {e}")

                await asyncio.sleep(0.1)

        return {"success": False, "msg": "等待超时或未到达"}

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
        "--set-xy",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="测试 set_goal_by_xy: 提供 X Y 坐标",
    )
    args = parser.parse_args()

    try:
        # If --set-xy provided, run a minimal test for set_goal_by_xy and wait_goal_reached
        if args.set_xy:
            x, y = args.set_xy
            logger.info(f"🔬 Running set_goal_by_xy test to x={x}, y={y} on {args.ws}")
            async with WsNavClient(args.ws) as client:
                # send set_goal_by_xy
                res = await client.call("set_goal_by_xy", {"x": float(x), "y": float(y), "distance": 0.05})
                logger.info(f"set_goal_by_xy response: {res}")

                # try to determine a target identifier to wait for
                target = res.get("target_str") or res.get("target") or f"xy({x},{y})"

                logger.info("等待到达目标（timeout=220s）...")
                wait = await client.call("wait_goal_reached", {"target": target, "timeout": 220.0})
                logger.info(f"wait_goal_reached response: {wait}")
                return 0 if wait.get("success") else 1

        # otherwise run full patrol test
        client = SimplePatrolClient(ws_url=args.ws)
        success = await client.start_patrol_test()

        if success:
            logger.info("\n🎉 所有巡检点位测试通过！")
            return 0
        logger.error("\n⚠️  部分巡检点位测试失败")
        return 1

    except KeyboardInterrupt:
        logger.info("\n⏹️  用户中断测试")
        return 2
    except Exception as e:
        logger.exception("\n💥 测试运行异常: %s", e)
        return 3


if __name__ == "__main__":
    exit(asyncio.run(main()))
