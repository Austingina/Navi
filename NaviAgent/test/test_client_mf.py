#!/usr/bin/env python3
"""
简单的MCP客户端测试脚本 - 测试525.json中的巡检点位
"""

import asyncio
import json
import time
import logging
import sys
import select
import termios
import tty
from typing import Dict
from fastmcp import Client

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 关闭 httpx 的请求日志
logging.getLogger("httpx").setLevel(logging.WARNING)


class KeyboardHandler:
    """非阻塞键盘输入处理器"""
    def __init__(self):
        self.old_settings = None
        
    def __enter__(self):
        """进入上下文时设置终端为原始模式"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except:
            pass
        return self
    
    def __exit__(self, *args):
        """退出上下文时恢复终端设置"""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
    
    def check_key(self) -> str:
        """非阻塞检查是否有按键，返回按键字符或空字符串"""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except:
            pass
        return ""

class SimplePatrolClient:
    def __init__(self, server_url: str = "http://localhost:9999/mcp"):
        self.server_url = server_url
        self.client = None
        self.pose_list = []  # 从服务器获取的点位列表
        self.is_paused = False  # 暂停状态
        
        # 巡检序列（按顺序）
        self.patrol_sequence = [
            # "charging point start",
            # "control point 1",
            # "control point 2",
            # "control point 3",
            # "control point 4",
            # "control point 5",
            # "control point 6",
            # "control point 7",
            # "control point 8",
            # "control point 9",
            # "control point 10", 
            # "control point 11", 
            # "control point 12", 
            # "control point 13", 
            # "control point 14", 
            # "control point 15", 
            # "control point 16", 
            # "control point 17",
            # "control point 18",
            # "charging point end" 
        ]
        # 优先尝试从服务器获取名为 ROUTE_NAME 的路线（可通过环境变量设置）
        import os
        self.route_name ='线路A'

    async def start_patrol_test(self):
        """开始巡检测试"""
        logger.info("🚀 开始巡检路径测试")
        logger.info(f"📋 测试点位: {' -> '.join(self.patrol_sequence)}")
        
        async with Client(self.server_url) as client:
            self.client = client
            
            # 1. 获取所有预设点列表
            pose_list_resp = await client.call_tool("get_pose_list_in_current_submap")
            pose_list_data = json.loads(pose_list_resp.content[0].text)
            if pose_list_data.get("success"):
                self.pose_list = pose_list_data['keys']
                logger.info(f"📋 目的地列表: {self.pose_list}")
            else:
                logger.error(f"❌ 获取目的地列表失败: {pose_list_data.get('msg')}")
                return False

            # 如果配置了 route_name，尝试从服务器获取路线并使用
            route_waypoints = None
            if self.route_name:
                logger.info(f"📌 配置了路线名称: {self.route_name}, 尝试从服务器获取对应路线")
                try:
                    logger.info(f"🔎 尝试获取路线: {self.route_name}")
                    route_resp = await client.call_tool("get_route", {"route_name": self.route_name})
                    route_data = json.loads(route_resp.content[0].text)
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

            # 1. 获取初始状态
            logger.info("🔍 检查服务器连接和机器人状态...")
            status = await client.call_tool("get_status")
            status_data = json.loads(status.content[0].text)
            if not status_data.get("success"):
                logger.error("❌ 无法获取机器人状态")
                return False
            
            current_pos = status_data["position"]
            logger.info(f"📍 当前位置: x={current_pos['x']:.3f}, y={current_pos['y']:.3f}")


            # 2. 获取最近的预设点
            nearest = await client.call_tool("get_nearest_point")
            nearest_data = json.loads(nearest.content[0].text)
            if nearest_data.get("success"):
                logger.info(f"📍 最接近的预设点: {nearest_data['nearest_point']} (距离: {nearest_data['distance']}m)")
            
            # 3. 依次访问每个巡检点
            success_count = 0
            total_time = 0
            
            for i, waypoint in enumerate(self.patrol_sequence):
                logger.info(f"\n{'='*50}")
                logger.info(f"📍 测试点位 {i+1}/{len(self.patrol_sequence)}: {waypoint}")

                # 如果路线点是字典（包含 submap 信息），按跨子地图逻辑处理
                is_route_dict = isinstance(waypoint, dict)
                target_name = None
                target_submap = None

                if is_route_dict:
                    # 期待字段名可能为 'semantic_point' 或 'point' 或 'name'
                    target_submap = waypoint.get('submap')
                    target_name = waypoint.get('semantic_point') or waypoint.get('point') or waypoint.get('name')
                    if not target_name:
                        logger.error(f"❌ 路线点格式不正确: {waypoint}")
                        continue
                else:
                    target_name = waypoint

                # 如果不是路由结构，验证点在服务器点位列表中
                if not is_route_dict and target_name not in self.pose_list:
                    logger.error(f"❌ 点位 {target_name} 不存在于服务器点位列表中")
                    continue
                
                start_time = time.time()
                
                # 获取下发前的位置
                pre_status = await client.call_tool("get_status")
                pre_status_data = json.loads(pre_status.content[0].text)
                if pre_status_data.get("success"):
                    pre_pos = pre_status_data["position"]
                    logger.info(f"📍 下发前位置: x={pre_pos['x']:.3f}, y={pre_pos['y']:.3f}")
                
                # 若路由点包含 submap 信息，检查是否需切换子地图
                if is_route_dict and target_submap:
                    # 获取当前子地图 ID
                    cur_map_resp = await client.call_tool("get_current_map_id")
                    cur_map_data = json.loads(cur_map_resp.content[0].text)
                    current_map_id = cur_map_data.get('map_id')

                    # 如果目标不在当前子地图，需要先导航到 portal 并切换地图
                    if current_map_id != target_submap:
                        logger.info(f"🔁 目标 {target_name} 在子地图 {target_submap}, 当前子地图为 {current_map_id}, 需要切换")
                        # 获取从当前子地图到目标子地图的 portal
                        portal_resp = await client.call_tool("get_portal_to_submap", {"target_submap": target_submap})
                        portal_data = json.loads(portal_resp.content[0].text)
                        if portal_data.get('success') and portal_data.get('portal_pose'):
                            portal_pose = portal_data['portal_pose']
                            px = portal_pose['position']['x']
                            py = portal_pose['position']['y']
                            logger.info(f"🚪 导航到传送门 {portal_data.get('portal')} at x={px}, y={py}")
                            # 使用 set_goal_by_xy 导航到传送门位置
                            goal_result = await client.call_tool("set_goal_by_xy", {"x": px, "y": py})
                            goal_data = json.loads(goal_result.content[0].text)
                            if not goal_data.get('success'):
                                logger.error(f"❌ 前往传送门失败: {goal_data.get('msg')}")
                                continue

                            # 等待到达传送门
                            logger.info("⏳ 等待到达传送门...")
                            wait_portal = await self.wait_goal_with_pause(client, f"portal_{i}", timeout=1200.0)
                            if not wait_portal.get('success'):
                                logger.error("❌ 到达传送门超时或失败，跳过此目标")
                                continue

                            # 切换地图
                            change_resp = await client.call_tool("change_map", {"map_id": target_submap})
                            change_data = json.loads(change_resp.content[0].text)
                            if not change_data.get('success'):
                                logger.error(f"❌ 切换地图失败: {change_data.get('msg')}")
                                continue
                            logger.info(f"✅ 切换地图成功: {target_submap}")
                            # 切换后可以使用 init_pose 或直接下发目标点
                            # 若 change_map 返回 init_pose，可记录
                            init_pose = change_data.get('init_pose')
                            if init_pose:
                                logger.info(f"📍 切换后初始位姿: {init_pose}")
                        else:
                            logger.error(f"❌ 未找到从 {current_map_id} 到 {target_submap} 的传送门: {portal_data.get('msg')}")
                            continue

                # 下发目标点
                logger.info(f"🎯 下发目标点: {target_name}")
                # 直接发送目标点名称，让服务端处理数据类型转换
                goal_result = await client.call_tool("set_goal", {"target": target_name})
                goal_data = json.loads(goal_result.content[0].text)
                
                if not goal_data.get("success"):
                    logger.error(f"❌ 下发目标失败: {goal_data.get('msg')}")
                    continue
                
                logger.info(f"✅ 目标点下发成功: {goal_data.get('msg')}")
                
                # 等待2秒让导航系统响应
                logger.info("⏳ 等待导航系统响应...")
                await asyncio.sleep(2)
                
                # 检查机器人是否开始移动
                post_status = await client.call_tool("get_status")
                post_status_data = json.loads(post_status.content[0].text)
                if post_status_data.get("success"):
                    post_pos = post_status_data["position"]
                    # vel = post_status_data.get("linear_velocity", {})
                    logger.info(f"📍 响应后位置: x={post_pos['x']:.3f}, y={post_pos['y']:.3f}")
                    # logger.info(f"🏃 当前速度: vx={vel.get('x', 0):.3f}, vy={vel.get('y', 0):.3f}")
                
                # 等待到达目标点
                logger.info(f"⏳ 等待到达目标点 (超时: 1200秒)... [按空格键暂停/恢复]")
                wait_result = await self.wait_goal_with_pause(client, target_name, timeout=1200.0, xy_tolerance=0.6, yaw_tolerance=3.14)
                wait_data = wait_result
                
                duration = time.time() - start_time
                
                if wait_data.get("success"):
                    logger.info(f"🎉 成功到达 {waypoint} (耗时: {duration:.2f}秒)")
                    success_count += 1
                    total_time += duration
                    
                    # 获取到达后的位置
                    current_status = await client.call_tool("get_status")
                    current_status_data = json.loads(current_status.content[0].text)
                    if current_status_data.get("success"):
                        pos = current_status_data["position"]
                        logger.info(f"📍 到达后位置: x={pos['x']:.3f}, y={pos['y']:.3f}")
                    
                    # 点位间暂停2秒
                    if i < len(self.patrol_sequence) - 1:
                        logger.info("⏸️  暂停2秒后发送下一个目标...")
                        await asyncio.sleep(2)
                        
                else:
                    logger.error(f"❌ 未能到达 {waypoint}: {wait_data.get('msg')}")
                    
                    # 获取当前位置看到底在哪
                    current_status = await client.call_tool("get_status")
                    current_status_data = json.loads(current_status.content[0].text)
                    if current_status_data.get("success"):
                        pos = current_status_data["position"]
                        logger.info(f"📍 失败时位置: x={pos['x']:.3f}, y={pos['y']:.3f}")
                    
                    # 失败时询问是否继续（这里先自动停止测试）
                    logger.error(f"🛑 由于未能到达 {waypoint}，测试中止")
                    logger.error("💡 请检查机器人导航系统和路径规划")
                    break  # 停止测试，不发送下一个目标
            
            # 生成测试报告
            self.generate_report(success_count, len(self.patrol_sequence), total_time)
            return success_count == len(self.patrol_sequence)

    async def wait_goal_with_pause(self, client, waypoint: str, timeout: float = 120.0, 
                                    xy_tolerance: float = 0.6, yaw_tolerance: float = 3.14) -> Dict:
        """
        等待到达目标点，支持空格键暂停/恢复
        
        参数:
            client: MCP客户端
            waypoint: 目标点名称（字符串）
            timeout: 超时时间（不含暂停时间）
            xy_tolerance: 位置容差
            yaw_tolerance: 角度容差
            
        返回:
            Dict: 到达结果
        """
        start_time = time.time()
        paused_duration = 0.0  # 累计暂停时间
        pause_start = None
        
        # 重置暂停状态
        self.is_paused = False
        
        with KeyboardHandler() as kb:
            while True:
                # 计算有效已用时间（排除暂停时间）
                elapsed = time.time() - start_time - paused_duration
                if elapsed >= timeout:
                    break
                    
                # 检查空格键
                key = kb.check_key()
                if key == ' ':
                    if not self.is_paused:
                        # 暂停导航
                        self.is_paused = True
                        pause_start = time.time()
                        logger.info("⏸️  [空格键] 导航已暂停，再次按空格键继续...")
                        pause_result = await client.call_tool("pause_navigation")
                        pause_data = json.loads(pause_result.content[0].text)
                        if not pause_data.get("success"):
                            logger.warning(f"暂停失败: {pause_data.get('msg')}")
                    else:
                        # 恢复导航
                        self.is_paused = False
                        if pause_start:
                            paused_duration += time.time() - pause_start
                            pause_start = None
                        logger.info("▶️  [空格键] 导航已恢复...")
                        resume_result = await client.call_tool("resume_navigation")
                        resume_data = json.loads(resume_result.content[0].text)
                        if not resume_data.get("success"):
                            logger.warning(f"恢复失败: {resume_data.get('msg')}")
                
                # 如果暂停中，不检查是否到达，只等待
                if self.is_paused:
                    await asyncio.sleep(0.2)
                    continue
                
                # 检查是否到达目标点
                try:
                    # 使用短超时轮询检查
                    wait_result = await client.call_tool("wait_goal_reached", {
                        "target": waypoint,
                        "timeout": 0.5,  # 短超时，便于响应键盘
                        "xy_tolerance": xy_tolerance,
                        "yaw_tolerance": yaw_tolerance
                    })
                    wait_data = json.loads(wait_result.content[0].text)
                    
                    if wait_data.get("success"):
                        return wait_data
                except Exception as e:
                    logger.debug(f"等待检查异常: {e}")
                
                await asyncio.sleep(0.1)
        
        return {"success": False, "msg": "等待超时或未到达"}

    def generate_report(self, success_count: int, total_count: int, total_time: float):
        """生成测试报告"""
        logger.info(f"\n{'='*60}")
        logger.info("📊 巡检测试报告")
        logger.info(f"{'='*60}")
        
        success_rate = (success_count / total_count) * 100
        
        logger.info(f"📈 测试结果:")
        logger.info(f"   • 总点位数量: {total_count}")
        logger.info(f"   • 成功点位: {success_count}")
        logger.info(f"   • 失败点位: {total_count - success_count}")
        logger.info(f"   • 成功率: {success_rate:.1f}%")
        logger.info(f"   • 总耗时: {total_time:.2f}秒")
        logger.info(f"   • 平均每点耗时: {total_time/success_count:.2f}秒" if success_count > 0 else "")

async def main():
    """主函数"""
    try:
        client = SimplePatrolClient()
        success = await client.start_patrol_test()
        
        if success:
            logger.info("\n🎉 所有巡检点位测试通过！")
            return 0
        else:
            logger.error("\n⚠️  部分巡检点位测试失败")
            return 1
            
    except KeyboardInterrupt:
        logger.info("\n⏹️  用户中断测试")
        return 2
    except Exception as e:
        logger.error(f"\n💥 测试运行异常: {e}")
        return 3

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code) 