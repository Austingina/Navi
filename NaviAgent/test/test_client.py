#!/usr/bin/env python3
"""
简单的MCP客户端测试脚本 - 测试525.json中的巡检点位
"""

import asyncio
import json
import time
import logging
from fastmcp import Client

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SimplePatrolClient:
    def __init__(self, server_url: str = "http://localhost:9999/mcp"):
        self.server_url = server_url
        self.client = None
        
        # 从525.json读取巡检点位（仅用于调试显示）
        with open("../config/525.json", "r", encoding="utf-8") as f:
            self.patrol_data = json.load(f)
        
        # 巡检序列（按顺序）
        self.patrol_sequence = [
            "charging point start", 
            "control point 1", 
            "control point 2", 
            "control point 3", 
            "control point 4", 
            "control point 5", 
            "control point 6", 
            "control point 7", 
            "control point 8", 
            "control point 9", 
            "control point 10", 
            "control point 11", 
            "control point 12", 
            "control point 13", 
            "control point 14", 
            "control point 15", 
            "control point 16", 
            "control point 17",
            "charging point end" 
        ]

    async def start_patrol_test(self):
        """开始巡检测试"""
        logger.info("🚀 开始巡检路径测试")
        logger.info(f"📋 测试点位: {' -> '.join(self.patrol_sequence)}")
        
        async with Client(self.server_url) as client:
            self.client = client
            
            # 1. 获取所有预设点列表
            pose_list_resp = await client.call_tool("get_pose_list")
            pose_list_data = json.loads(pose_list_resp.content[0].text)
            if pose_list_data.get("success"):
                logger.info(f"📋 目的地列表: {pose_list_data['keys']}")
            else:
                logger.error(f"❌ 获取目的地列表失败: {pose_list_data.get('msg')}")


            # 1. 获取初始状态
            logger.info("🔍 检查服务器连接和机器人状态...")
            status = await client.call_tool("get_status")
            status_data = json.loads(status.content[0].text)
            if not status_data.get("success"):
                logger.error("❌ 无法获取机器人状态")
                return False
            
            current_pos = status_data["position"]
            logger.info(f"📍 当前位置: x={current_pos['x']:.3f}, y={current_pos['y']:.3f}")
            


            # # 新增：测试 set_goal_by_lonlat
            # logger.info("\n🌐 测试 set_goal_by_lonlat 接口")
            # # 示例经纬度（请替换为实际可用的经纬度）
            # test_lon, test_lat = 116.391389, 39.9075
            # lonlat_result = await client.call_tool("set_goal_by_lonlat", {"lon": test_lon, "lat": test_lat})
            # lonlat_data = json.loads(lonlat_result.content[0].text)
            # if lonlat_data.get("success"):
            #     logger.info(f"✅ set_goal_by_lonlat 成功: {lonlat_data.get('msg')}")
            # else:
            #     logger.error(f"❌ set_goal_by_lonlat 失败: {lonlat_data.get('msg')}")

            # # 新增：测试 set_goal_by_xy
            # logger.info("\n🗺️  测试 set_goal_by_xy 接口")
            # # 示例 map 坐标（请替换为实际可用的坐标）
            # test_x, test_y = 1.0, 1.0
            # xy_result = await client.call_tool("set_goal_by_xy", {"x": test_x, "y": test_y})
            # xy_data = json.loads(xy_result.content[0].text)
            # if xy_data.get("success"):
            #     logger.info(f"✅ set_goal_by_xy 成功: {xy_data.get('msg')}")
            # else:
            #     logger.error(f"❌ set_goal_by_xy 失败: {xy_data.get('msg')}")






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
                
                # 检查点位是否存在于525.json中
                if waypoint not in self.patrol_data:
                    logger.error(f"❌ 点位 {waypoint} 不存在于525.json中")
                    continue
                
                start_time = time.time()
                
                # 获取下发前的位置
                pre_status = await client.call_tool("get_status")
                pre_status_data = json.loads(pre_status.content[0].text)
                if pre_status_data.get("success"):
                    pre_pos = pre_status_data["position"]
                    logger.info(f"📍 下发前位置: x={pre_pos['x']:.3f}, y={pre_pos['y']:.3f}")
                
                # 下发目标点
                logger.info(f"🎯 下发目标点: {waypoint}")
                logger.info(f"🔍 调用参数: {{'target': '{waypoint}'}}")
                
                # 调试：检查525.json中这个点位的数据
                if waypoint in self.patrol_data:
                    point_data = self.patrol_data[waypoint]
                    logger.info(f"🔍 点位数据: {point_data}")
                    logger.info(f"🔍 z坐标类型: {type(point_data['pose']['position']['z'])}, 值: {point_data['pose']['position']['z']}")
                
                # 直接发送目标点名称，让服务端处理数据类型转换
                goal_result = await client.call_tool("set_goal", {"target": waypoint})
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
                logger.info(f"⏳ 等待到达目标点 (超时: 120秒)...")
                wait_result = await client.call_tool("wait_goal_reached", {
                    "target": waypoint, 
                    "timeout": 120.0
                })
                wait_data = json.loads(wait_result.content[0].text)
                
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