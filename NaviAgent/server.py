# 导入必要的库
import time  # 时间处理库
from typing import Dict, Any  # 类型提示
import asyncio  # 异步IO库
import logging  # 日志记录库
import json  # JSON数据处理库
import os  # 操作系统接口库
import threading  # 线程处理库
import numpy as np  # 数值计算库
# 导入自定义模块
from wgs84 import WGS84Converter 
from nav_node import NavNode  # 导入导航节点类
from map_manager import MapManager  # 导入多楼层地图管理器
# 导入FastMCP框架
from fastmcp import FastMCP
# 导入ROS相关的库
import rclpy  # ROS2 Python客户端库
from rclpy.executors import SingleThreadedExecutor  # 单线程执行器
from geometry_msgs.msg import PoseStamped  # 位姿消息类型
import argparse  # 添加在文件顶部
# 加载地图配置文件的函数
# 直接在此定义 load_intent_pose_map，兼容 intent_pose_map.json 的多种结构

# 导航工具管理器类
class NavigationToolManager:
    """
    导航工具管理器
    
    功能：
    1. 初始化MCP服务和ROS节点
    2. 注册导航相关的工具函数
    3. 管理ROS线程和资源清理
    """
    def __init__(self, map_manager: MapManager = None):
        """
        初始化导航工具管理器
        
        :param map_manager: 多楼层地图管理器实例，可选
        """
        # 创建FastMCP实例
        self.mcp = FastMCP("NavigationMCP")
        # 多楼层地图管理器
        self.map_manager = map_manager
        # 初始化经纬度转换器
        self.wgs84_converter = WGS84Converter()
        self.wgs84_converter.initialize()
        # 初始化ROS
        try:
            # # 创建导航节点实例
            self.ros_node = NavNode()
            # 创建单线程执行器
            self.executor = SingleThreadedExecutor()
            # 将节点添加到执行器
            self.executor.add_node(self.ros_node)
            
            # 启动ROS spin线程
            # 创建后台守护线程运行ROS消息循环
            self.ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self.ros_thread.start()
        except Exception as e:
            logging.error(f"ROS初始化失败: {e}")
            raise

        # 注册MCP工具函数
        self.register_tools()

    def _ros_spin_thread(self):
        """
        在独立线程中运行ROS spin
        
        功能：
        1. 在后台线程中持续运行ROS消息循环
        2. 处理订阅的消息和服务调用
        """
        """在独立线程中运行ROS spin"""
        try:
            # 启动ROS执行器，开始处理消息
            self.executor.spin()
        except Exception as e:
            logging.error(f"ROS spin线程出错: {e}")


    def register_tools(self):
        """
        注册MCP工具函数
        
        功能：
        1. 定义并注册导航相关的工具函数
        2. 包括设置目标、获取状态、等待到达、获取最近点等功能
        """
        @self.mcp.tool()
        def set_goal(target: str) -> Dict[str, Any]:
            """
            下发导航目标
            可用目标点：狗巢、配电房、灭火器箱、液晶大屏、会议桌
            参数:
            target: 目标点名称（必须在预设地图中）
            
            返回:
            Dict: 包含操作结果的字典
            """
            """下发导航目标"""
            try:
                # 检查目标点是否在预设地图中
                if target not in self.map_manager.pose_map_current_submap:
                    return {"success": False, "msg": f"目标点 {target} 不在白名单中"}
                
                # 获取目标点的位姿信息
                pose_info = self.map_manager.pose_map_current_submap[target].get("pose")
                if not pose_info:
                    return {"success": False, "msg": f"目标点 {target} 缺少坐标信息"}
                
                # 创建PoseStamped消息
                pose = PoseStamped()
                # 设置坐标系为里程计坐标系
                pose.header.frame_id = "odom"
                # 设置位置信息
                pose.pose.position.x = pose_info["position"]["x"]
                pose.pose.position.y = pose_info["position"]["y"]
                pose.pose.position.z = pose_info["position"]["z"]
                # 设置姿态信息（四元数）
                pose.pose.orientation.x = pose_info["orientation"]["x"]
                pose.pose.orientation.y = pose_info["orientation"]["y"]
                pose.pose.orientation.z = pose_info["orientation"]["z"]
                pose.pose.orientation.w = pose_info["orientation"]["w"]
                
                # 发布目标点
                self.ros_node.publish_goal(pose,target)
                return {"success": True, "msg": f"目标 {target} 已下发", "target": target}
            except Exception as e:
                return {"success": False, "msg": f"设置目标失败: {str(e)}"}

        @self.mcp.tool()
        async def set_goal_by_multipose(target: str, timeout_per_pose: float = 60.0, xy_tolerance: float = 0.04, yaw_tolerance: float = 0.05) -> Dict[str, Any]:
            """
            导航到multi_pose类型点位的所有位姿，依次到达每个位姿点
            使用PID控制直接到达目标，跳过nav2导航
            
            参数:
            target: 目标点名称（必须是multi_pose类型）
            timeout_per_pose: 每个位姿点的超时时间（秒），默认120秒
            xy_tolerance: 位置容差（米），默认0.04米
            yaw_tolerance: 姿态容差（弧度），默认0.05弧度
            
            返回:
            Dict: 包含操作结果的字典，包括每个位姿点的到达状态
            """
            try:
                # 检查目标点是否在预设地图中
                if target not in self.map_manager.pose_map_current_submap:
                    return {"success": False, "msg": f"目标点 {target} 不在白名单中"}
                
                # 检查是否为multi_pose类型
                if not self.map_manager.is_multi_pose(target):
                    return {"success": False, "msg": f"目标点 {target} 不是multi_pose类型，请使用set_goal接口"}
                
                # 获取所有位姿
                poses_list = self.map_manager.get_multi_poses_in_submap(target)
                if not poses_list:
                    return {"success": False, "msg": f"目标点 {target} 缺少位姿信息"}
                
                total_poses = len(poses_list)
                results = []
                
                # 依次导航到每个位姿点
                for idx, pose_info in enumerate(poses_list):
                    # 创建PoseStamped消息
                    pose = PoseStamped()
                    pose.header.frame_id = "odom"
                    pose.pose.position.x = pose_info["position"]["x"]
                    pose.pose.position.y = pose_info["position"]["y"]
                    pose.pose.position.z = pose_info["position"]["z"]
                    pose.pose.orientation.x = pose_info["orientation"]["x"]
                    pose.pose.orientation.y = pose_info["orientation"]["y"]
                    pose.pose.orientation.z = pose_info["orientation"]["z"]
                    pose.pose.orientation.w = pose_info["orientation"]["w"]
                    
                    # 直接设置last_goal，跳过nav2导航
                    self.ros_node.last_goal = pose
                    self.ros_node.pid_control_active = True
                    self.ros_node.pid_rotation_phase = True  # 先调整yaw角度
                    
                    logging.info(f"[multipose] 开始PID控制到位姿点 {idx+1}/{total_poses}: x={pose_info['position']['x']}, y={pose_info['position']['y']}")
                    
                    # 使用PID控制到达当前位姿点
                    start_time = time.time()
                    reached = False
                    while time.time() - start_time < timeout_per_pose:
                        await asyncio.sleep(0.05)  # 50ms控制周期
                        # 执行一步PID控制
                        if self.ros_node.pid_control_step(yaw_tolerance, xy_tolerance):
                            reached = True
                            break
                    
                    # 停止机器人
                    self.ros_node.pid_control_active = False
                    
                    pose_result = {
                        "pose_index": idx,
                        "position": pose_info["position"],
                        "reached": reached,
                        "elapsed_time": round(time.time() - start_time, 2)
                    }
                    results.append(pose_result)
                    
                    logging.info(f"[multipose] 位姿点 {idx+1}/{total_poses} {'到达' if reached else '超时'}, 耗时: {pose_result['elapsed_time']}s")
                    
                    # 如果未到达当前位姿点，返回部分结果
                    if not reached:
                        return {
                            "success": False,
                            "msg": f"在位姿点 {idx + 1}/{total_poses} 超时未到达",
                            "target": target,
                            "total_poses": total_poses,
                            "completed_poses": idx,
                            "results": results
                        }
                
                # 所有位姿点都已到达
                return {
                    "success": True,
                    "msg": f"已完成 {target} 的所有 {total_poses} 个位姿点导航",
                    "target": target,
                    "total_poses": total_poses,
                    "completed_poses": total_poses,
                    "results": results
                }
            except Exception as e:
                return {"success": False, "msg": f"多位姿导航失败: {str(e)}"}

        @self.mcp.tool()
        def set_goal_by_lonlat(lon: float, lat: float, distance: float = 1.5) -> Dict[str, Any]:
            """
            根据输入的经纬度导航到目标点附近
            步骤：
            1. 经纬度转map坐标系
            2. 判断是否在map bound范围内
            3. 计算距离目标点distance距离的实际目标位置
            4. 调用publish_goal发起导航

            参数:
            lon: 目标点经度
            lat: 目标点纬度
            distance: 距离目标点的距离，默认1.5米

            返回:
            Dict: 操作结果
            """
            try:
                # 将经纬度转换为地图坐标系
                x, y = self.wgs84_converter.lonlat_to_xy(lon, lat)
                
                # 检查转换后的坐标是否在地图边界内
                if not (self.ros_node.map_bound["xmin"] <= x <= self.ros_node.map_bound["xmax"] and
                        self.ros_node.map_bound["ymin"] <= y <= self.ros_node.map_bound["ymax"]):
                    return {"success": False, "msg": f"目标点({lon}, {lat})转换后坐标({x:.2f}, {y:.2f})超出地图边界"}
                
                # 获取当前位置
                odom = self.ros_node.current_pose
                if not odom:
                    return {"success": False, "msg": "无法获取当前位置信息"}
                
                curr_x = odom.pose.position.x
                curr_y = odom.pose.position.y
                
                # 计算从当前位置到目标点的方向向量
                dx = x - curr_x
                dy = y - curr_y
                target_distance = (dx**2 + dy**2) ** 0.5
                
                # 如果当前距离目标点小于指定距离，则直接面向目标点
                if target_distance <= distance:
                    actual_x = curr_x
                    actual_y = curr_y
                else:
                    # 计算单位方向向量
                    unit_dx = dx / target_distance
                    unit_dy = dy / target_distance
                    
                    # 计算实际目标位置（距离目标点distance距离的位置）
                    actual_x = x - unit_dx * distance
                    actual_y = y - unit_dy * distance
                
                # 检查实际目标位置是否在地图边界内
                if not (self.ros_node.map_bound["xmin"] <= actual_x <= self.ros_node.map_bound["xmax"] and
                        self.ros_node.map_bound["ymin"] <= actual_y <= self.ros_node.map_bound["ymax"]):
                    return {"success": False, "msg": f"计算的实际目标位置({actual_x:.2f}, {actual_y:.2f})超出地图边界"}
                
                # 计算面向目标点的朝向（从实际位置指向目标点）
                face_dx = x - actual_x
                face_dy = y - actual_y
                yaw = np.arctan2(face_dy, face_dx)
                
                # 计算四元数
                qz = np.sin(yaw / 2)
                qw = np.cos(yaw / 2)

                # 创建PoseStamped消息
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.pose.position.x = actual_x
                pose.pose.position.y = actual_y
                pose.pose.position.z = 0.0  # 假设z=0
                
                # 设置朝向为面向目标点
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                target_str = f"lonlat({lon},{lat})_dist_{distance}"
                # 发布目标点
                self.ros_node.publish_goal(pose, target_str)

                return {
                    "success": True,
                    "target_str": target_str,
                    "msg": f"已下发目标，将在距离经纬度({lon}, {lat})约{distance}米处停下并面向目标点",
                    "original_target": {"lon": lon, "lat": lat, "x": x, "y": y},
                    "actual_target": {"x": actual_x, "y": actual_y},
                    "distance": distance
                }
            except Exception as e:
                return {"success": False, "msg": f"经纬度导航失败: {str(e)}"}

        @self.mcp.tool()
        def set_goal_by_xy(x: float, y: float, distance: float = 1.5) -> Dict[str, Any]:
            """
            根据输入的地图坐标导航到目标点附近
            步骤：
            1. 判断是否在map bound范围内
            2. 计算距离目标点distance距离的实际目标位置
            3. 调用publish_goal发起导航

            参数:
            x: 目标点X坐标
            y: 目标点Y坐标
            distance: 距离目标点的距离，默认1.5米

            返回:
            Dict: 操作结果
            """
            try:
                # 检查输入坐标是否在地图边界内
                if not (self.ros_node.map_bound["xmin"] <= x <= self.ros_node.map_bound["xmax"] and
                        self.ros_node.map_bound["ymin"] <= y <= self.ros_node.map_bound["ymax"]):
                    return {"success": False, "msg": f"目标点({x}, {y})超出地图边界"}
                
                # 获取当前位置
                odom = self.ros_node.current_pose
                if not odom:
                    return {"success": False, "msg": "无法获取当前位置信息"}
                
                curr_x = odom.pose.position.x
                curr_y = odom.pose.position.y
                
                # 计算从当前位置到目标点的方向向量
                dx = x - curr_x
                dy = y - curr_y
                target_distance = (dx**2 + dy**2) ** 0.5
                
                # 如果当前距离目标点小于指定距离，则直接面向目标点
                if target_distance <= distance:
                    actual_x = curr_x
                    actual_y = curr_y
                else:
                    # 计算单位方向向量
                    unit_dx = dx / target_distance
                    unit_dy = dy / target_distance
                    
                    # 计算实际目标位置（距离目标点distance距离的位置）
                    actual_x = x - unit_dx * distance
                    actual_y = y - unit_dy * distance
                
                # 检查实际目标位置是否在地图边界内
                if not (self.ros_node.map_bound["xmin"] <= actual_x <= self.ros_node.map_bound["xmax"] and
                        self.ros_node.map_bound["ymin"] <= actual_y <= self.ros_node.map_bound["ymax"]):
                    return {"success": False, "msg": f"计算的实际目标位置({actual_x:.2f}, {actual_y:.2f})超出地图边界"}
                
                # 计算面向目标点的朝向（从实际位置指向目标点）
                face_dx = x - actual_x
                face_dy = y - actual_y
                yaw = np.arctan2(face_dy, face_dx)
                
                # 计算四元数
                qz = np.sin(yaw / 2)
                qw = np.cos(yaw / 2)

                # 创建PoseStamped消息
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.pose.position.x = actual_x
                pose.pose.position.y = actual_y
                pose.pose.position.z = 0.0  # 假设z=0
                
                # 设置朝向为面向目标点
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                target_str = f"xy({x},{y})_dist_{distance}"
                # 发布目标点
                self.ros_node.publish_goal(pose, target_str)

                return {
                    "success": True,
                    "target_str": target_str,
                    "msg": f"已下发目标，将在距离({x}, {y})约{distance}米处停下并面向目标点",
                    "original_target": {"x": x, "y": y},
                    "actual_target": {"x": actual_x, "y": actual_y},
                    "distance": distance
                }
            except Exception as e:
                return {"success": False, "msg": f"地图坐标导航失败: {str(e)}"}

        @self.mcp.tool()
        def pause_navigation() -> Dict[str, Any]:
            """
            暂停导航（取消当前目标）
            """
            try:
                self.ros_node.pause_navigation()
                return {"success": True, "msg": "导航已暂停（已取消当前目标）"}
            except Exception as e:
                return {"success": False, "msg": f"暂停导航失败: {str(e)}"}

        @self.mcp.tool()
        def cancel_navigation() -> Dict[str, Any]:
            """
            取消导航
            """
            try:
                self.ros_node.cancel_navigation()
                return {"success": True, "msg": "导航已取消"}
            except Exception as e:
                return {"success": False, "msg": f"取消导航失败: {str(e)}"}

        @self.mcp.tool()
        def resume_navigation() -> Dict[str, Any]:
            """
            继续导航（重新发送上一个目标）
            """
            try:
                self.ros_node.resume_navigation()
                return {"success": True, "msg": "导航已继续"}
            except Exception as e:
                return {"success": False, "msg": f"继续导航失败: {str(e)}"}

        @self.mcp.tool()
        def get_pose_list() -> Dict[str, Any]:
            """
            获取所有预设点名称

            返回:
            Dict: {"success": True, "keys": [...], "count": N}
            """
            try:
                keys = list(self.map_manager.pose_map_current_submap.keys())
                return {"success": True, "keys": keys, "count": len(keys)}
            except Exception as e:
                return {"success": False, "msg": f"获取pose列表失败: {str(e)}"}

        @self.mcp.tool()
        def get_current_map_id() -> Dict[str, Any]:
            """
            获取当前加载的地图信息

            返回:
            Dict: {"success": True, "map_id": "...", "map_path": "..."}
            """
            try:
                # 通过 ROS 参数服务获取当前地图路径
                map_path = self.ros_node.get_current_map_path()
                if not map_path:
                    return {"success": False, "msg": "无法获取当前地图路径"}
                
                # 通过 MapManager 获取 submap ID
                map_id = self.map_manager.get_submap_id_by_map_file(map_path)
                
                return {
                    "success": True,
                    "map_id": map_id,
                    "map_path": map_path,
                    "msg": f"当前地图: {map_id or '未知'} ({map_path})"
                }
            except Exception as e:
                return {"success": False, "msg": f"获取当前地图失败: {str(e)}"}

        @self.mcp.tool()
        def change_map(map_id: str) -> Dict[str, Any]:
            """
            切换到指定地图

            参数:
            map_id: 目标地图 ID（如 floor, underground, 5floor）

            返回:
            Dict: {"success": True, "map_id": "...", "map_path": "...", "msg": "..."}
            """
            try:
                # 检查 map_id 是否存在
                submap_list = self.map_manager.get_submap_list()
                if map_id not in submap_list:
                    return {
                        "success": False,
                        "msg": f"地图 {map_id} 不存在，可用地图: {submap_list}"
                    }
                
                # 获取当前地图 ID
                current_map_path = self.ros_node.get_current_map_path()
                if current_map_path:
                    current_map_id = self.map_manager.get_submap_id_by_map_file(current_map_path)
                    if current_map_id == map_id:
                        return {
                            "success": True,
                            "map_id": map_id,
                            "map_path": current_map_path,
                            "msg": f"当前已经是地图 {map_id}，无需切换"
                        }
                
                # 获取目标地图的完整路径
                target_map_path = self.map_manager.get_submap_full_map_path(map_id)
                if not target_map_path:
                    return {"success": False, "msg": f"无法获取地图 {map_id} 的路径"}
                
                # 通过 NavNode 切换地图
                success = self.ros_node.change_map(target_map_path)
                if not success:
                    return {"success": False, "msg": f"切换地图失败: {target_map_path}"}
                
                # 更新 MapManager 的当前 submap
                self.map_manager.set_current_submap(map_id)
                
                return {
                    "success": True,
                    "map_id": map_id,
                    "map_path": target_map_path,
                    "msg": f"已切换到地图 {map_id}"
                }
            except Exception as e:
                return {"success": False, "msg": f"切换地图失败: {str(e)}"}

        @self.mcp.tool()
        def get_route_names() -> Dict[str, Any]:
            """
            获取所有巡逻线路名称

            返回:
            Dict: {"success": True, "route_names": [...], "count": N}
            """
            try:
                route_list = self.map_manager.get_route_list()
                return {
                    "success": True,
                    "route_names": [r['name'] for r in route_list],
                    "routes": route_list,
                    "count": len(route_list)
                }
            except Exception as e:
                return {"success": False, "msg": f"获取路线列表失败: {str(e)}"}

        @self.mcp.tool()
        def get_route(route_name: str) -> Dict[str, Any]:
            """
            根据路线名称获取巡逻点位列表。

            Args:
                route_name: 路线名称（如 线路A）

            Returns:
            Dict: 包含路线描述和巡逻点位列表
            """
            try:
                route_info = self.map_manager.get_route_by_name(route_name)
                if not route_info:
                    return {"success": False, "msg": f"路线不存在: {route_name}"}

                return {
                    "success": True,
                    "name": route_info['name'],
                    "description": route_info['description'],
                    "waypoints": route_info['waypoints'],
                    "count": len(route_info['waypoints'])
                }
            except Exception as e:
                return {"success": False, "msg": f"获取路线失败: {str(e)}"}
        
                
        @self.mcp.tool()
        def get_status() -> Dict[str, Any]:
            """
            获取小车实时状态
            
            返回:
            Dict: 包含位置、姿态、线速度、角速度的状态信息
            """
            """获取小车实时状态"""
            try:
                # 获取当前里程计数据
                odom = self.ros_node.current_pose
                # print(f"get_status: return odom:{odom.pose.position, odom.pose.orientation}")
                if not odom:
                    return {"success": False, "msg": "无实时位姿数据"}
                
                # 构造状态信息字典
                return {
                    "success": True,
                    "position": {
                        "x": odom.pose.position.x,
                        "y": odom.pose.position.y,
                        "z": odom.pose.position.z
                    },
                    "orientation": {
                        "x": odom.pose.orientation.x,
                        "y": odom.pose.orientation.y,
                        "z": odom.pose.orientation.z,
                        "w": odom.pose.orientation.w
                    }
                }
            except Exception as e:
                return {"success": False, "msg": f"获取状态失败: {str(e)}"}

        @self.mcp.tool()
        async def wait_goal_reached(target: str, timeout: float = 120.0, xy_tolerance: float = 0.1, yaw_tolerance: float = 0.1) -> Dict[str, Any]:
            """
            阻塞直到到达目标点或超时
            
            参数:
            target: 目标点名称
            timeout: 超时时间（秒）
            
            返回:
            Dict: 包含到达结果的字典
            """
            """阻塞直到到达目标点或超时"""
            try:
                # 记录开始时间
                start_time = time.time()
                # 循环检查是否到达目标或超时
                while time.time() - start_time < timeout:
                    # 异步等待0.2秒再次检查
                    await asyncio.sleep(0.2)
                    # 检查是否到达目标点
                    if self.ros_node.check_goal_reached(xy_tolerance, yaw_tolerance):
                        return {"success": True, "msg": f"已到达目标 {target}", "timestamp": int(time.time())}
                # 超时未到达
                return {"success": False, "msg": "等待超时或未到达"}
            except Exception as e:
                return {"success": False, "msg": f"等待目标失败: {str(e)}"}

        @self.mcp.tool()
        def get_nearest_point() -> Dict[str, Any]:
            """
            获取当前位置最接近的预设点
            
            返回:
            Dict: 包含最近点信息和所有距离的字典
            """
            """获取当前位置最接近的预设点"""
            try:
                # 获取当前里程计数据
                odom = self.ros_node.current_pose
                if not odom:
                    return {"success": False, "msg": "无实时位姿数据"}
                
                # 获取当前位置坐标
                current_x = odom.pose.position.x
                current_y = odom.pose.position.y

                # 初始化最近点查找变量
                nearest_point = None
                min_distance = float('inf')  # 初始化为无穷大
                distances = {}  # 存储所有点到当前位置的距离
                
                # 遍历所有预设点计算距离
                for point_name, point_data in self.map_manager.pose_map_current_submap.items():
                    # 获取预设点的位姿信息
                    pose_info = point_data.get("pose")
                    if not pose_info:
                        continue
                    
                    # 获取预设点坐标
                    target_x = pose_info["position"]["x"]
                    target_y = pose_info["position"]["y"]
                    
                    # 计算欧几里得距离
                    distance = ((current_x - target_x) ** 2 + (current_y - target_y) ** 2) ** 0.5
                    # 记录距离
                    distances[point_name] = distance
                    
                    # 更新最近点
                    if distance < min_distance:
                        min_distance = distance
                        nearest_point = point_name
                
                # 检查是否找到有效点
                if nearest_point is None:
                    return {"success": False, "msg": "没有找到有效的预设点"}
                
                # 返回结果
                return {
                    "success": True,
                    "nearest_point": nearest_point,  # 最近点名称
                    "distance": round(min_distance, 3),  # 最近距离（保留3位小数）
                    "current_position": {  # 当前位置
                        "x": round(current_x, 3),
                        "y": round(current_y, 3)
                    },
                    "all_distances": {k: round(v, 3) for k, v in distances.items()},  # 所有距离
                    "msg": f"当前位置最接近的预设点是: {nearest_point} (距离: {min_distance:.3f}m)"
                }
            except Exception as e:
                return {"success": False, "msg": f"获取最近点失败: {str(e)}"}

        @self.mcp.tool()
        def get_map() -> Dict[str, Any]:
            """
            获取当前栅格地图
            
            返回:
            Dict: 包含地图信息的字典，包括:
                - info: 地图元信息（分辨率、尺寸、原点）
                - data: 栅格数据（-1=未知, 0=空闲, 100=占用）
                - bound: 地图边界
            """
            try:
                map_msg = self.ros_node.current_map
                if map_msg is None:
                    return {"success": False, "msg": "尚未接收到地图数据，请等待/map话题发布"}
                
                # 获取地图元信息
                info = map_msg.info
                
                # 将地图数据转换为列表（原始数据是int8数组）
                # 注意：地图数据可能很大，按需返回
                map_data = list(map_msg.data)
                logging.info(f"获取地图数据: size={len(map_data)}, width={info.width}, height={info.height}, resolution={info.resolution}")
                return {
                    "success": True,
                    "info": {
                        "resolution": info.resolution,  # 米/像素
                        "width": info.width,            # 像素
                        "height": info.height,          # 像素
                        "origin": {
                            "x": info.origin.position.x,
                            "y": info.origin.position.y,
                            "z": info.origin.position.z
                        }
                    },
                    "bound": self.ros_node.map_bound,
                    "data": map_data,  # 栅格数据: -1=未知, 0=空闲, 1-100=占用概率
                    "msg": f"地图大小: {info.width}x{info.height}, 分辨率: {info.resolution}m/pixel"
                }
            except Exception as e:
                return {"success": False, "msg": f"获取地图失败: {str(e)}"}

    async def start(self, port=8000):
        """
        启动MCP服务
        
        功能：
        1. 以异步方式启动MCP HTTP服务
        2. 监听指定主机和端口
        """
        print(f"启动MCP服务，监听端口: {port}")
        try:
            # 启动MCP异步HTTP服务
            await self.mcp.run_async(
                transport="streamable-http",  # 使用流式HTTP传输
                host="0.0.0.0",  # 监听所有网络接口
                port=port,  # 监听端口8000
                log_level="warning"  # 设置日志级别为WARNING
            )
        except Exception as e:
            logging.error(f"MCP服务启动失败: {e}")
            raise

    def cleanup(self):
        """
        清理资源
        
        功能：
        1. 关闭ROS执行器
        2. 销毁ROS节点
        3. 关闭ROS客户端库
        """
        """清理资源"""
        try:
            # 关闭执行器
            if hasattr(self, 'executor'):
                self.executor.shutdown()
            # 销毁ROS节点
            if hasattr(self, 'ros_node'):
                self.ros_node.destroy_node()
            # 关闭ROS客户端库
            rclpy.shutdown()
        except Exception as e:
            logging.error(f"清理资源失败: {e}")

# 异步主函数
async def main():
    """
    异步主函数
    """

    # 初始化ROS2客户端库
    rclpy.init()
    # 解析命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test', action='store_true', help='测试环境')
    parser.add_argument('-m', '--map', type=str, default=None,
                        help='多楼层地图配置文件路径 (multifloor.json)')
    args = parser.parse_args()

    port = 9999 if args.test else 9000

    # 初始化地图管理器
    map_manager = None
    if args.map:
        try:
            map_manager = MapManager(args.map)
            logging.info(f"已加载多楼层地图: {args.map}")
        except Exception as e:
            logging.error(f"加载地图配置失败: {e}")

    mgr = None
    try:
        # 创建导航工具管理器
        mgr = NavigationToolManager(map_manager=map_manager)
        # 启动MCP服务（阻塞运行）
        await mgr.start(port=port)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断信号
        logging.info("收到中断信号，正在关闭...")
    except Exception as e:
        # 处理其他异常
        logging.error(f"程序运行出错: {e}")
    finally:
        # 确保资源被正确清理
        if mgr:
            mgr.cleanup()

# 程序入口点
if __name__ == "__main__":
    # 设置日志级别为INFO
    # 设置日志
    logging.basicConfig(level=logging.WARNING)
    # 关闭 httpx 的请求日志
    logging.getLogger("httpx").setLevel(logging.WARNING)
    
    # 运行异步主函数
    # 使用asyncio运行异步主函数
    asyncio.run(main())
    
