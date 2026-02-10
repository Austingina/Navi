#!/usr/bin/env python3
"""
带UI的MCP客户端测试脚本
功能：
1. 显示栅格地图
2. 实时显示机器人位置
3. 路线巡检
4. API导航测试
"""

import asyncio
import json
import time
import logging
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import numpy as np
from typing import Dict, List, Optional
from fastmcp import Client
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("mcp.client").setLevel(logging.WARNING)
logging.getLogger("mcp.client.session").setLevel(logging.WARNING)
logging.getLogger("fastmcp").setLevel(logging.WARNING)


class MCPClientUI:
    def __init__(self, server_url: str = "http://localhost:9999/mcp"):
        self.server_url = server_url
        self.client = None
        
        # 地图数据
        self.map_data = None
        self.map_info = None
        self.map_bound = None
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # 点位列表
        self.pose_list = []
        
        # 路线数据
        self.route_waypoints = []
        self.current_waypoint_idx = 0
        self.is_patrolling = False
        
        # 导航模式
        self.xy_select_mode = False  # 是否在地图上选点模式
        
        # 异步事件循环
        self.loop = None
        self.async_thread = None
        
        # 创建UI
        self.create_ui()
        
        # 启动异步线程
        self.start_async_loop()
        
        # 初始化数据
        self.schedule_async(self.init_data())
        
        # 启动定时刷新
        self.start_status_update()

    def create_ui(self):
        """创建UI界面"""
        self.root = tk.Tk()
        self.root.title("MCP Navigation Test Client")
        self.root.geometry("1200x800")
        
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左侧控制面板
        left_frame = ttk.Frame(main_frame, width=300)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_frame.pack_propagate(False)
        
        # 右侧地图区域
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # === 左上：路线巡检区域 ===
        patrol_frame = ttk.LabelFrame(left_frame, text="路线巡检", padding=10)
        patrol_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 路线选择
        route_select_frame = ttk.Frame(patrol_frame)
        route_select_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(route_select_frame, text="路线:").pack(side=tk.LEFT)
        self.route_var = tk.StringVar(value="线路A")
        self.route_combo = ttk.Combobox(route_select_frame, textvariable=self.route_var, 
                                         values=["线路A"], width=15)
        self.route_combo.pack(side=tk.LEFT, padx=(5, 0))
        
        # 开始/停止按钮
        btn_frame = ttk.Frame(patrol_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        self.patrol_btn = ttk.Button(btn_frame, text="开始巡检", command=self.toggle_patrol)
        self.patrol_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.stop_patrol_btn = ttk.Button(btn_frame, text="停止", command=self.stop_patrol, state=tk.DISABLED)
        self.stop_patrol_btn.pack(side=tk.LEFT)
        
        # Waypoint显示区域
        ttk.Label(patrol_frame, text="巡检点位:").pack(anchor=tk.W)
        self.waypoint_text = scrolledtext.ScrolledText(patrol_frame, height=8, width=30, state=tk.DISABLED)
        self.waypoint_text.pack(fill=tk.X, pady=(5, 0))
        
        # 巡检状态
        self.patrol_status_var = tk.StringVar(value="状态: 空闲")
        ttk.Label(patrol_frame, textvariable=self.patrol_status_var).pack(anchor=tk.W, pady=(5, 0))
        
        # === 左下：API导航区域 ===
        nav_frame = ttk.LabelFrame(left_frame, text="API导航", padding=10)
        nav_frame.pack(fill=tk.BOTH, expand=True)
        
        # set_goal 区域
        goal_frame = ttk.Frame(nav_frame)
        goal_frame.pack(fill=tk.X, pady=(0, 10))
        self.set_goal_btn = ttk.Button(goal_frame, text="set_goal", command=self.on_set_goal, width=12)
        self.set_goal_btn.pack(side=tk.LEFT)
        self.goal_var = tk.StringVar()
        self.goal_combo = ttk.Combobox(goal_frame, textvariable=self.goal_var, width=15)
        self.goal_combo.pack(side=tk.LEFT, padx=(5, 0))
        
        # set_goal_by_xy 区域
        xy_frame = ttk.Frame(nav_frame)
        xy_frame.pack(fill=tk.X, pady=(0, 10))
        self.set_goal_xy_btn = ttk.Button(xy_frame, text="set_goal_by_xy", command=self.on_set_goal_xy, width=15)
        self.set_goal_xy_btn.pack(side=tk.LEFT)
        self.xy_status_var = tk.StringVar(value="点击按钮后在地图选点")
        ttk.Label(xy_frame, textvariable=self.xy_status_var, foreground="gray").pack(side=tk.LEFT, padx=(5, 0))
        
        # set_goal_by_lonlat 区域 (暂不可用)
        lonlat_frame = ttk.Frame(nav_frame)
        lonlat_frame.pack(fill=tk.X, pady=(0, 10))
        self.set_goal_lonlat_btn = ttk.Button(lonlat_frame, text="set_goal_by_lonlat", 
                                               command=self.on_set_goal_lonlat, width=18, state=tk.DISABLED)
        self.set_goal_lonlat_btn.pack(side=tk.LEFT)
        ttk.Label(lonlat_frame, text="(暂不可用)", foreground="gray").pack(side=tk.LEFT, padx=(5, 0))
        
        # 导航状态
        ttk.Separator(nav_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        self.nav_status_var = tk.StringVar(value="导航状态: 空闲")
        ttk.Label(nav_frame, textvariable=self.nav_status_var).pack(anchor=tk.W)
        
        # 机器人位置
        self.robot_pos_var = tk.StringVar(value="位置: (-, -)")
        ttk.Label(nav_frame, textvariable=self.robot_pos_var).pack(anchor=tk.W, pady=(5, 0))
        
        # === 右侧：地图显示区域 ===
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Map View")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 绑定鼠标点击事件
        self.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        # 机器人标记
        self.robot_marker = None
        self.robot_arrow = None
        self.target_marker = None

    def start_async_loop(self):
        """启动异步事件循环线程"""
        loop_ready = threading.Event()
        
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            loop_ready.set()  # 通知主线程 loop 已准备好
            self.loop.run_forever()
        
        self.async_thread = threading.Thread(target=run_loop, daemon=True)
        self.async_thread.start()
        loop_ready.wait()  # 等待 loop 创建完成

    def schedule_async(self, coro):
        """在异步线程中调度协程"""
        if self.loop:
            asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def init_data(self):
        """初始化数据：获取地图和点位列表"""
        try:
            async with Client(self.server_url) as client:
                # 获取地图
                logger.info("获取地图数据...")
                map_result = await client.call_tool("get_map")
                map_data = json.loads(map_result.content[0].text)
                if map_data.get("success"):
                    self.map_info = map_data["info"]
                    self.map_bound = map_data["bound"]
                    self.map_data = np.array(map_data["data"]).reshape(
                        self.map_info["height"], self.map_info["width"]
                    )
                    logger.info(f"地图加载成功: {self.map_info['width']}x{self.map_info['height']}")
                    self.root.after(0, self.update_map_display)
                else:
                    logger.error(f"获取地图失败: {map_data.get('msg')}")
                
                # 获取点位列表
                logger.info("获取点位列表...")
                pose_result = await client.call_tool("get_pose_list")
                pose_data = json.loads(pose_result.content[0].text)
                if pose_data.get("success"):
                    self.pose_list = pose_data["keys"]
                    logger.info(f"点位列表: {self.pose_list}")
                    self.root.after(0, lambda: self.goal_combo.configure(values=self.pose_list))
                    if self.pose_list:
                        self.root.after(0, lambda: self.goal_var.set(self.pose_list[0]))
                else:
                    logger.error(f"获取点位列表失败: {pose_data.get('msg')}")
                    
        except Exception as e:
            logger.error(f"初始化数据失败: {e}")

    def update_map_display(self):
        """更新地图显示"""
        if self.map_data is None or self.map_info is None:
            return
        
        self.ax.clear()
        
        # 计算地图范围
        origin_x = self.map_info["origin"]["x"]
        origin_y = self.map_info["origin"]["y"]
        resolution = self.map_info["resolution"]
        width = self.map_info["width"]
        height = self.map_info["height"]
        
        extent = [
            origin_x,
            origin_x + width * resolution,
            origin_y,
            origin_y + height * resolution
        ]
        
        # 显示地图 (-1=未知灰色, 0=空闲白色, 100=障碍物黑色)
        display_map = np.copy(self.map_data).astype(float)
        display_map[display_map == -1] = 50  # 未知区域显示为灰色
        
        self.ax.imshow(display_map, cmap='gray_r', origin='lower', extent=extent, 
                       vmin=0, vmax=100, alpha=0.8)
        
        self.ax.set_title("Map View")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # 重新绘制机器人位置
        self.draw_robot()
        
        self.canvas.draw()

    def draw_robot(self):
        """绘制机器人位置"""
        if self.map_data is None:
            return
            
        # 移除旧标记
        if self.robot_marker:
            self.robot_marker.remove()
            self.robot_marker = None
        if self.robot_arrow:
            self.robot_arrow.remove()
            self.robot_arrow = None
        
        # 绘制机器人位置（红色圆点）
        self.robot_marker, = self.ax.plot(self.robot_x, self.robot_y, 'ro', markersize=10, label='Robot')
        
        # 绘制朝向箭头
        arrow_len = 0.5
        dx = arrow_len * np.cos(self.robot_yaw)
        dy = arrow_len * np.sin(self.robot_yaw)
        self.robot_arrow = self.ax.arrow(self.robot_x, self.robot_y, dx, dy,
                                          head_width=0.2, head_length=0.1, fc='red', ec='red')
        
        self.canvas.draw_idle()

    def start_status_update(self):
        """启动状态更新定时器 (1Hz)"""
        self.update_robot_status()
        self.root.after(1000, self.start_status_update)

    def update_robot_status(self):
        """更新机器人状态"""
        self.schedule_async(self._async_update_status())

    async def _async_update_status(self):
        """异步获取机器人状态"""
        try:
            async with Client(self.server_url) as client:
                result = await client.call_tool("get_status")
                data = json.loads(result.content[0].text)
                if data.get("success"):
                    pos = data["position"]
                    ori = data["orientation"]
                    self.robot_x = pos["x"]
                    self.robot_y = pos["y"]
                    # 从四元数计算yaw
                    self.robot_yaw = np.arctan2(
                        2.0 * (ori["w"] * ori["z"] + ori["x"] * ori["y"]),
                        1.0 - 2.0 * (ori["y"]**2 + ori["z"]**2)
                    )
                    # 更新UI
                    self.root.after(0, lambda: self.robot_pos_var.set(
                        f"位置: ({self.robot_x:.2f}, {self.robot_y:.2f})"
                    ))
                    self.root.after(0, self.draw_robot)
        except Exception as e:
            logger.debug(f"获取状态失败: {e}")

    # === 路线巡检功能 ===
    def toggle_patrol(self):
        """开始/暂停巡检"""
        if not self.is_patrolling:
            self.start_patrol()
        else:
            self.pause_patrol()

    def start_patrol(self):
        """开始巡检"""
        self.is_patrolling = True
        self.patrol_btn.configure(text="暂停")
        self.stop_patrol_btn.configure(state=tk.NORMAL)
        self.schedule_async(self._async_patrol())

    def pause_patrol(self):
        """暂停巡检"""
        self.is_patrolling = False
        self.patrol_btn.configure(text="继续")
        self.schedule_async(self._async_pause_navigation())

    async def _async_pause_navigation(self):
        """异步暂停导航"""
        try:
            async with Client(self.server_url) as client:
                await client.call_tool("pause_navigation")
                self.root.after(0, lambda: self.patrol_status_var.set("状态: 已暂停"))
        except Exception as e:
            logger.error(f"暂停导航失败: {e}")

    def stop_patrol(self):
        """停止巡检"""
        self.is_patrolling = False
        self.current_waypoint_idx = 0
        self.route_waypoints = []
        self.patrol_btn.configure(text="开始巡检")
        self.stop_patrol_btn.configure(state=tk.DISABLED)
        self.schedule_async(self._async_cancel_navigation())
        self.root.after(0, lambda: self.patrol_status_var.set("状态: 已停止"))

    async def _async_cancel_navigation(self):
        """异步取消导航"""
        try:
            async with Client(self.server_url) as client:
                await client.call_tool("cancel_navigation")
        except Exception as e:
            logger.error(f"取消导航失败: {e}")

    async def _async_patrol(self):
        """异步执行巡检"""
        try:
            async with Client(self.server_url) as client:
                # 获取路线
                route_name = self.route_var.get()
                self.root.after(0, lambda: self.patrol_status_var.set(f"状态: 获取路线 {route_name}..."))
                
                route_result = await client.call_tool("get_route", {"route_name": route_name})
                route_data = json.loads(route_result.content[0].text)
                
                if not route_data.get("success"):
                    self.root.after(0, lambda: messagebox.showerror("错误", f"获取路线失败: {route_data.get('msg')}"))
                    self.root.after(0, self.stop_patrol)
                    return
                
                self.route_waypoints = route_data["points"]
                self.current_waypoint_idx = 0
                
                # 更新waypoint显示
                waypoint_text = "\n".join([f"{i+1}. {wp}" for i, wp in enumerate(self.route_waypoints)])
                self.root.after(0, lambda: self._update_waypoint_text(waypoint_text))
                
                logger.info(f"开始巡检路线: {route_name}, 共 {len(self.route_waypoints)} 个点位")
                
                # 巡检统计
                success_count = 0
                total_time = 0
                start_total = time.time()
                
                # 依次访问每个waypoint
                while self.current_waypoint_idx < len(self.route_waypoints) and self.is_patrolling:
                    waypoint = self.route_waypoints[self.current_waypoint_idx]
                    idx = self.current_waypoint_idx + 1
                    total = len(self.route_waypoints)
                    
                    self.root.after(0, lambda w=waypoint, i=idx, t=total: 
                                    self.patrol_status_var.set(f"状态: 前往 {w} ({i}/{t})"))
                    
                    logger.info(f"巡检点 {idx}/{total}: {waypoint}")
                    
                    # 发送目标
                    start_time = time.time()
                    goal_result = await client.call_tool("set_goal", {"target": waypoint})
                    goal_data = json.loads(goal_result.content[0].text)
                    
                    if not goal_data.get("success"):
                        logger.error(f"发送目标失败: {goal_data.get('msg')}")
                        self.current_waypoint_idx += 1
                        continue
                    
                    # 等待到达
                    while self.is_patrolling:
                        await asyncio.sleep(1.0)
                        wait_result = await client.call_tool("wait_goal_reached", {
                            "target": waypoint,
                            "timeout": 1.0,
                            "xy_tolerance": 0.6,
                            "yaw_tolerance": 3.14
                        })
                        wait_data = json.loads(wait_result.content[0].text)
                        
                        if wait_data.get("success"):
                            duration = time.time() - start_time
                            success_count += 1
                            total_time += duration
                            logger.info(f"✓ 到达 {waypoint}, 耗时 {duration:.1f}秒")
                            break
                    
                    self.current_waypoint_idx += 1
                    
                    # 点位间等待
                    if self.is_patrolling and self.current_waypoint_idx < len(self.route_waypoints):
                        await asyncio.sleep(2.0)
                
                # 巡检完成
                total_duration = time.time() - start_total
                self.root.after(0, self.stop_patrol)
                
                # 输出巡检报告
                report = f"""
{'='*50}
巡检报告
{'='*50}
路线: {route_name}
总点位: {len(self.route_waypoints)}
成功: {success_count}
失败: {len(self.route_waypoints) - success_count}
成功率: {success_count/len(self.route_waypoints)*100:.1f}%
总耗时: {total_duration:.1f}秒
平均每点: {total_time/max(success_count,1):.1f}秒
{'='*50}
"""
                logger.info(report)
                print(report)
                
        except Exception as e:
            logger.error(f"巡检执行失败: {e}")
            self.root.after(0, self.stop_patrol)

    def _update_waypoint_text(self, text):
        """更新waypoint文本显示"""
        self.waypoint_text.configure(state=tk.NORMAL)
        self.waypoint_text.delete(1.0, tk.END)
        self.waypoint_text.insert(tk.END, text)
        self.waypoint_text.configure(state=tk.DISABLED)

    # === API导航功能 ===
    def on_set_goal(self):
        """点击set_goal按钮"""
        target = self.goal_var.get()
        if not target:
            messagebox.showwarning("警告", "请选择目标点")
            return
        self.schedule_async(self._async_set_goal(target))

    async def _async_set_goal(self, target: str):
        """异步发送set_goal"""
        try:
            self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 前往 {target}..."))
            async with Client(self.server_url) as client:
                result = await client.call_tool("set_goal", {"target": target})
                data = json.loads(result.content[0].text)
                if data.get("success"):
                    logger.info(f"set_goal成功: {target}")
                    self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 正在前往 {target}"))
                else:
                    logger.error(f"set_goal失败: {data.get('msg')}")
                    self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 失败 - {data.get('msg')}"))
        except Exception as e:
            logger.error(f"set_goal异常: {e}")
            self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 异常"))

    def on_set_goal_xy(self):
        """点击set_goal_by_xy按钮 - 进入地图选点模式"""
        self.xy_select_mode = True
        self.xy_status_var.set("请在地图上点击选择目标点...")
        self.set_goal_xy_btn.configure(text="取消选点")
        logger.info("进入地图选点模式")

    def on_map_click(self, event):
        """地图点击事件"""
        if not self.xy_select_mode:
            return
        
        if event.xdata is None or event.ydata is None:
            return
        
        x, y = event.xdata, event.ydata
        logger.info(f"选择点位: ({x:.2f}, {y:.2f})")
        
        # 退出选点模式
        self.xy_select_mode = False
        self.xy_status_var.set("点击按钮后在地图选点")
        self.set_goal_xy_btn.configure(text="set_goal_by_xy")
        
        # 显示目标标记
        if self.target_marker:
            self.target_marker.remove()
        self.target_marker, = self.ax.plot(x, y, 'g*', markersize=15, label='Target')
        self.canvas.draw()
        
        # 发送导航请求
        self.schedule_async(self._async_set_goal_by_xy(x, y))

    async def _async_set_goal_by_xy(self, x: float, y: float):
        """异步发送set_goal_by_xy"""
        try:
            self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 前往 ({x:.2f}, {y:.2f})..."))
            async with Client(self.server_url) as client:
                result = await client.call_tool("set_goal_by_xy", {"x": x, "y": y, "distance": 0.5})
                data = json.loads(result.content[0].text)
                if data.get("success"):
                    logger.info(f"set_goal_by_xy成功: ({x:.2f}, {y:.2f})")
                    self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 正在前往 ({x:.2f}, {y:.2f})"))
                else:
                    logger.error(f"set_goal_by_xy失败: {data.get('msg')}")
                    self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 失败 - {data.get('msg')}"))
        except Exception as e:
            logger.error(f"set_goal_by_xy异常: {e}")
            self.root.after(0, lambda: self.nav_status_var.set(f"导航状态: 异常"))

    def on_set_goal_lonlat(self):
        """点击set_goal_by_lonlat按钮 (暂不可用)"""
        messagebox.showinfo("提示", "set_goal_by_lonlat 功能暂不可用")

    def run(self):
        """运行UI主循环"""
        self.root.mainloop()


def main():
    """主函数"""
    import argparse
    parser = argparse.ArgumentParser(description="MCP Navigation Test Client with UI")
    parser.add_argument("--url", default="http://localhost:9999/mcp", help="MCP server URL")
    args = parser.parse_args()
    
    app = MCPClientUI(server_url=args.url)
    app.run()


if __name__ == "__main__":
    main()
