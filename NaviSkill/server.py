# 导入必要的库
import time  # 时间处理库
from typing import Dict, Any, Optional  # 类型提示
import inspect
import asyncio  # 异步IO库
import logging  # 日志记录库
import json  # JSON数据处理库
import os  # 操作系统接口库
import sys
import threading  # 线程处理库
from pathlib import Path
import numpy as np  # 数值计算库
# 让仓库根 (/home/nvidia/Navi) 进入 sys.path，方便 import mapserver
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
# 导入自定义模块
from wgs84 import WGS84Converter 
from nav_node import NavNode  # 导入导航节点类
from map_manager import MapManager  # 导入多楼层地图管理器
from vpr_bootstrap import VprBootstrap, VprRelocator  # VPR 子进程 / 共识重定位
from mapserver.client import resolve_map_source  # -m URL / 本地路径双模式解析
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
# 导入ROS相关的库
import rclpy  # ROS2 Python客户端库
from rclpy.executors import MultiThreadedExecutor  # 多线程执行器（配合 NavNode 回调组并行调度）
from geometry_msgs.msg import PoseStamped  # 位姿消息类型
import argparse  # 添加在文件顶部
# 加载地图配置文件的函数
# 直接在此定义 load_intent_pose_map，兼容 intent_pose_map.json 的多种结构


# 导航工具管理器类
class NavigationToolManager:
    """
    导航工具管理器

    功能：
    1. 初始化 FastAPI WebSocket 与 ROS 节点
    2. 注册导航相关 RPC 方法（经 /ws JSON 调用）
    3. 管理 ROS 线程和资源清理
    """
    def __init__(
        self,
        map_manager: MapManager = None,
        *,
        submap_specified: bool = False,
        vpr_device: str = "cuda",
        camera_image_topic: Optional[str] = None,
    ):
        """
        初始化导航工具管理器

        :param map_manager: 多楼层地图管理器实例，可选
        :param submap_specified: 用户是否在命令行显式指定了初始 submap；为 False
                                  时若存在 VPR 数据库会用相机图像引导确定 submap
        :param vpr_device: VPR 子进程使用的设备（``cuda`` / ``cpu``）
        :param camera_image_topic: 用于 VPR 引导/重定位的 CompressedImage 话题；
                                    None 时使用 NavNode 默认值
        """
        # 方法名 -> 可调用（同步或 async），由 register_tools 填充
        self._handlers = {}
        # 可视化（rerun）引用；server.py main() 在创建 NaviRerunViz 后会注入。
        # 为 None 时所有 viz_* RPC 静默 no-op。
        self.viz = None
        # 多楼层地图管理器
        self.map_manager = map_manager
        # 初始化经纬度转换器
        self.wgs84_converter = WGS84Converter()
        self.wgs84_converter.initialize()
        # 标记用户是否显式指定 submap：影响启动时是否触发 VPR 引导
        self._submap_specified = submap_specified
        # VPR 子进程（独立 venv，按需懒启动）
        self.vpr_bootstrap: Optional[VprBootstrap] = None
        # VPR 高层共识 + 重定位编排器（依赖 ros_node / map_manager / _change_map，
        # 在 ROS 节点起来之后再构造）
        self.vpr_relocator: Optional[VprRelocator] = None
        if map_manager is not None:
            try:
                self.vpr_bootstrap = VprBootstrap(map_manager, device=vpr_device)
            except Exception as e:
                logging.error(f"VprBootstrap 初始化失败: {e}", exc_info=True)
                self.vpr_bootstrap = None
        # VPR watchdog 协程相关状态
        self._tf_watchdog_task: Optional[asyncio.Task] = None
        self._shutdown = False
        # 初始化ROS
        try:
            # # 创建导航节点实例
            if camera_image_topic:
                self.ros_node = NavNode(camera_image_topic=camera_image_topic)
            else:
                self.ros_node = NavNode()
            # 创建多线程执行器：3 个线程分别服务 TF 心跳 / 重量级消息 / call-queue 三个回调组
            self.executor = MultiThreadedExecutor(num_threads=3)
            # 将节点添加到执行器
            self.executor.add_node(self.ros_node)
            
            # 启动ROS spin线程
            # 创建后台守护线程运行ROS消息循环
            self.ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self.ros_thread.start()

            # 启动 VPR 子进程（如果发现至少一个 submap 有数据库）；
            # 这一步会一直阻塞到模型 ready，之后才进入 sync_map_state。
            if self.vpr_bootstrap is not None and self.vpr_bootstrap.has_databases:
                self.vpr_bootstrap.start()

            # 子进程 ready 后再构造 relocator —— 它需要 ros_node + _change_map 都
            # 可用。无论 VPR 是否真的 available，都创建实例：内部会自检并安全降级。
            if self.vpr_bootstrap is not None:
                self.vpr_relocator = VprRelocator(
                    vpr_bootstrap=self.vpr_bootstrap,
                    ros_node=self.ros_node,
                    map_manager=self.map_manager,
                    change_map_fn=self._change_map,
                )

            # 启动时若未指定 submap：让 relocator 做多帧共识识别 submap，
            # 再让 _sync_map_state 完成 change_map。
            if (
                not self._submap_specified
                and self.vpr_relocator is not None
                and self.vpr_relocator.available
            ):
                self.vpr_relocator.pick_initial_submap()
            elif (
                not self._submap_specified
                and map_manager is not None
            ):
                logging.warning(
                    "启动时未指定 submap 且 VPR 不可用，将使用 MapManager 当前 submap: %s",
                    map_manager.get_current_submap(),
                )

            # 同步地图状态：检查 MapManager 与 ROS 节点的当前地图是否一致
            self._sync_map_state()
        except Exception as e:
            logging.error(f"ROS初始化失败: {e}")
            raise

        # 注册MCP工具函数
        self.register_tools()

    def _change_map(
        self,
        target_submap: str,
        *,
        timeout: float = 60.0,
        poll_interval: float = 0.5,
        init_pose_in_target: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """统一的换图流程，被 ``_sync_map_state`` / VPR 引导 / VPR 重定位 /
        WebSocket RPC ``change_map`` 全部复用。

        完整步骤：

        1. 校验 ``target_submap`` 是否在 MapManager 中存在
        2. 取 ROS 端当前地图（``get_current_map_path``）；若已是 ``target_submap``
           则直接返回成功，不再下发 change_map 请求
        3. 解析 ``init_pose_in_target``：
           - 若调用方已传入（VPR 共识场景：直接把识别到的位置作为目标
             submap 中的初始位姿），则跳过跨图换算，直接使用；
           - 否则若已定位（``ros_node.current_pose`` 非空且能确定
             ``from_map_id``），用 ``map_manager.get_init_pose_after_changemap``
             计算；未定位则跳过位姿发布。
        4. 通过 ``ros_node.change_map`` 发送切换请求
        5. 轮询 ``get_current_map_path`` 直到目标地图加载完成或超时
        6. 更新 MapManager.current_submap
        7. 若步骤 3 解析出 ``init_pose_in_target``，发布到 ``/pose_in_target``

        :param init_pose_in_target: 可选。形如 ``{position: {x,y,z},
            orientation: {x,y,z,w}}``。VPR 重定位/引导路径会传入：用 VPR
            识别到的位置 + 单位四元数（朝向无意义）。其他调用方传 None
            走原有的跨图位姿换算路径。

        返回的字典与 WebSocket RPC ``change_map`` 兼容：
        ``{success, map_id, map_path, from_map_id, msg,
           pose_in_source?, pose_in_target?}``。
        """
        if not self.map_manager:
            return {"success": False, "msg": "MapManager 未配置"}

        # 1. map_id 合法性
        submap_list = self.map_manager.get_submap_list()
        if target_submap not in submap_list:
            return {
                "success": False,
                "msg": f"地图 {target_submap} 不存在，可用地图: {submap_list}",
            }

        # 2. ROS 端当前地图（以 ROS 为单一真相，避免 MapManager 缓存与实际不一致）
        current_map_path = self.ros_node.get_current_map_path()
        current_map_id_ros: Optional[str] = None
        if current_map_path:
            current_map_id_ros = self.map_manager.get_submap_id_by_map_file(current_map_path)
        # from_map_id：用于位姿换算与结果上报；ROS 端无法判定时回退到 MapManager 缓存
        from_map_id = current_map_id_ros or self.map_manager.get_current_submap()

        if current_map_id_ros == target_submap:
            # 同步缓存以防外部直接改过 ROS 端地图
            self.map_manager.set_current_submap(target_submap)
            return {
                "success": True,
                "map_id": target_submap,
                "map_path": current_map_path or "",
                "from_map_id": current_map_id_ros,
                "msg": f"当前已经是地图 {target_submap}，无需切换",
            }

        # 3. 解析 init_pose_in_target
        current_pose: Optional[Dict[str, Any]] = None
        odom = self.ros_node.current_pose
        if odom is not None:
            current_pose = {
                "position": {
                    "x": odom.pose.position.x,
                    "y": odom.pose.position.y,
                    "z": odom.pose.position.z,
                },
                "orientation": {
                    "x": odom.pose.orientation.x,
                    "y": odom.pose.orientation.y,
                    "z": odom.pose.orientation.z,
                    "w": odom.pose.orientation.w,
                },
            }

        if init_pose_in_target is not None:
            # 3a. VPR 等调用方直接给了目标 submap 中的位姿 —— 跳过跨图换算，
            #     仅做一次基本结构兜底（强制 z=0）。
            logging.info(
                "_change_map: 使用调用方提供的 init_pose_in_target，跳过跨图换算 "
                "(target=%s, pose=%s)",
                target_submap, init_pose_in_target,
            )
            if isinstance(init_pose_in_target, dict):
                pos = init_pose_in_target.get("position")
                if not isinstance(pos, dict):
                    init_pose_in_target["position"] = {"x": 0.0, "y": 0.0, "z": 0.0}
                else:
                    try:
                        init_pose_in_target["position"]["z"] = 0
                    except Exception:
                        init_pose_in_target["position"] = {
                            **init_pose_in_target.get("position", {}),
                            "z": 0,
                        }
        elif current_pose is not None and from_map_id:
            # 3b. 当前已定位时，按 from_map_id → target_submap 做跨图换算
            try:
                init_pose_in_target = self.map_manager.get_init_pose_after_changemap(
                    current_pose, from_map_id, target_submap
                )
                # 强制 z=0（目标地图为平面位姿）
                if init_pose_in_target and isinstance(init_pose_in_target, dict):
                    pos = init_pose_in_target.get("position")
                    if not isinstance(pos, dict):
                        init_pose_in_target["position"] = {"x": 0.0, "y": 0.0, "z": 0.0}
                    else:
                        try:
                            init_pose_in_target["position"]["z"] = 0
                        except Exception:
                            init_pose_in_target["position"] = {
                                **init_pose_in_target.get("position", {}),
                                "z": 0,
                            }
            except Exception as e:
                logging.warning(
                    "_change_map: 计算 init_pose_in_target 失败 (from=%s, to=%s): %s",
                    from_map_id, target_submap, e,
                )
                init_pose_in_target = None
        else:
            logging.info(
                "_change_map: 未定位且未提供 init_pose_in_target "
                "(current_pose=%s, from_map_id=%s)，跳过位姿换算",
                "set" if odom is not None else "None", from_map_id,
            )

        # 4. 下发 change_map
        target_map_path = self.map_manager.get_submap_full_map_path(target_submap)
        if not target_map_path:
            return {"success": False, "msg": f"无法获取地图 {target_submap} 的路径"}

        if not self.ros_node.change_map(target_map_path):
            return {
                "success": False,
                "msg": f"发送切换地图请求失败: {target_map_path}",
            }

        # 5. 轮询等待 ROS 端真正切到目标地图
        target_basename = os.path.basename(target_map_path)
        start_time = time.time()
        map_switched = False
        while time.time() - start_time < timeout:
            time.sleep(poll_interval)
            new_map_path = self.ros_node.get_current_map_path()
            if new_map_path:
                if (
                    new_map_path == target_map_path
                    or os.path.basename(new_map_path) == target_basename
                ):
                    map_switched = True
                    logging.info(f"地图切换完成: {new_map_path}")
                    break

        if not map_switched:
            return {
                "success": False,
                "msg": f"地图切换超时({timeout}秒)，目标地图: {target_map_path}",
            }

        # 6. 更新 MapManager 缓存
        self.map_manager.set_current_submap(target_submap)


        logging.warning(f"init_pose_in_target: {init_pose_in_target}")
        # 7. 若有初始位姿则发布到 /pose_in_target
        if init_pose_in_target:
            try:
                if isinstance(init_pose_in_target.get("position"), dict):
                    init_pose_in_target["position"]["z"] = 0
                self.ros_node.publish_pose_in_target(init_pose_in_target)
                logging.warning(f"已发布初始位姿到 /pose_in_target: {init_pose_in_target}")
            except Exception as e:
                logging.warning(f"_change_map: publish_pose_in_target 失败: {e}")
        else:
            logging.warning(f"init_pose_in_target为空，未发布初始位姿到 /pose_in_target")

        # 8. 等待定位就绪（is_map_localized 为 True）
        localize_timeout = timeout  # 复用同一超时参数
        localize_start = time.time()
        localized = False
        # 等待 3s 让 FAST-LIO 完成换图初始化再开始轮询，避免读到换图前的旧 TF
        time.sleep(3.0)
        while time.time() - localize_start < localize_timeout:
            if self.ros_node.is_map_localized():
                localized = True
                logging.warning(
                    "_change_map: 定位已就绪 (elapsed=%.1fs)",
                    time.time() - localize_start,
                )
                break
            time.sleep(poll_interval)

        if not localized:
            logging.warning(
                "_change_map: 等待定位超时(%.0fs)，地图已切换但定位尚未就绪",
                localize_timeout,
            )

        result: Dict[str, Any] = {
            "success": True,
            "map_id": target_submap,
            "map_path": target_map_path,
            "from_map_id": from_map_id,
            "localized": localized,
            "msg": f"已切换到地图 {target_submap}" + ("" if localized else "（定位超时，尚未就绪）"),
        }
        if current_pose is not None:
            result["pose_in_source"] = current_pose
        if init_pose_in_target is not None:
            result["pose_in_target"] = init_pose_in_target
        return result

    async def _tf_localization_watchdog(self) -> None:
        """后台监视 ``tf_ready``：定位丢失 grace 秒后触发一次 VPR 共识重定位。

        三种 cooldown 并存（重要，互相不可替代）：
        - ``grace_sec``           失定位至少持续这么久才认为是真的丢失
        - ``fail_cooldown_sec``   两次 VPR **尝试**之间的最小间隔（无论成败）
        - ``success_cooldown_sec`` VPR **成功**后即使 tf_ready 仍为 False，也
                                    在本静默期内不再触发 VPR（按需求点 4）
        """
        if self.vpr_relocator is None or not self.vpr_relocator.available:
            logging.info("[VPR-watchdog] VPR 不可用，跳过 tf_ready 监视")
            return

        grace_sec = 150.0
        fail_cooldown_sec = 30.0
        success_cooldown_sec = 60.0
        check_interval_sec = 2.0
        loss_since: Optional[float] = None
        last_attempt_t = 0.0
        last_success_t = 0.0

        logging.info(
            "[VPR-watchdog] 已启动 (grace=%.0fs, fail_cd=%.0fs, success_cd=%.0fs, interval=%.1fs)",
            grace_sec, fail_cooldown_sec, success_cooldown_sec, check_interval_sec,
        )

        while not self._shutdown:
            try:
                await asyncio.sleep(check_interval_sec)
                if self.ros_node.is_map_localized():
                    if loss_since is not None:
                        logging.info("[VPR-watchdog] TF 定位已恢复")
                    loss_since = None
                    continue
                now = time.monotonic()

                # 需求点 4：VPR 成功后 success_cooldown_sec 内静默
                if last_success_t > 0.0 and now - last_success_t < success_cooldown_sec:
                    remaining = success_cooldown_sec - (now - last_success_t)
                    # 只在第一次进入静默或长间隔时打日志，避免刷屏
                    if loss_since is None:
                        logging.info(
                            "[VPR-watchdog] tf_ready=False，但距上次 VPR 成功仅 "
                            "%.1fs，静默期剩余 %.1fs",
                            now - last_success_t, remaining,
                        )
                    loss_since = now  # 复用 loss_since 抑制重复日志
                    continue

                if loss_since is None:
                    loss_since = now
                    logging.warning(
                        "[VPR-watchdog] 检测到 TF 定位丢失，%.1fs 后若未恢复将尝试 VPR 重定位",
                        grace_sec,
                    )
                    continue
                if now - loss_since < grace_sec:
                    continue
                if now - last_attempt_t < fail_cooldown_sec:
                    continue
                last_attempt_t = now
                lost_for = now - loss_since
                # change_map / VPR 调用是同步阻塞的（含多帧采集），丢到线程池避免卡 asyncio
                success = await asyncio.to_thread(
                    self.vpr_relocator.relocate_and_change_map,
                    f"tf_ready=False 已 {lost_for:.1f}s",
                )
                if success:
                    last_success_t = time.monotonic()
                    logging.info(
                        "[VPR-watchdog] VPR 重定位成功，未来 %.0fs 内即使未定位也不再触发",
                        success_cooldown_sec,
                    )
            except asyncio.CancelledError:
                break
            except Exception as e:
                logging.error(f"[VPR-watchdog] 异常: {e}", exc_info=True)

        logging.info("[VPR-watchdog] 已退出")

    def _sync_map_state(self):
        """
        同步地图状态：把 ROS 端的当前地图对齐到 MapManager 配置的 submap。

        实际换图流程统一委托给 ``_change_map``（含 ROS 端一致性检查、位姿
        换算、轮询等待与 ``/pose_in_target`` 发布）。
        """
        if not self.map_manager:
            logging.info("未配置 MapManager，跳过地图同步")
            return

        try:
            # 给 ROS 节点一小段时间初始化（订阅 /map_path 等参数）
            time.sleep(1.0)

            expected = self.map_manager.get_current_submap()
            if not expected:
                logging.warning("MapManager 未设置当前 submap，跳过地图同步")
                return

            result = self._change_map(expected)
            if result.get("success"):
                logging.info(result.get("msg", f"地图已同步到 {expected}"))
            else:
                logging.error(f"地图同步失败: {result.get('msg')}")
        except Exception as e:
            logging.error(f"同步地图状态失败: {e}")

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

    # ══════════════════════════════════════════════════════════════════════
    # 混合路线：逐 waypoint 自动切换导航模式（SLAM 室内点云 ↔ 室外 GPS）
    #
    # 一条路线的 waypoint 可以是某张 SLAM 地图内的语义点（map），也可以是室外
    # 经纬度点（gps），还可能是进楼衔接的场景传送门（scene_point）。执行路线时
    # 逐个 waypoint 判断其所需导航模式：从「某张 SLAM 地图」跨到「室外」，或从
    # 「室外」跨回「某张 SLAM 地图」时，自动调用 NavNode.set_nav_mode 切换，并在
    # SLAM 侧对齐目标 slam_map / submap（跨 SLAM 地图时用 geo_anchor 播种初始位姿）。
    # ══════════════════════════════════════════════════════════════════════
    @staticmethod
    def _waypoint_nav_mode(wp: Dict[str, Any]) -> Optional[str]:
        """由 waypoint 类型推断所需导航模式：

        - ``gps``（经纬度 / OSM 主干）           → ``NavNode.NAV_MODE_GPS``（室外）
        - ``map``（语义点 / ``{x,y}``，属某张 SLAM 地图）→ ``NavNode.NAV_MODE_SLAM``
        - ``scene_point``（场景传送门，进楼衔接点） → ``NavNode.NAV_MODE_SLAM``
        - ``instruction``（VLN）/ 未知             → ``None``（不改变当前模式）
        """
        wtype = wp.get('type') or MapManager.classify_waypoint(wp)
        if wtype == 'gps':
            return NavNode.NAV_MODE_GPS
        if wtype in ('map', 'scene_point'):
            return NavNode.NAV_MODE_SLAM
        return None

    def get_route_mode_plan(self, route_name: str) -> Dict[str, Any]:
        """把一条路线展开成带导航模式标注的段计划（供混合路线执行 / 排障）。

        - 场景全局路线（顶层 ``routes``，异构 waypoint）：用 ``get_global_route_by_name``；
        - 旧单图内部路线：用 ``get_route_by_name``（全部为 SLAM 段，向后兼容）。

        :return: ``{success, route_name, waypoint_count, waypoints, segments}``；
                 每个 waypoint 追加 ``nav_mode`` 字段，``segments`` 把相邻同模式的
                 waypoint 归并成段（段边界即需要自动切换导航模式处）。
        """
        if not self.map_manager:
            return {"success": False, "msg": "未配置 MapManager"}
        route = None
        try:
            if route_name in self.map_manager.get_global_route_names():
                route = self.map_manager.get_global_route_by_name(route_name)
        except Exception as e:
            logging.warning(f"读取全局路线失败({route_name}): {e}")
        if route is None:
            route = self.map_manager.get_route_by_name(route_name)
        if route is None:
            return {"success": False, "msg": f"路线不存在: {route_name}"}

        waypoints = route.get('waypoints', [])
        annotated = []
        for wp in waypoints:
            item = dict(wp)
            item.setdefault('type', MapManager.classify_waypoint(wp))
            item['nav_mode'] = self._waypoint_nav_mode(item)
            annotated.append(item)

        segments: list = []
        for i, wp in enumerate(annotated):
            mode = wp['nav_mode']
            if segments and segments[-1]['mode'] == mode:
                segments[-1]['waypoints'].append(i)
            else:
                segments.append({
                    'index': len(segments),
                    'mode': mode,
                    'wp_type': wp['type'],
                    'waypoints': [i],
                })
        return {
            "success": True,
            "route_name": route_name,
            "waypoint_count": len(annotated),
            "waypoints": annotated,
            "segments": segments,
        }

    def _compute_slam_entry_pose(
        self, wp: Dict[str, Any], slam_map_id: Optional[str]
    ) -> Optional[Dict[str, Any]]:
        """进入某张 SLAM 地图时的初始位姿（best-effort，用于 ``/pose_in_target``）。

        - ``scene_point``：由传送门经纬度 + 朝向经 ``geo_anchor`` 反算根系位姿；
        - 其它（多为紧接室外 GPS 段进楼）：用当前 GNSS fix 经 ``geo_anchor`` 反算。

        失败返回 None——``_change_map`` 会跳过位姿发布，交由 VPR / 后续重定位兜底。
        """
        try:
            if wp.get('type') == 'scene_point' and wp.get('scene_point'):
                entry = self.map_manager.get_slam_entry_pose_from_scene_point(
                    wp['scene_point'])
                if entry and entry.get('pose'):
                    return entry['pose']
            fix = self.ros_node.get_latest_gps_fix()
            if fix and slam_map_id:
                return self.map_manager.lonlat_to_slam_root_pose(
                    fix['lon'], fix['lat'], None, slam_map_id)
        except Exception as e:
            logging.warning(f"计算进入 SLAM 初始位姿失败(best-effort): {e}")
        return None

    def _ensure_slam_context_for_waypoint(self, wp: Dict[str, Any]) -> Dict[str, Any]:
        """确保当前活跃 SLAM 地图 / submap 与该 map/scene_point waypoint 匹配。

        跨 SLAM 地图时：切 slam_map + 换图（``_change_map``）并 best-effort 播种
        初始位姿；同图内跨 submap 时：仅换图（复用现有跨 submap 位姿衔接）。
        """
        info: Dict[str, Any] = {"switched_slam_map": False, "switched_submap": False}
        if not self.map_manager:
            return info

        desired_map = wp.get('slam_map')
        desired_submap = wp.get('submap')
        seed_target_map = desired_map
        # 场景传送门：目标 SLAM 地图由传送门解析
        if wp.get('type') == 'scene_point':
            try:
                entry = self.map_manager.get_slam_entry_pose_from_scene_point(
                    wp.get('scene_point'))
            except Exception:
                entry = None
            if entry and entry.get('slam_map'):
                desired_map = entry['slam_map']
                seed_target_map = desired_map

        cur_map = self.map_manager.get_current_slam_map()
        # 1) 跨 SLAM 地图
        if desired_map and desired_map != cur_map:
            seed = self._compute_slam_entry_pose(wp, seed_target_map)
            if not self.map_manager.set_current_slam_map(desired_map):
                info["error"] = f"SLAM 地图不存在: {desired_map}"
                return info
            info["switched_slam_map"] = True
            info["slam_map"] = desired_map
            target_submap = desired_submap or self.map_manager.get_current_submap()
            info["change_map"] = self._change_map(
                target_submap, init_pose_in_target=seed)
            info["switched_submap"] = True
            return info
        # 2) 同一 SLAM 地图内跨 submap
        if desired_submap and desired_submap != self.map_manager.get_current_submap():
            info["change_map"] = self._change_map(desired_submap)
            info["switched_submap"] = True
        return info

    def _ensure_nav_mode(self, target_mode: str) -> Dict[str, Any]:
        """确保 NavNode 处于目标导航模式；已在目标模式则空操作。

        供 set_goal 系列在下发前对齐模式（SLAM 语义点/坐标 → slam；经纬度 → gps）。
        """
        cur = self.ros_node.get_nav_mode()
        if cur == target_mode:
            return {"switched": False, "nav_mode": cur}
        ok, msg = self.ros_node.set_nav_mode(target_mode)
        return {
            "switched": bool(ok), "from": cur,
            "nav_mode": self.ros_node.get_nav_mode(), "msg": msg,
        }

    def register_tools(self):
        """
        注册 WebSocket RPC 方法（与原先 MCP 工具一一对应）

        客户端 JSON: {\"id\": optional, \"method\": 方法名, \"params\": {...}}
        服务端回复: {\"id\": ..., \"result\": ...} 或 {\"id\": ..., \"error\": \"...\"}
        """
        def set_goal_by_slam(target: str, x: float = None, y: float = None,
                             slam_map: str = None, submap: str = None,
                             distance: float = 0.5) -> Dict[str, Any]:
            """
            下发导航目标（SLAM 模式）。两种用法：

            1. **命名语义点**（``target`` 为地图预设点名，如 狗巢/配电房/灭火器箱）：
               从 MapManager 取该点位姿下发，支持 ``slam_map``/``submap`` 跨图切换与
               ``elevator_*`` 类型处理。
            2. **临时坐标点**（``target == "tmp"``）：不查 MapManager，直接以入参 ``x``/``y``
               为目标（原 set_goal_by_xy 逻辑：在距目标 ``distance`` 米处停下并面向目标）。

            参数:
            target: 目标点名称；传 ``"tmp"`` 表示使用 ``x``/``y`` 临时坐标。
            x, y: 当 ``target == "tmp"`` 时必填，地图(odom)系坐标。
            slam_map: 可选（仅命名点位），目标所在 SLAM 地图 id；与当前不同会自动切换。
            submap: 可选（仅命名点位），目标所在 submap id；与当前不同会自动换图。
            distance: 仅 ``target == "tmp"`` 时生效，停靠在距目标的距离（米），默认 0.5。

            说明（命名语义点）:
            - 语义点属于 SLAM 导航：本接口会自动确保处于 `slam` 模式（若当前为 `gps` 室外模式则切回）。
            - 适用于 `type` 为 `single_pose` 的点位；调用此接口会使用普通导航下发目标。
            - 如果点位 `type` 为 `elevator_call`：机器人到达该点后应等待 agent 执行呼叫电梯的操作（本接口只负责下发导航）。
            - 如果点位 `type` 为 `elevator_wait`：机器人到达该点后应等待 agent 确认电梯已经到达并开门。
            - 如果点位 `type` 为 `elevator_in`：下发目标时自动调用 `enter_elevator`（锁定 Z 轴），在前往电梯途中即生效。
            - 如果点位 `type` 为 `elevator_out`：`wait_goal_reached` 在判定到达后自动调用 `exit_elevator`（解锁 Z 轴）。

            返回:
            Dict: 包含操作结果的字典
            """
            try:
                # 均属 SLAM：确保处于 slam 模式（室外→室内自动切换）
                mode_switch = self._ensure_nav_mode(NavNode.NAV_MODE_SLAM)

                # ── target == "tmp"：临时坐标点，不查 MapManager，直接用 x/y ──
                if target == "tmp":
                    if x is None or y is None:
                        return {"success": False, "msg": "target 为 tmp 时必须提供 x、y 坐标",
                                "mode_switch": mode_switch}
                    x, y = float(x), float(y)
                    if not (self.ros_node.map_bound["xmin"] <= x <= self.ros_node.map_bound["xmax"] and
                            self.ros_node.map_bound["ymin"] <= y <= self.ros_node.map_bound["ymax"]):
                        return {"success": False, "msg": f"目标点({x}, {y})超出地图边界"}

                    odom = self.ros_node.current_pose
                    if not odom:
                        return {"success": False, "msg": "无法获取当前位置信息"}
                    curr_x = odom.pose.position.x
                    curr_y = odom.pose.position.y

                    # 计算距目标 distance 米、面向目标的实际停靠位姿
                    dx = x - curr_x
                    dy = y - curr_y
                    target_distance = (dx**2 + dy**2) ** 0.5
                    if target_distance <= distance:
                        actual_x, actual_y = curr_x, curr_y
                    else:
                        unit_dx = dx / target_distance
                        unit_dy = dy / target_distance
                        actual_x = x - unit_dx * distance
                        actual_y = y - unit_dy * distance

                    if not (self.ros_node.map_bound["xmin"] <= actual_x <= self.ros_node.map_bound["xmax"] and
                            self.ros_node.map_bound["ymin"] <= actual_y <= self.ros_node.map_bound["ymax"]):
                        return {"success": False, "msg": f"计算的实际目标位置({actual_x:.2f}, {actual_y:.2f})超出地图边界"}

                    face_dx = x - actual_x
                    face_dy = y - actual_y
                    yaw = np.arctan2(face_dy, face_dx)
                    qz = np.sin(yaw / 2)
                    qw = np.cos(yaw / 2)

                    pose = PoseStamped()
                    pose.header.frame_id = "odom"
                    pose.pose.position.x = actual_x
                    pose.pose.position.y = actual_y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw

                    target_str = f"tmp({x},{y})_dist_{distance}"
                    self.ros_node.publish_goal(pose, target_str)
                    return {
                        "success": True,
                        "target": "tmp",
                        "target_str": target_str,
                        "mode_switch": mode_switch,
                        "msg": f"已下发临时目标，将在距离({x}, {y})约{distance}米处停下并面向目标点",
                        "original_target": {"x": x, "y": y},
                        "actual_target": {"x": actual_x, "y": actual_y},
                        "distance": distance,
                    }

                # ── 命名语义点：从 MapManager 取位姿 ──
                # 若显式指定 slam_map/submap，先对齐地图上下文（跨图/跨 submap 自动换图）
                ctx = None
                if (slam_map or submap) and self.map_manager:
                    ctx = self._ensure_slam_context_for_waypoint(
                        {"type": "map", "slam_map": slam_map, "submap": submap})

                # 检查目标点是否在预设地图中
                if target not in self.map_manager.pose_map_current_submap:
                    return {"success": False, "msg": f"目标点 {target} 不在白名单中",
                            "mode_switch": mode_switch, "slam_context": ctx}

                # 获取目标点的位姿信息
                pose_info = self.map_manager.pose_map_current_submap[target].get("pose")
                if not pose_info:
                    return {"success": False, "msg": f"目标点 {target} 缺少坐标信息"}

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

                point_info = self.map_manager.pose_map_current_submap[target]
                if point_info.get("type") == "elevator_in":
                    ok, emsg = self.ros_node.set_elevator_mode(enter=True)
                    logging.info("前往 elevator_in 点位 %s，enter_elevator: %s %s", target, ok, emsg)

                self.ros_node.publish_goal(pose, target)
                return {"success": True, "msg": f"目标 {target} 已下发", "target": target,
                        "mode_switch": mode_switch}
            except Exception as e:
                return {"success": False, "msg": f"设置目标失败: {str(e)}"}

        def set_goal_by_outdoor(target: str, lon: float = None, lat: float = None,
                                tolerance: float = 3.0) -> Dict[str, Any]:
            """
            按语义点下发室外 GPS 导航目标（campus_nav 路网规划器）。

            仅支持室外 GPS 模式，调用时自动切到 gps 模式（不再支持 SLAM 换算）。

            两种用法：
            - **命名场景语义点**（``target`` 为具体点名）：经纬度**始终**从 MapManager 的
              场景语义点 ``target`` 读取，忽略入参 ``lon``/``lat``。
            - **临时坐标点**（``target == "tmp"``）：不查 MapManager，直接以入参 ``lon``/``lat``
              为目标（此时 ``lon``/``lat`` 必填）。

            参数:
            target: 必填，目标语义点名称；传 ``"tmp"`` 表示用 ``lon``/``lat`` 临时坐标。
                    随目标记录，供 `wait_goal_reached` 以该 target 判定是否到达
                    （GPS 无 Nav2 到达回调，按几何距离阈值判定）。
            lon: 目标点经度；仅 ``target == "tmp"`` 时使用且必填。
            lat: 目标点纬度；仅 ``target == "tmp"`` 时使用且必填。
            tolerance: 到达判定距离阈值（米），默认 3.0；随目标记录并供 `wait_goal_reached` 使用。

            返回:
            Dict: 操作结果
            """
            try:
                if not target:
                    return {"success": False, "msg": "target 不能为空"}
                # 仅 target == "tmp" 时用入参 lon/lat；否则从场景语义点解析（忽略入参 lon/lat）。
                if target == "tmp":
                    if lon is None or lat is None:
                        return {"success": False, "msg": "target 为 tmp 时必须提供 lon、lat"}
                else:
                    if self.map_manager is None:
                        return {"success": False,
                                "msg": "未加载地图，无法从 target 解析经纬度，请用 target=tmp 并显式传入 lon/lat"}
                    pt = self.map_manager.get_scene_point(target)
                    if not pt or pt.get("lon") is None or pt.get("lat") is None:
                        return {"success": False,
                                "msg": f"场景语义点 '{target}' 不存在或缺少经纬度，请用 target=tmp 并显式传入 lon/lat"}
                    lon, lat = pt["lon"], pt["lat"]

                mode_switch = self._ensure_nav_mode(NavNode.NAV_MODE_GPS)
                ok = self.ros_node.set_outdoor_goal_by_lonlat(
                    float(lon), float(lat), target=target, tolerance=float(tolerance))
                return {
                    "success": bool(ok),
                    "mode": "gps",
                    "mode_switch": mode_switch,
                    "target": target,
                    "goal": {"lon": lon, "lat": lat},
                    "tolerance": tolerance,
                    "msg": (f"已向室外路网规划器下发目标 target={target} ({lon},{lat})"
                            if ok else "室外经纬度目标下发失败"),
                }
            except Exception as e:
                return {"success": False, "msg": f"经纬度导航失败: {str(e)}"}

        def pause_navigation() -> Dict[str, Any]:
            """
            暂停导航（取消当前目标）
            """
            try:
                self.ros_node.pause_navigation()
                return {"success": True, "msg": "导航已暂停（已取消当前目标）"}
            except Exception as e:
                return {"success": False, "msg": f"暂停导航失败: {str(e)}"}

        def cancel_navigation() -> Dict[str, Any]:
            """
            取消导航
            """
            try:
                self.ros_node.cancel_navigation()
                return {"success": True, "msg": "导航已取消"}
            except Exception as e:
                return {"success": False, "msg": f"取消导航失败: {str(e)}"}

        def resume_navigation() -> Dict[str, Any]:
            """
            继续导航（重新发送上一个目标）
            """
            try:
                self.ros_node.resume_navigation()
                return {"success": True, "msg": "导航已继续"}
            except Exception as e:
                return {"success": False, "msg": f"继续导航失败: {str(e)}"}

        def get_pose_list_in_current_submap(slam_map: str = None, submap_id: str = None) -> Dict[str, Any]:
            """
            获取预设点名称列表（默认当前 submap；可指定 SLAM 地图 / submap）

            :param slam_map: 可选 SLAM 地图 id（多 SLAM 地图场景），默认当前
            :param submap_id: 可选 submap id，默认当前
            返回:
            Dict: {"success": True, "keys": [...], "count": N, "slam_map": ..., "submap": ...}
            """
            try:
                if slam_map or submap_id:
                    points = self.map_manager.get_semantic_points_in_submap(submap_id, slam_map)
                    keys = list(points.keys())
                else:
                    keys = list(self.map_manager.pose_map_current_submap.keys())
                return {
                    "success": True, "keys": keys, "count": len(keys),
                    "slam_map": slam_map or self.map_manager.get_current_slam_map(),
                    "submap": submap_id or self.map_manager.get_current_submap(),
                }
            except Exception as e:
                return {"success": False, "msg": f"获取pose列表失败: {str(e)}"}

        def get_pose_in_submap(pose_name: str, submap_id: str = None, slam_map: str = None) -> Dict[str, Any]:
            """
            获取指定点位的完整 point_info（由 MapManager.get_pose_in_submap 提供）

            :param pose_name: 点位名称
            :param submap_id: 可选 submap ID，默认为当前 submap
            :param slam_map: 可选 SLAM 地图 id（多 SLAM 地图场景），默认当前
            :return: {success: bool, point_info: {...}} 或 error msg

            point_info 中 `type` 字段说明：
            - "single_pose": 单一位姿点；使用普通导航接口 `set_goal_by_slam` 下发目标。
            - "multi_pose": 多位姿点（一串按顺序经过的位姿，用 `poses` 数组）；当前无专用下发接口。
            - "LowPrecision_pose": 低精度点位；到达后不要求位置和朝向精确，使用较大容差即可。
            - "elevator_call": 电梯呼叫点；机器人到达该点后应等待外部 agent 执行呼叫电梯的动作。
            - "elevator_wait": 电梯等待点；机器人到达该点后应等待外部 agent 确认电梯已到达且车门已打开。
            - "elevator_in": 进梯点；`set_goal_by_slam` 下发目标时自动调用 `enter_elevator`（锁 Z）。
            - "elevator_out": 出梯点；`wait_goal_reached` 到达后自动调用 `exit_elevator`（解锁 Z）。
            - "door_open": 开门点；机器人到达该点后需要执行开门动作。
            """
            try:
                if not self.map_manager:
                    return {"success": False, "msg": "MapManager 未配置"}

                point_info = self.map_manager.get_pose_in_submap(pose_name, submap_id, slam_map)
                if point_info is None:
                    return {"success": False, "msg": f"点位 {pose_name} 在子地图 {submap_id or self.map_manager.get_current_submap()} 中不存在"}

                return {"success": True, "pose_name": pose_name, "point_info": point_info}
            except Exception as e:
                return {"success": False, "msg": f"获取点位失败: {str(e)}"}

        def get_submap_list(slam_map: str = None) -> Dict[str, Any]:
            """
            获取子地图 ID 列表（默认当前 SLAM 地图；可指定其它 SLAM 地图）

            参数:
            slam_map: 可选 SLAM 地图 id（多 SLAM 地图场景），默认当前

            返回:
            Dict: {"success": True, "submaps": [...], "count": N, "slam_map": ..., "slam_maps": [...]}
            """
            try:
                submaps = self.map_manager.get_submap_list(slam_map)
                return {
                    "success": True, "submaps": submaps, "count": len(submaps),
                    "slam_map": slam_map or self.map_manager.get_current_slam_map(),
                    "slam_maps": self.map_manager.get_slam_map_list(),
                }
            except Exception as e:
                return {"success": False, "msg": f"获取子地图列表失败: {str(e)}"}

        def get_submap_info(submap_id: str, slam_map: str = None) -> Dict[str, Any]:
            """
            获取指定子地图的详细信息（默认当前 SLAM 地图；可指定其它 SLAM 地图）

            参数:
            submap_id: 子地图 ID（如 floor, underground, 5floor）
            slam_map: 可选 SLAM 地图 id（多 SLAM 地图场景），默认当前

            返回:
            Dict: {"success": True, "submap_id": "...", "slam_map": "...", "info": {...}}
            """
            try:
                info = self.map_manager.get_submap_info(submap_id, slam_map)
                if info is None:
                    submaps = self.map_manager.get_submap_list(slam_map)
                    return {"success": False, "msg": f"子地图 {submap_id} 不存在，可用子地图: {submaps}"}
                return {
                    "success": True,
                    "submap_id": submap_id,
                    "slam_map": slam_map or self.map_manager.get_current_slam_map(),
                    "info": info
                }
            except Exception as e:
                return {"success": False, "msg": f"获取子地图信息失败: {str(e)}"}

        def get_current_map_id() -> Dict[str, Any]:
            """
            获取当前加载的地图信息

            返回:
            Dict: {"success": True, "map_id": "...", "map_path": "..."}
            """
            try:
                slam_map = self.map_manager.get_current_slam_map() if self.map_manager else None
                # 室外 GPS 模式：无 SLAM 点云地图加载，且室外栈不含
                # grid_map_pcl_loader_node（查询它会白等 2s 超时）。直接短路返回，
                # 明确 map_id 为空，避免回退到过时的室内 submap 造成误导。
                if self.ros_node.get_nav_mode() == NavNode.NAV_MODE_GPS:
                    return {
                        "success": True,
                        "nav_mode": NavNode.NAV_MODE_GPS,
                        "map_id": None,
                        "slam_map": None,
                        "map_path": "",
                        "msg": "当前为室外 GPS 导航模式，无活跃 SLAM 点云地图",
                    }
                map_path = self.ros_node.get_current_map_path()
                if map_path:
                    map_id = self.map_manager.get_submap_id_by_map_file(map_path)
                    return {
                        "success": True,
                        "nav_mode": NavNode.NAV_MODE_SLAM,
                        "map_id": map_id,
                        "slam_map": slam_map,
                        "map_path": map_path,
                        "msg": f"当前地图: {slam_map + '/' if slam_map else ''}{map_id or '未知'} ({map_path})"
                    }
                # ROS 参数服务超时，回退到 MapManager 缓存
                cached_id = self.map_manager.get_current_submap()
                if cached_id:
                    return {
                        "success": True,
                        "nav_mode": NavNode.NAV_MODE_SLAM,
                        "map_id": cached_id,
                        "slam_map": slam_map,
                        "map_path": "",
                        "msg": f"当前地图(缓存): {cached_id}（ROS参数服务暂不可用）"
                    }
                return {"success": False, "msg": "无法获取当前地图路径，且无缓存"}
            except Exception as e:
                return {"success": False, "msg": f"获取当前地图失败: {str(e)}"}

        def change_map(map_id: str, timeout: float = 60.0) -> Dict[str, Any]:
            """
            切换到指定地图

            参数:
            map_id: 目标地图 ID（如 floor, underground, 5floor）
            timeout: 等待地图切换完成的超时时间（秒），默认60秒

            返回:
            Dict: {"success": True, "map_id": "...", "map_path": "...",
                   "from_map_id": "...", "pose_in_source": {...},
                   "pose_in_target": {...}, "msg": "..."}
            """
            try:
                return self._change_map(map_id, timeout=timeout)
            except Exception as e:
                return {"success": False, "msg": f"切换地图失败: {str(e)}"}

        def get_portal_to_submap(target_submap: str) -> Dict[str, Any]:
            """
            获取从当前子地图到目标子地图的传送门

            参数:
            target_submap: 目标子地图 ID（如 floor, underground, 5floor）

            若当前子地图有多条 portal 均可直达同一目标子地图，则根据机器人当前位姿
            （map→base 可用时）与各 portal 语义点位置的 XY 距离选取最近的一条。

            返回:
            Dict: {"success": True, "portal": "...", "target_submap": "...", "from_submap": "...",
                   "portal_candidates": [...], "selected_by_nearest": bool}
            """
            try:
                # 获取当前 submap
                current_submap = self.map_manager.get_current_submap()
                if not current_submap:
                    return {"success": False, "msg": "当前子地图未设置"}
                
                # 检查目标 submap 是否存在
                submap_list = self.map_manager.get_submap_list()
                if target_submap not in submap_list:
                    return {
                        "success": False,
                        "msg": f"目标子地图 {target_submap} 不存在，可用子地图: {submap_list}"
                    }
                
                # 如果目标就是当前 submap
                if target_submap == current_submap:
                    return {
                        "success": False,
                        "msg": f"目标子地图 {target_submap} 就是当前子地图，无需传送门"
                    }
                
                candidates = self.map_manager.get_portals_to_submap(
                    target_submap, current_submap
                )
                robot_xyz = None
                cp = self.ros_node.current_pose
                if cp is not None:
                    try:
                        p = cp.pose.position
                        robot_xyz = (float(p.x), float(p.y), float(p.z))
                    except Exception:
                        robot_xyz = None

                portal_config = self.map_manager.get_portal_to_submap(
                    target_submap,
                    from_submap=current_submap,
                    robot_position=robot_xyz,
                    xy_distance_only=True,
                )
                if not portal_config:
                    return {
                        "success": False,
                        "msg": f"从 {current_submap} 无直达传送门到 {target_submap}",
                    }
                
                # 获取传送门的位姿
                portal_name = portal_config.get('portal')
                portal_point_info = self.map_manager.get_portal_pose(portal_name)
                # MapManager.get_portal_pose now returns full point_info; here we only need the 'pose' field
                portal_pose = None
                if portal_point_info and isinstance(portal_point_info, dict):
                    portal_pose = portal_point_info.get('pose')

                cand_names = [
                    c.get("portal")
                    for c in candidates
                    if c.get("portal")
                ]
                selected_by_nearest = len(candidates) > 1 and robot_xyz is not None
                msg = f"从 {current_submap} 到 {target_submap} 的传送门是: {portal_name}"
                if len(candidates) > 1:
                    if robot_xyz is not None:
                        msg += f"（共 {len(candidates)} 个候选，已按当前位置选最近）"
                    else:
                        msg += f"（共 {len(candidates)} 个候选，无当前位姿时使用第一个: {cand_names}）"

                return {
                    "success": True,
                    "portal": portal_name,
                    "target_submap": target_submap,
                    "from_submap": current_submap,
                    "portal_pose": portal_pose,
                    "portal_candidates": cand_names,
                    "selected_by_nearest": selected_by_nearest,
                    "msg": msg,
                }
            except Exception as e:
                return {"success": False, "msg": f"获取传送门失败: {str(e)}"}

        def get_route_names() -> Dict[str, Any]:
            """
            获取所有巡逻线路名称（含场景级全局路线与当前 SLAM 地图的内部路线）。

            返回:
            Dict: {"success": True, "route_names": [...], "routes": [...],
                   "global_routes": [...], "slam_routes": [...], "count": N}
                  routes 每项含 ``scope``（``global`` 跨地图 / ``slam`` 当前 SLAM 地图内部）。
            """
            try:
                slam_routes = self.map_manager.get_route_list()
                global_routes = []
                try:
                    global_routes = self.map_manager.get_global_route_list()
                except Exception:
                    global_routes = []
                combined = []
                for r in global_routes:
                    combined.append({**r, "scope": "global"})
                for r in slam_routes:
                    combined.append({**r, "scope": "slam"})
                return {
                    "success": True,
                    "route_names": [r['name'] for r in combined],
                    "routes": combined,
                    "global_routes": [r['name'] for r in global_routes],
                    "slam_routes": [r['name'] for r in slam_routes],
                    "count": len(combined)
                }
            except Exception as e:
                return {"success": False, "msg": f"获取路线列表失败: {str(e)}"}

        def get_default_route() -> Dict[str, Any]:
            """
            获取默认路线名称（地图配置中的 global.default_route）。

            返回:
            Dict: {"success": True, "default_route": "..."} 或 success=false
            """
            try:
                name = self.map_manager.get_default_route()
                if name:
                    return {"success": True, "default_route": name}
                return {"success": False, "msg": "未配置 default_route"}
            except Exception as e:
                return {"success": False, "msg": f"获取默认路线失败: {str(e)}"}

        def get_route(route_name: str) -> Dict[str, Any]:
            """
            根据路线名称获取巡逻点位列表。

            - 场景级全局路线：返回异构 waypoint（每个带 ``type``：map/gps/instruction/scene_point），
              并附带按导航模式归并的 ``segments``（段边界即 SLAM↔室外切换处）。
            - 当前 SLAM 地图内部路线：返回展开后的 ``{submap, semantic_point}`` waypoint。

            Args:
                route_name: 路线名称（如 线路A）

            Returns:
            Dict: 包含路线描述、巡逻点位列表（全局路线含 type/nav_mode）与可选 segments。
            """
            try:
                is_global = False
                try:
                    is_global = route_name in self.map_manager.get_global_route_names()
                except Exception:
                    is_global = False
                if is_global:
                    plan = self.get_route_mode_plan(route_name)
                    if not plan.get("success"):
                        return plan
                    return {
                        "success": True,
                        "scope": "global",
                        "name": route_name,
                        "waypoints": plan["waypoints"],
                        "segments": plan["segments"],
                        "count": plan["waypoint_count"],
                    }
                route_info = self.map_manager.get_route_by_name(route_name)
                if not route_info:
                    return {"success": False, "msg": f"路线不存在: {route_name}"}
                return {
                    "success": True,
                    "scope": "slam",
                    "name": route_info['name'],
                    "description": route_info['description'],
                    "waypoints": route_info['waypoints'],
                    "count": len(route_info['waypoints'])
                }
            except Exception as e:
                return {"success": False, "msg": f"获取路线失败: {str(e)}"}

        def viz_set_active_route(route_name: str) -> Dict[str, Any]:
            """告诉可视化（rerun）当前正在导航的 route。

            客户端在开始按 route 巡检时调用此方法；viz 会画出这条 route 的
            折线与 waypoints。未启动 viz（即未带 ``-v``）时静默成功。

            Args:
                route_name: 路线名称（必须在 map_manager 中存在）

            Returns:
                Dict: ``{"success": True}`` 或 ``{"success": False, "msg": ...}``
            """
            if not route_name or not isinstance(route_name, str):
                return {"success": False, "msg": "route_name 不能为空"}
            try:
                is_global = route_name in self.map_manager.get_global_route_names()
                if not is_global and self.map_manager.get_route_by_name(route_name) is None:
                    return {"success": False, "msg": f"路线不存在: {route_name}"}
            except Exception as e:
                return {"success": False, "msg": f"校验路线失败: {e}"}
            if self.viz is not None:
                try:
                    self.viz.set_active_route(route_name)
                except Exception as e:
                    return {"success": False, "msg": f"viz set_active_route 失败: {e}"}
            return {"success": True, "route_name": route_name, "viz_enabled": self.viz is not None}

        def viz_clear_active_route() -> Dict[str, Any]:
            """通知可视化清掉当前导航 route 的显示（导航结束 / 取消时调用）。

            未启动 viz 时静默成功。
            """
            if self.viz is not None:
                try:
                    self.viz.set_active_route(None)
                except Exception as e:
                    return {"success": False, "msg": f"viz clear 失败: {e}"}
            return {"success": True, "viz_enabled": self.viz is not None}


        def get_status() -> Dict[str, Any]:
            """
            获取小车实时状态（含导航模式与按模式判定的定位状态）。

            返回:
            Dict: success 为 True 时含 position、orientation；并含：
              - nav_mode: 当前导航模式（``slam`` / ``gps``）
              - localized: 按当前模式判定是否已定位（slam 看 map→base_link TF；gps 看 GNSS fix + 位姿）
              - map_localized / gps_available: 两种定位判据的原始值（便于排障）
              - slam_map: 当前活跃 SLAM 地图 id
              - gps_fix: gps 模式下的最新 GNSS fix（若有）
            """
            try:
                # 获取当前里程计数据
                odom = self.ros_node.current_pose
                nav_mode = self.ros_node.get_nav_mode()
                if not odom:
                    # 无位姿时也回报模式/定位判据，便于上层等待
                    return {
                        "success": False, "msg": "无实时位姿数据",
                        "nav_mode": nav_mode,
                        "localized": self.ros_node.is_localized(),
                        "map_localized": self.ros_node.is_map_localized(),
                        "gps_available": self.ros_node.is_gps_available(),
                    }

                status: Dict[str, Any] = {
                    "success": True,
                    "nav_mode": nav_mode,
                    "localized": self.ros_node.is_localized(),
                    "map_localized": self.ros_node.is_map_localized(),
                    "gps_available": self.ros_node.is_gps_available(),
                    "slam_map": (self.map_manager.get_current_slam_map()
                                 if self.map_manager else None),
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
                    },
                }
                if nav_mode == NavNode.NAV_MODE_GPS:
                    status["gps_fix"] = self.ros_node.get_latest_gps_fix()
                return status
            except Exception as e:
                return {"success": False, "msg": f"获取状态失败: {str(e)}"}

        async def wait_goal_reached(target: str, timeout: float = 120.0, xy_tolerance: float = 0.1, yaw_tolerance: float = 0.1) -> Dict[str, Any]:
            """
            阻塞直到到达目标点或超时

            - 室外 GPS 模式：按 target 关联的室外目标做几何距离判定
              （campus 规划器无“到达”回调，用 `is_outdoor_goal_reached` 的距离阈值，
              阈值取 `set_goal_by_outdoor` 时记录的 tolerance）。
            - SLAM 模式：按 Nav2 + PID 精调结果判定（`check_goal_reached`）。

            参数:
            target: 目标点名称（GPS 模式下即 `set_goal_by_outdoor` 传入的语义点名称）
            timeout: 超时时间（秒）

            返回:
            Dict: 包含到达结果的字典
            """
            # ── 室外 GPS：按 target 关联的室外目标做距离判定 ──
            if self.ros_node.get_nav_mode() == NavNode.NAV_MODE_GPS:
                tol = self.ros_node.get_outdoor_goal_tolerance()
                try:
                    start_time = time.time()
                    while time.time() - start_time < timeout:
                        await asyncio.sleep(0.2)
                        if self.ros_node.is_outdoor_goal_reached(tol):
                            return {"success": True, "msg": f"已到达目标 {target}",
                                    "timestamp": int(time.time())}
                    return {"success": False, "msg": "等待超时或未到达", "nav_failed": False}
                except Exception as e:
                    return {"success": False, "msg": f"等待目标失败: {str(e)}"}

            # elevator_in / LowPrecision_pose 强制宽松容差（SLAM）
            if self.map_manager:
                pt = self.map_manager.get_pose_in_submap(target)
                if pt and pt.get("type") in ("elevator_in", "LowPrecision_pose"):
                    xy_tolerance = max(xy_tolerance, 0.6)
                    yaw_tolerance = max(yaw_tolerance, 3.14)
            try:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    await asyncio.sleep(0.05)
                    status, fail_msg = self.ros_node.check_goal_reached(xy_tolerance, yaw_tolerance)
                    if status == self.ros_node.GOAL_CHECK_REACHED:
                        return {"success": True, "msg": f"已到达目标 {target}", "timestamp": int(time.time())}
                    if status == self.ros_node.GOAL_CHECK_FAILED:
                        return {
                            "success": False,
                            "msg": fail_msg or "导航失败",
                            "nav_failed": True,
                        }
                # 超时仍未到达（非导航失败）
                return {"success": False, "msg": "等待超时或未到达", "nav_failed": False}
            except Exception as e:
                return {"success": False, "msg": f"等待目标失败: {str(e)}"}

        async def get_navigation_plan(timeout: float = 10.0) -> Dict[str, Any]:
            """
            获取 Nav2 发布在 /plan 上的全局路径（nav_msgs/Path）。

            注意：每次 ``publish_goal`` / ``set_goal_*`` 下发新目标时，节点会清空缓存并递增代次，
            只保留与当前目标对应的新路径；请在下发目标后再调用本接口等待新 plan。

            参数:
            timeout: 等待非空路径的最长时间（秒）

            返回:
            success 为 True 时含 ``plan`` 字段：frame_id、stamp、poses、pose_count
            """
            start = time.time()
            try:
                while time.time() - start < timeout:
                    snap = self.ros_node.get_navigation_plan_snapshot()
                    if snap is not None:
                        return {
                            "success": True,
                            "plan": snap,
                            "msg": f"已获取全局路径，共 {snap['pose_count']} 个路径点",
                        }
                    await asyncio.sleep(0.2)
                return {
                    "success": False,
                    "msg": f"{timeout}s 内未收到非空的 /plan（请确认已下发导航目标且 Nav2 planner 已订阅）",
                }
            except Exception as e:
                return {"success": False, "msg": f"获取导航路径失败: {str(e)}"}

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

        def set_xy_goal_tolerance(tolerance: float) -> Dict[str, Any]:
            """
            临时修改 Nav2 controller_server 的 xy_goal_tolerance 参数。

            参数:
            tolerance: 新的位置容差（米），例如 0.3

            返回:
            Dict: {success, msg}
            """
            try:
                ok, msg = self.ros_node.set_remote_parameter(
                    '/controller_server',
                    'general_goal_checker.xy_goal_tolerance',
                    float(tolerance),
                )
                return {"success": ok, "msg": msg}
            except Exception as e:
                return {"success": False, "msg": f"设置 xy_goal_tolerance 失败: {str(e)}"}

        def enter_elevator() -> Dict[str, Any]:
            """
            进入电梯模式：锁定 Z 轴，防止电梯升降导致定位漂移。
            应在机器人进入电梯前调用。

            返回:
            Dict: {success, msg}
            """
            try:
                ok, msg = self.ros_node.set_elevator_mode(enter=True)
                return {"success": ok, "msg": msg}
            except Exception as e:
                return {"success": False, "msg": f"进入电梯模式失败: {str(e)}"}

        def exit_elevator() -> Dict[str, Any]:
            """
            退出电梯模式：解锁 Z 轴，恢复正常 3D 里程计。
            应在机器人出电梯后调用。

            返回:
            Dict: {success, msg}
            """
            try:
                ok, msg = self.ros_node.set_elevator_mode(enter=False)
                return {"success": ok, "msg": msg}
            except Exception as e:
                return {"success": False, "msg": f"退出电梯模式失败: {str(e)}"}

        def get_nav_mode() -> Dict[str, Any]:
            """获取当前导航模式（``slam`` 室内点云定位 / ``gps`` 室外 GNSS）。"""
            try:
                return {"success": True, "nav_mode": self.ros_node.get_nav_mode()}
            except Exception as e:
                return {"success": False, "msg": f"获取导航模式失败: {str(e)}"}

        def set_nav_mode(mode: str) -> Dict[str, Any]:
            """手动切换导航模式：``slam`` ↔ ``gps``（会停当前目标并复位定位判据）。"""
            try:
                ok, msg = self.ros_node.set_nav_mode(mode)
                return {"success": ok, "nav_mode": self.ros_node.get_nav_mode(), "msg": msg}
            except Exception as e:
                return {"success": False, "msg": f"切换导航模式失败: {str(e)}"}

        def get_route_plan(route_name: str) -> Dict[str, Any]:
            """返回一条路线逐 waypoint 的导航模式标注与段计划（SLAM/GPS 段边界即切换点）。

            仅供调用方组织路线用（只读，不执行）：调用方据此按序对每个 waypoint
            选用相应的单点导航接口（``set_goal_by_*`` / ``set_nav_mode``）并等待到达。
            """
            try:
                return self.get_route_mode_plan(route_name)
            except Exception as e:
                return {"success": False, "msg": f"生成路线模式计划失败: {str(e)}"}

        self._handlers = {
            "set_goal_by_slam": set_goal_by_slam,
            "set_goal_by_outdoor": set_goal_by_outdoor,
            "pause_navigation": pause_navigation,
            "cancel_navigation": cancel_navigation,
            "resume_navigation": resume_navigation,
            "get_pose_list_in_current_submap": get_pose_list_in_current_submap,
            "get_pose_in_submap": get_pose_in_submap,
            "get_submap_list": get_submap_list,
            "get_submap_info": get_submap_info,
            "get_current_map_id": get_current_map_id,
            "change_map": change_map,
            "get_portal_to_submap": get_portal_to_submap,
            "get_route_names": get_route_names,
            "get_default_route": get_default_route,
            "get_route": get_route,
            "viz_set_active_route": viz_set_active_route,
            "viz_clear_active_route": viz_clear_active_route,
            "get_status": get_status,
            "wait_goal_reached": wait_goal_reached,
            "get_navigation_plan": get_navigation_plan,
            "get_nearest_point": get_nearest_point,
            "get_map": get_map,
            "enter_elevator": enter_elevator,
            "exit_elevator": exit_elevator,
            "set_xy_goal_tolerance": set_xy_goal_tolerance,
            "get_nav_mode": get_nav_mode,
            "set_nav_mode": set_nav_mode,
            "get_route_plan": get_route_plan,
        }

    async def call_method(self, method: str, params: Dict[str, Any]) -> Any:
        """调度 WebSocket 请求的单个方法。"""
        handler = self._handlers.get(method)
        if handler is None:
            return {"success": False, "msg": f"未知方法: {method}，可用: {sorted(self._handlers.keys())}"}
        if not isinstance(params, dict):
            params = {}
        try:
            if inspect.iscoroutinefunction(handler):
                return await handler(**params)
            return await asyncio.to_thread(handler, **params)
        except TypeError as e:
            return {"success": False, "msg": f"参数不匹配: {e}"}
        except Exception as e:
            logging.exception("call_method %s", method)
            return {"success": False, "msg": str(e)}

    async def start(self, port=8000):
        """
        启动 FastAPI + WebSocket 服务

        - WebSocket: ws://0.0.0.0:{port}/ws
        - 健康检查: GET http://0.0.0.0:{port}/health
        """
        mgr = self
        app = FastAPI(title="Navigation WebSocket API", version="1.0")

        @app.get("/health")
        def health():
            tf_ready = mgr.ros_node.is_map_localized()
            message = "已定位" if tf_ready else "未定位"
            return {"status": "ok", "service": "NaviSkill", "message": message}

        @app.websocket("/ws")
        async def websocket_nav(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    try:
                        data = await websocket.receive_json()
                    except json.JSONDecodeError as e:
                        await websocket.send_json({"error": f"invalid json: {e}"})
                        continue

                    req_id = data.get("id")
                    method = data.get("method")
                    params = data.get("params")
                    if params is None:
                        params = {}

                    if not method:
                        await websocket.send_json(
                            {"id": req_id, "error": "missing field: method"}
                        )
                        continue

                    result = await mgr.call_method(str(method), params)
                    await websocket.send_json({"id": req_id, "result": result})
            except WebSocketDisconnect:
                logging.info("WebSocket 客户端断开")
            except Exception as e:
                logging.exception("WebSocket 异常: %s", e)
                try:
                    await websocket.send_json({"error": str(e)})
                except Exception:
                    pass

        config = uvicorn.Config(app, host="0.0.0.0", port=port, log_level="warning")
        server = uvicorn.Server(config)
        print(f"FastAPI WebSocket: ws://0.0.0.0:{port}/ws  |  GET /health")

        # 在 uvicorn 同一事件循环里启动 VPR tf_ready 监视协程
        if self.vpr_relocator is not None and self.vpr_relocator.available:
            self._tf_watchdog_task = asyncio.create_task(
                self._tf_localization_watchdog(),
                name="vpr-tf-watchdog",
            )

        try:
            await server.serve()
        finally:
            self._shutdown = True
            if self._tf_watchdog_task is not None:
                self._tf_watchdog_task.cancel()
                try:
                    await self._tf_watchdog_task
                except (asyncio.CancelledError, Exception):
                    pass
                self._tf_watchdog_task = None

    def cleanup(self):
        """
        清理资源
        
        功能：
        1. 关闭ROS执行器
        2. 销毁ROS节点
        3. 关闭ROS客户端库
        """
        """清理资源"""
        self._shutdown = True
        try:
            # 关闭 VPR 子进程
            if getattr(self, "vpr_bootstrap", None) is not None:
                try:
                    self.vpr_bootstrap.stop()
                except Exception as e:
                    logging.error(f"关闭 VPR 子进程失败: {e}")
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
def _launch_nav_stack(args, map_manager) -> None:
    """
    依据 -s/--submap 决定导航模式并拉起对应导航栈（仅负责"首次拉起"）：

      * 指定 -s slam_map_id/submap_id → SLAM 室内栈：
        解析该 submap 的 pcd 路径后 ``start_slam.sh -m <pcd>``；
      * 未指定 -s → 室外 GPS 栈：``start_outdoor.sh``。

    仿真时钟：读环境变量 ``$NAVI_SIM_ARG``，非空则给启动脚本追加 ``-s``（--simulated）。
    server.py 运行在 tmux pane 内，子进程需 unset ``$TMUX`` 才能让脚本里的 tmuxp 正常新建会话。
    """
    import subprocess
    scripts_dir = _REPO_ROOT / "scripts"
    child_env = os.environ.copy()
    # 去掉 TMUX，避免脚本里的 tmuxp 因"嵌套在当前会话内"而拒绝新建会话。
    child_env.pop('TMUX', None)
    child_env.pop('TMUX_PANE', None)
    sim = child_env.get('NAVI_SIM_ARG', '').strip()
    sim_args = ['-s'] if sim else []

    if args.submap:
        slam_map_id, submap_id = args.submap.split('/', 1)
        if map_manager is None:
            logging.error("指定了 -s %s 但地图未加载成功，无法拉起 SLAM 栈", args.submap)
            return
        pcd = map_manager.get_submap_full_map_path(submap_id, slam_map_id)
        if not pcd:
            logging.error("无法解析 slam_map/submap %s/%s 的 pcd 路径，跳过拉起 SLAM 栈",
                          slam_map_id, submap_id)
            return
        cmd = ['bash', str(scripts_dir / 'start_slam.sh'), '-m', pcd] + sim_args
        logging.warning("导航模式=SLAM，拉起: %s", ' '.join(cmd))
    else:
        cmd = ['bash', str(scripts_dir / 'start_outdoor.sh')] + sim_args
        logging.warning("导航模式=室外GPS，拉起: %s", ' '.join(cmd))

    try:
        # 启动脚本内部用 ``tmuxp load -d`` 建会话后即返回，这里非阻塞拉起即可。
        subprocess.Popen(cmd, env=child_env)
    except Exception as e:
        logging.error("拉起导航栈失败: %s", e)


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
                        help='多楼层地图来源；可填两种形式：'
                             '1) 本地路径，如 /home/nvidia/Navi/maps/408/408.json；'
                             '2) mapserver URL，如 http://10.239.18.181:8080/mapserver/408 '
                             '（首次启动会下载到 <repo>/maps/<map_id>/，之后做增量同步；'
                             '网络失败但本地有缓存时降级到本地）')
    parser.add_argument('-s', '--submap', type=str, default=None,
                        help='初始定位目标，格式严格为 slam_map_id/submap_id（例如 caihong/2f；'
                             '单文件地图的 slam_map_id 为 default，如 default/2f）。'
                             '给出 -s 即进入 SLAM 室内模式：server.py 会解析该 submap 的 pcd 并'
                             '拉起 start_slam.sh -m <pcd>；不给 -s 则进入室外 GPS 模式，拉起 start_outdoor.sh')
    parser.add_argument('--image_topic', type=str, default=None,
                        help='VPR 引导/重定位使用的相机 CompressedImage 话题；'
                             '省略则使用 NavNode 默认 '
                             '(/hdas/camera_head/left_raw/image_raw_color/compressed)')
    parser.add_argument('--vpr_device', type=str, default='cuda', choices=['cuda', 'cpu'],
                        help='VPR 子进程使用的设备')
    parser.add_argument('-v', '--viz', action='store_true',
                        help='启动 rerun 可视化（依赖 rerun-sdk）。展示：'
                             '多 submap 点云对齐总图、机器人位姿、实时雷达点云、'
                             '所有 routes、栅格地图 /map 与 /plan')
    parser.add_argument('--viz_lidar_topic', type=str, default='/cloud_registered',
                        help='可视化使用的 LiDAR 话题；默认 /cloud_registered（FAST_LIO '
                             '已去畸变+world 系输出，直接 log 不做外参变换）；'
                             '也可设为 /livox/lidar（原始 sensor 系，需要 lidar→base_link 外参）')
    parser.add_argument('--viz_mode', type=str, default='spawn',
                        choices=['spawn', 'connect', 'serve', 'save'],
                        help='rerun viewer 连接方式：'
                             'spawn=本机起 viewer 子进程（需 X11/Wayland）；'
                             'connect=连接已开的 viewer (--viz_addr 指定)；'
                             'serve=不画窗口，启 SDK TCP server 等远端连接 (--viz_addr 指定监听 bind)；'
                             'save=不联网，全部写入 .rrd 文件 (--viz_save_path)')
    parser.add_argument('--viz_addr', type=str, default=None,
                        help='viz_mode=connect 时 viewer 地址 (默认 127.0.0.1:9876)；'
                             'viz_mode=serve 时 bind 地址 (默认 0.0.0.0:9876)')
    parser.add_argument('--viz_save_path', type=str, default=None,
                        help='viz_mode=save 时 .rrd 文件保存路径，默认 /tmp/navi_rerun.rrd')
    parser.add_argument('--viz_no_spawn', action='store_true',
                        help='(已废弃，等价于 --viz_mode serve) 不 spawn viewer，仅起 TCP server')
    args = parser.parse_args()

    # -s 严格要求 slam_map_id/submap_id 形式（区分"哪张 SLAM 地图的哪个 submap"）。
    if args.submap is not None and '/' not in args.submap:
        parser.error("-s/--submap 需为 slam_map_id/submap_id 格式（例如 caihong/2f 或 default/2f）")

    # 兼容旧参数
    if args.viz_no_spawn and args.viz_mode == 'spawn':
        args.viz_mode = 'serve'

    port = 9999 if args.test else 9000

    # 初始化地图管理器
    map_manager = None
    if args.map:
        try:
            # 解析 -m：可能是本地路径，也可能是 mapserver URL。
            # URL 情况下 resolve_map_source 会负责"本地缓存 / 增量同步 / 网络降级"，
            # 并把最终的本地 <map_id>.json 路径返回给 MapManager。
            cache_root = _REPO_ROOT / "mapdata"
            map_json_path = resolve_map_source(args.map, cache_root=cache_root)
            map_manager = MapManager(str(map_json_path))
            logging.info(f"已加载多楼层地图: {map_json_path}  (来源: {args.map})")
            if args.submap:
                # 已在 parse 阶段校验含 '/'；先切 slam_map（会把 submap 复位到该图默认），
                # 再切到目标 submap。
                slam_map_id, submap_id = args.submap.split('/', 1)
                if not map_manager.set_current_slam_map(slam_map_id):
                    logging.error(f"指定的 slam_map '{slam_map_id}' 不存在，可用: {map_manager.get_slam_map_list()}")
                elif not map_manager.set_current_submap(submap_id):
                    logging.error(f"指定的 submap '{submap_id}' 不存在，可用: {map_manager.get_submap_list(slam_map_id)}")
                else:
                    logging.info(f"已指定初始 slam_map/submap: {slam_map_id}/{submap_id}")
        except Exception as e:
            logging.error(f"加载地图配置失败: {e}")

    # 依据 -s 决定导航模式并拉起对应导航栈（"首次拉起"统一由 server.py 负责）。
    _launch_nav_stack(args, map_manager)

    mgr = None
    viz = None
    try:
        # 创建导航工具管理器
        mgr = NavigationToolManager(
            map_manager=map_manager,
            submap_specified=args.submap is not None,
            vpr_device=args.vpr_device,
            camera_image_topic=args.image_topic,
        )

        # 可选：启动 rerun 可视化（-v）
        if args.viz:
            try:
                from visualization import NaviRerunViz
                # bind / connect 地址默认值依赖 mode
                if args.viz_addr is None:
                    addr_default = "0.0.0.0:9876" if args.viz_mode == "serve" else "127.0.0.1:9876"
                else:
                    addr_default = args.viz_addr
                viz = NaviRerunViz(
                    nav_node=mgr.ros_node,
                    map_manager=mgr.map_manager,
                    lidar_topic=args.viz_lidar_topic,
                    mode=args.viz_mode,
                    connect_addr=addr_default,
                    serve_addr=addr_default,
                    save_path=args.viz_save_path,
                )
                viz.start()
                # 注入给 mgr，让 viz_set_active_route / viz_clear_active_route 能用
                mgr.viz = viz
            except Exception as e:
                logging.error(f"启动可视化失败（继续运行 server）: {e}", exc_info=True)
                viz = None

        # 启动 FastAPI WebSocket 服务
        await mgr.start(port=port)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断信号
        logging.info("收到中断信号，正在关闭...")
    except Exception as e:
        # 处理其他异常
        logging.error(f"程序运行出错: {e}")
    finally:
        # 确保资源被正确清理
        if viz:
            try:
                viz.stop()
            except Exception:
                pass
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
    
