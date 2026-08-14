# 导入必要的库
import time  # 时间处理库
import math  # 数学库（传送门经纬度距离/位姿换算）
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
_SLAM_HEADING_HANDOFF = Path('/tmp/navi_slam_heading_handoff.json')
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
        nav_mode: str = NavNode.NAV_MODE_SLAM,
        vpr_device: str = "cuda",
        camera_image_topic: Optional[str] = None,
    ):
        """
        初始化导航工具管理器

        :param map_manager: 多楼层地图管理器实例，可选
        :param submap_specified: 用户是否在命令行显式指定了初始 submap；为 False
                                  时若存在 VPR 数据库会用相机图像引导确定 submap
        :param nav_mode: 初始导航模式（``slam`` / ``gps``）。室外 GPS 模式下不做
                          初始 submap 识别、不与 ROS 端点云地图对齐（不发起换图）。
        :param vpr_device: VPR 子进程使用的设备（``cuda`` / ``cpu``）
        :param camera_image_topic: 用于 VPR 引导/重定位的 CompressedImage 话题；
                                    None 时使用 NavNode 默认值
        """
        # 方法名 -> 可调用（同步或 async），由 register_tools 填充
        self._handlers = {}
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
            # 仿真时钟：按 NAVI_SIM_ARG（即启动脚本的 -s）打开 use_sim_time，
            # 否则仿真下节点用墙上时钟、TF 用 /clock 仿真时间戳，两者相差几十年会导致
            # TF 新鲜度恒定超标、current_pose 永远刷不出。
            use_sim_time = bool(os.environ.get('NAVI_SIM_ARG', '').strip())
            if camera_image_topic:
                self.ros_node = NavNode(camera_image_topic=camera_image_topic,
                                        use_sim_time=use_sim_time,
                                        initial_nav_mode=nav_mode)
            else:
                self.ros_node = NavNode(use_sim_time=use_sim_time,
                                        initial_nav_mode=nav_mode)
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

            # 仅 SLAM 室内模式才做"初始 submap 识别 + 与 ROS 端点云地图对齐"。
            # 室外 GPS 模式无点云地图定位：既不设置/引导 submap，也不发起换图，
            # 否则会把 MapManager 缓存的 submap（如 caihong/2f）误当作目标发一次 change_map。
            if nav_mode == NavNode.NAV_MODE_SLAM:
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
            else:
                logging.info("室外 GPS 模式：跳过初始 submap 识别与地图切换（不发起 change_map）")
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
    # 一条路线的 waypoint 统一由 ``type`` 唯一确定所需导航模式：
    #   - ``slam`` → SLAM 室内点云定位（某 SLAM 地图 submap 内的语义点）
    #   - ``gps``  → 室外 GNSS（场景 scene_point / 经纬度点）
    # 执行时客户端按 ``type`` 顺序调用对应单点接口（``set_goal_by_slam`` /
    # ``set_goal_by_outdoor``），服务端在下发目标时按需自动切模式并走传送门
    # （见 ``_ensure_nav_mode`` / ``_ensure_submap``），无需提前规划切换段。
    # ══════════════════════════════════════════════════════════════════════

    def _current_submap_id(self) -> Optional[str]:
        """当前活跃 submap id：以 ROS 端实际加载的地图为准，回退 MapManager 缓存。

        对应原 test_client.py ``ensure_submap`` 里的 ``get_current_map_id``。
        """
        try:
            map_path = self.ros_node.get_current_map_path()
            if map_path and self.map_manager:
                rid = self.map_manager.get_submap_id_by_map_file(map_path)
                if rid:
                    return rid
        except Exception:
            pass
        return self.map_manager.get_current_submap() if self.map_manager else None

    def _ensure_submap(self, target_submap: str,
                       portal_timeout: float = 120.0,
                       portal_tol: float = 0.5) -> Dict[str, Any]:
        """确保 ROS 端加载的 submap 为 ``target_submap``；已一致则空操作。

        参考 test_client.py ``start_patrol_test`` 的跨子图流程：**submap 不一致时
        不能直接 ``_change_map``**，必须先在当前 submap 内导航到「当前 submap → 目标
        submap」的传送门，到达后再换图（换图会做跨图位姿衔接与 ``/pose_in_target``）。

        步骤：
        1. 当前 submap == 目标 → 直接成功；
        2. 校验目标 submap 存在；
        3. ``get_portal_to_submap`` 取直达传送门（多条时按当前位姿选最近）；
        4. ``publish_goal`` 导航到传送门语义点并阻塞等待到达；
        5. 到达后 ``_change_map(target_submap)`` 完成换图。

        :return: ``{success, msg, ...}``；传送门缺失/导航失败/换图失败均返回
            ``success=False``，此时**未换图**（保持当前 submap）。
        """
        if not self.map_manager:
            return {"success": False, "msg": "MapManager 未配置"}

        current_submap = self._current_submap_id()
        if current_submap == target_submap:
            return {"success": True, "changed": False, "submap": target_submap,
                    "msg": f"当前已在目标 submap {target_submap}"}

        submap_list = self.map_manager.get_submap_list()
        if target_submap not in submap_list:
            return {"success": False,
                    "msg": f"目标 submap {target_submap} 不存在，可用: {submap_list}"}

        # 3) 取「当前 submap → 目标 submap」直达传送门（多条按当前位姿选最近）
        robot_xyz = None
        cp = self.ros_node.current_pose
        if cp is not None:
            try:
                p = cp.pose.position
                robot_xyz = (float(p.x), float(p.y), float(p.z))
            except Exception:
                robot_xyz = None
        portal_config = self.map_manager.get_portal_to_submap(
            target_submap, from_submap=current_submap,
            robot_position=robot_xyz, xy_distance_only=True)
        if not portal_config:
            return {"success": False,
                    "msg": f"从 {current_submap} 无直达传送门到 {target_submap}，无法换图"}
        portal_name = portal_config.get('portal')
        portal_info = self.map_manager.get_portal_pose(portal_name, current_submap)
        portal_pose = (portal_info or {}).get('pose') if isinstance(portal_info, dict) else None
        if not portal_pose or not portal_pose.get('position'):
            return {"success": False,
                    "msg": f"传送门 {portal_name} 缺少位姿，无法导航换图"}

        # 4) 导航到传送门语义点并等待到达（传送门为过渡点，放宽 yaw）
        pos = portal_pose['position']
        ori = portal_pose.get('orientation') or {}
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(pos.get('x', 0.0))
        pose.pose.position.y = float(pos.get('y', 0.0))
        pose.pose.position.z = float(pos.get('z', 0.0))
        pose.pose.orientation.x = float(ori.get('x', 0.0))
        pose.pose.orientation.y = float(ori.get('y', 0.0))
        pose.pose.orientation.z = float(ori.get('z', 0.0))
        pose.pose.orientation.w = float(ori.get('w', 1.0))
        self.ros_node.publish_goal(pose, f"portal:{portal_name}")
        logging.info("换图前导航到传送门 %s(%s→%s) x=%.2f y=%.2f，等待到达…",
                     portal_name, current_submap, target_submap,
                     pose.pose.position.x, pose.pose.position.y)
        start = time.time()
        reached = False
        while time.time() - start < portal_timeout:
            state, fmsg = self.ros_node.check_goal_reached(
                tolerance=portal_tol, yaw_tolerance=math.pi)
            if state == NavNode.GOAL_CHECK_REACHED:
                reached = True
                break
            if state == NavNode.GOAL_CHECK_FAILED:
                return {"success": False, "portal": portal_name,
                        "msg": f"导航到传送门 {portal_name} 失败: {fmsg}"}
            time.sleep(0.5)
        if not reached:
            return {"success": False, "portal": portal_name,
                    "msg": f"导航到传送门 {portal_name} 超时"}

        # 5) 到达传送门后换图（跨图位姿衔接 + /pose_in_target 在 _change_map 内完成）
        logging.info("已到达传送门 %s，开始换图 → %s", portal_name, target_submap)
        ctx = self._change_map(target_submap)
        if not ctx.get("success"):
            return {"success": False, "portal": portal_name, "change_map": ctx,
                    "msg": f"到达传送门后换图失败: {ctx.get('msg')}"}
        return {"success": True, "changed": True, "portal": portal_name,
                "submap": target_submap, "change_map": ctx,
                "msg": f"已导航到传送门 {portal_name} 并换图到 {target_submap}"}

    def _spawn_nav_script(self, cmd: list) -> bool:
        """非阻塞拉起导航栈脚本（start_slam.sh / start_outdoor.sh）。

        server.py 运行在 tmux pane 内，子进程需 unset ``$TMUX`` 才能让脚本里的
        tmuxp 正常新建会话（否则会因"嵌套在当前会话内"被拒）。脚本内部 ``tmuxp
        load -d`` 建会话后即返回，这里非阻塞拉起即可。
        """
        import subprocess
        child_env = os.environ.copy()
        child_env.pop('TMUX', None)
        child_env.pop('TMUX_PANE', None)
        try:
            subprocess.Popen(cmd, env=child_env)
            return True
        except Exception as e:
            logging.error("拉起导航栈失败: %s", e)
            return False

    @staticmethod
    def _sim_args() -> list:
        """仿真时钟：读环境变量 ``$NAVI_SIM_ARG``，非空则给启动脚本追加 ``-s``。"""
        sim = os.environ.get('NAVI_SIM_ARG', '').strip()
        return ['-s'] if sim else []

    def _launch_slam_stack(self) -> Dict[str, Any]:
        """按 map_manager 当前 slam_map/submap 解析 pcd 并拉起 ``start_slam.sh -m <pcd>``。

        调用方应在切换前把 map_manager 的当前 slam_map/submap 设为目标值，
        使新拉起的 SLAM 栈直接加载正确 pcd（无需再 change_map）。
        """
        scripts_dir = _REPO_ROOT / "scripts"
        if self.map_manager is None:
            return {"launched": False, "msg": "未加载地图，无法拉起 SLAM 栈"}
        slam_map_id = self.map_manager.get_current_slam_map()
        submap_id = self.map_manager.get_current_submap()
        pcd = self.map_manager.get_submap_full_map_path(submap_id, slam_map_id)
        if not pcd:
            return {"launched": False,
                    "msg": f"无法解析 slam_map/submap {slam_map_id}/{submap_id} 的 pcd 路径"}
        cmd = ['bash', str(scripts_dir / 'start_slam.sh'), '-m', pcd] + self._sim_args()
        logging.warning("切到 SLAM 模式 (%s/%s)，拉起: %s",
                        slam_map_id, submap_id, ' '.join(cmd))
        ok = self._spawn_nav_script(cmd)
        return {"launched": ok, "slam_map": slam_map_id, "submap": submap_id,
                "pcd": pcd, "cmd": ' '.join(cmd)}

    def _launch_outdoor_stack(self) -> Dict[str, Any]:
        """拉起室外 GPS 栈 ``start_outdoor.sh``。"""
        scripts_dir = _REPO_ROOT / "scripts"
        cmd = ['bash', str(scripts_dir / 'start_outdoor.sh')] + self._sim_args()
        logging.warning("切到室外 GPS 模式，拉起: %s", ' '.join(cmd))
        ok = self._spawn_nav_script(cmd)
        return {"launched": ok, "cmd": ' '.join(cmd)}

    def _write_slam_heading_handoff(self, slam_map_id: str) -> Dict[str, Any]:
        """把切栈前最后 SLAM 朝向换算成 ENU yaw，供新 outdoor 栈粗对齐。

        ``current_pose`` 位于当前 submap 局部系；submap 的级联 rotation 将它
        转到 SLAM 根系，geo_anchor.heading_deg 再把根系转到 ENU。文件由
        gps_heading_align 启动时限时、一次性消费，不能被当成真正 GPS 锁定。
        """
        cp = self.ros_node.current_pose
        submap_id = self.map_manager.get_current_submap()
        anchor = self.map_manager.get_slam_map_geo_anchor(slam_map_id) or {}
        if cp is None or not submap_id or 'heading_deg' not in anchor:
            return {"written": False, "msg": "缺少最后 SLAM 位姿/submap/geo_anchor heading"}

        q_local = {
            'x': float(cp.pose.orientation.x),
            'y': float(cp.pose.orientation.y),
            'z': float(cp.pose.orientation.z),
            'w': float(cp.pose.orientation.w),
        }
        tf = self.map_manager.get_submap_transform(submap_id, slam_map_id)
        q_root = (self.map_manager._quaternion_multiply(tf['rotation'], q_local)
                  if tf else q_local)
        root_yaw = math.atan2(
            2.0 * (q_root['w'] * q_root['z'] + q_root['x'] * q_root['y']),
            1.0 - 2.0 * (q_root['y'] ** 2 + q_root['z'] ** 2),
        )
        enu_yaw = math.atan2(
            math.sin(root_yaw + math.radians(float(anchor['heading_deg']))),
            math.cos(root_yaw + math.radians(float(anchor['heading_deg']))),
        )
        payload = {
            'version': 1,
            'created_at': time.time(),
            'slam_map': slam_map_id,
            'submap': submap_id,
            'slam_root_yaw_rad': root_yaw,
            'geo_heading_deg': float(anchor['heading_deg']),
            'absolute_enu_yaw_rad': enu_yaw,
        }
        tmp = _SLAM_HEADING_HANDOFF.with_suffix('.tmp')
        try:
            tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
            os.replace(tmp, _SLAM_HEADING_HANDOFF)
        except OSError as exc:
            logging.warning("写入 SLAM→ENU 航向交接失败: %s", exc)
            return {"written": False, "msg": str(exc)}
        logging.info(
            "SLAM→outdoor 粗航向: map=%s submap=%s root=%.1f° anchor=%.1f° ENU=%.1f°",
            slam_map_id, submap_id, math.degrees(root_yaw),
            float(anchor['heading_deg']), math.degrees(enu_yaw))
        return {"written": True, **payload}

    @staticmethod
    def _lonlat_dist_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
        """两经纬度间的近似平面距离（米），仅用于比较远近选最近传送门。"""
        R = 6378137.0
        dn = math.radians(lat2 - lat1) * R
        de = math.radians(lon2 - lon1) * R * math.cos(math.radians((lat1 + lat2) / 2.0))
        return math.hypot(de, dn)

    def _nearest_scene_portal(self, slam_map_id: str,
                              ref_lon: float, ref_lat: float):
        """在通往 ``slam_map_id`` 的场景传送门中，按经纬度距离选最近的一个。

        场景传送门是连接「全局(OSM/室外)地图」与某张 SLAM 地图的进出衔接点
        （scene ``portals`` 中 ``target_slam_map==slam_map_id`` 的场景语义点）。

        :return: ``(portal_name, scene_point_dict)``；无可用传送门返回 ``(None, None)``。
        """
        best = (None, None)
        best_d = float('inf')
        for name in self.map_manager.find_scene_portals_to_slam_map(slam_map_id):
            pt = self.map_manager.get_scene_point(name)
            if not pt or pt.get('lon') is None or pt.get('lat') is None:
                continue
            d = self._lonlat_dist_m(ref_lon, ref_lat, pt['lon'], pt['lat'])
            if d < best_d:
                best_d, best = d, (name, pt)
        if best[0]:
            logging.info("最近场景传送门(→%s): %s (%.1fm)", slam_map_id, best[0], best_d)
        return best

    def _navigate_to_portal_outdoor(self, portal_name: str, pt: Dict[str, Any],
                                    timeout: float, tol: float) -> Dict[str, Any]:
        """室外 GPS 模式下导航到传送门（经纬度）并阻塞等待到达。"""
        res = self.ros_node.set_outdoor_goal(
            float(pt['lon']), float(pt['lat']), target=portal_name, tolerance=float(tol))
        if not res.get("ok"):
            return {"success": False, "msg": f"下发室外传送门目标失败: {portal_name}"}
        logging.info("室外导航到传送门 %s (%.7f,%.7f)，等待到达…",
                     portal_name, pt['lon'], pt['lat'])
        start = time.time()
        while time.time() - start < timeout:
            if self.ros_node.is_outdoor_goal_reached(tol):
                return {"success": True, "portal": portal_name}
            time.sleep(1.0)
        return {"success": False, "msg": f"导航到室外传送门超时: {portal_name}"}

    def _navigate_to_portal_slam(self, portal_name: str, pt: Dict[str, Any],
                                 slam_map_id: str, timeout: float,
                                 tol: float) -> Dict[str, Any]:
        """SLAM 模式下导航到传送门（换算为当前 submap 局部位姿）并阻塞等待到达。"""
        # 传送门经纬度 → SLAM 地图根系位姿（geo_anchor 反算）
        root_pose = self.map_manager.lonlat_to_slam_root_pose(
            pt.get('lon'), pt.get('lat'), pt.get('heading_deg'), slam_map_id)
        if not root_pose:
            return {"success": False,
                    "msg": f"无法解析传送门 {portal_name} 的 SLAM 位姿（缺 geo_anchor）"}
        # 根系 → 当前 submap 局部（publish_goal 使用当前 submap 的 "odom" 系）
        cur_submap = self.map_manager.get_current_submap()
        tf = self.map_manager.get_submap_transform(cur_submap, slam_map_id)
        pos = root_pose['position']
        ori = root_pose['orientation']
        if tf:
            pos = self.map_manager._apply_inverse_transform(
                pos, tf['translation'], tf['rotation'])
            ori = self.map_manager._quaternion_multiply(
                self.map_manager._quaternion_inverse(tf['rotation']), ori)
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = pos['x']
        pose.pose.position.y = pos['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = ori['x']
        pose.pose.orientation.y = ori['y']
        pose.pose.orientation.z = ori['z']
        pose.pose.orientation.w = ori['w']
        self.ros_node.publish_goal(pose, f"portal:{portal_name}")
        logging.info("SLAM 导航到传送门 %s (submap=%s, x=%.2f y=%.2f)，等待到达…",
                     portal_name, cur_submap, pos['x'], pos['y'])
        start = time.time()
        while time.time() - start < timeout:
            # 传送门只是过渡点，朝向不重要：放宽 yaw 容差，仅按距离判定到达。
            state, fmsg = self.ros_node.check_goal_reached(
                tolerance=tol, yaw_tolerance=math.pi)
            if state == NavNode.GOAL_CHECK_REACHED:
                return {"success": True, "portal": portal_name}
            if state == NavNode.GOAL_CHECK_FAILED:
                return {"success": False,
                        "msg": f"导航到 SLAM 传送门失败: {portal_name} ({fmsg})"}
            time.sleep(0.5)
        return {"success": False, "msg": f"导航到 SLAM 传送门超时: {portal_name}"}

    def _wait_localized(self, target_mode: str, timeout: float,
                        poll_interval: float) -> bool:
        """阻塞等待 ``ros_node.is_localized()`` 就绪（新栈定位建立）。"""
        start = time.time()
        while time.time() - start < timeout:
            if self.ros_node.is_localized():
                logging.info("_ensure_nav_mode: %s 模式定位就绪 (elapsed=%.1fs)",
                             target_mode, time.time() - start)
                return True
            time.sleep(poll_interval)
        logging.warning("_ensure_nav_mode: 等待 %s 模式定位就绪超时(%.0fs)",
                        target_mode, timeout)
        return False

    def _ensure_nav_mode(self, target_mode: str,
                         localize_timeout: float = 180.0,
                         poll_interval: float = 1.0,
                         portal_timeout: float = 300.0,
                         portal_tol: float = 1.0) -> Dict[str, Any]:
        """确保 NavNode 处于目标导航模式；已在目标模式则空操作。

        供 set_goal 系列在下发前对齐模式（SLAM 语义点/坐标 → slam；经纬度 → gps）。

        **模式不一致时不能直接切换**，必须先在当前模式下把机器人开到该 SLAM 地图
        「进出全局地图」的最近场景传送门，到达后再重启目标模式的导航栈：

        - **室外 GPS → SLAM**：以当前 GNSS fix 为参考选最近传送门，室外路网导航过去，
          到达后 ``start_slam.sh`` 拉起 SLAM 栈（加载目标 slam_map/submap 的 pcd）。
        - **SLAM → 室外 GPS**：以机器人当前 SLAM 位置为参考选最近传送门，SLAM 导航过去，
          到达后 ``start_outdoor.sh`` 拉起室外栈。

        切到 SLAM 前，调用方应已把 map_manager 的当前 slam_map/submap 设为目标值。
        重启导航栈后会阻塞等待新栈定位就绪（``ros_node.is_localized()``）再返回。
        传送门导航失败时不切换（``nav_mode`` 保持原样）并带 ``msg``。
        """
        cur = self.ros_node.get_nav_mode()
        if cur == target_mode:
            return {"switched": False, "nav_mode": cur, "localized": True}

        if self.map_manager is None:
            return {"switched": False, "nav_mode": cur, "localized": False,
                    "msg": "未加载地图，无法执行跨模式传送门切换"}

        # 待切换所涉及的 SLAM 地图：
        # - 室外→SLAM：目标 slam_map（调用方已 set_current_slam_map 到目标）；
        # - SLAM→室外：当前所在 slam_map。
        slam_map_id = self.map_manager.get_current_slam_map()
        if not slam_map_id:
            return {"switched": False, "nav_mode": cur, "localized": False,
                    "msg": "未确定 SLAM 地图，无法选择进出全局地图的传送门"}

        # ── 1) 当前模式下先导航到最近传送门 ──
        if target_mode == NavNode.NAV_MODE_SLAM:
            fix = self.ros_node.get_latest_gps_fix()
            if not fix or fix.get('lon') is None or fix.get('lat') is None:
                return {"switched": False, "nav_mode": cur, "localized": False,
                        "msg": "无 GNSS fix，无法选择通往 SLAM 的室外传送门"}
            name, pt = self._nearest_scene_portal(slam_map_id, fix['lon'], fix['lat'])
            if not name:
                return {"switched": False, "nav_mode": cur, "localized": False,
                        "msg": f"未找到通往 SLAM 地图 {slam_map_id} 的场景传送门"}
            portal_nav = self._navigate_to_portal_outdoor(
                name, pt, portal_timeout, portal_tol)
        else:
            # 参考点：机器人当前 SLAM 位置换算成经纬度（回退 geo_anchor 原点）
            ref_lon = ref_lat = None
            cp = self.ros_node.current_pose
            cur_submap = self.map_manager.get_current_submap()
            if cp is not None and cur_submap:
                ll = self.map_manager.slam_point_to_lonlat(
                    cur_submap, cp.pose.position.x, cp.pose.position.y,
                    slam_map_id=slam_map_id)
                if ll:
                    ref_lon, ref_lat = ll
            if ref_lon is None:
                anchor = self.map_manager.get_slam_map_geo_anchor(slam_map_id) or {}
                origin = anchor.get('origin', {}) or {}
                ref_lon, ref_lat = origin.get('lon'), origin.get('lat')
            if ref_lon is None:
                return {"switched": False, "nav_mode": cur, "localized": False,
                        "msg": "无法确定当前 SLAM 位置，无法选择传送门"}
            name, pt = self._nearest_scene_portal(slam_map_id, ref_lon, ref_lat)
            if not name:
                return {"switched": False, "nav_mode": cur, "localized": False,
                        "msg": f"未找到 SLAM 地图 {slam_map_id} 通往全局地图的传送门"}
            portal_nav = self._navigate_to_portal_slam(
                name, pt, slam_map_id, portal_timeout, portal_tol)

        if not portal_nav.get('success'):
            # 传送门未到达：不切换，保持原模式。
            return {"switched": False, "nav_mode": self.ros_node.get_nav_mode(),
                    "localized": False, "portal": name, "portal_nav": portal_nav,
                    "msg": portal_nav.get('msg', "传送门导航失败，未切换模式")}

        # ── 2) 到达传送门后重启目标模式导航栈 ──
        # 传送门“到达”是几何判定（进入 tol 半径即返回），机器人此刻可能仍在移动。
        # 拆栈前必须先停车，否则切换期间会溜车：cancel_navigation 只能取消本节点持有
        # handle 的目标；campus_nav 近距也可能直发 /goal_pose，nav_node 无其 handle，
        # 故还需 cancel_all_nav_goals 走 action 底层 CancelGoal 服务取消全部目标。
        self.ros_node.cancel_navigation()
        self.ros_node.cancel_all_nav_goals()
        # 取消只是让 Nav2 **开始**减速：零速要经 controller -> velocity_smoother
        # -> collision_monitor 三跳才到底盘，且 velocity_smoother 按 max_decel 渐变。
        # 而下面 _launch_*_stack 会 tmux kill-session 掉当前栈，谁在减速途中拆栈，
        # 最后一条 /cmd_vel 就是非零值，底盘保持它继续溜车。所以这里盯底盘反馈，
        # 确认真的停了再拆。
        stop = self.ros_node.wait_until_stopped()
        if stop.get('stopped') is False:
            logging.warning("切换模式前未能确认停车: %s", stop)
        heading_handoff = None
        if target_mode == NavNode.NAV_MODE_GPS:
            # 必须在 start_outdoor.sh 拆掉 SLAM 栈之前捕获最后姿态。
            heading_handoff = self._write_slam_heading_handoff(slam_map_id)
        if target_mode == NavNode.NAV_MODE_SLAM:
            launch = self._launch_slam_stack()
        else:
            launch = self._launch_outdoor_stack()
        ok, msg = self.ros_node.set_nav_mode(target_mode)

        # ── 3) 阻塞等待新栈定位就绪 ──
        localized = self._wait_localized(target_mode, localize_timeout, poll_interval)

        return {
            "switched": bool(ok), "from": cur,
            "nav_mode": self.ros_node.get_nav_mode(),
            "portal": name, "portal_nav": portal_nav,
            "launch": launch, "stop": stop, "localized": localized, "msg": msg,
            "heading_handoff": heading_handoff,
        }

    def register_tools(self):
        """
        注册 WebSocket RPC 方法（与原先 MCP 工具一一对应）

        客户端 JSON: {\"id\": optional, \"method\": 方法名, \"params\": {...}}
        服务端回复: {\"id\": ..., \"result\": ...} 或 {\"id\": ..., \"error\": \"...\"}
        """
        def set_goal_by_slam(target: str, x: float = None, y: float = None,
                             slam_map: str = None, submap: str = None,
                             xy_tolerance: float = 0.1,
                             yaw_tolerance: float = 0.1) -> Dict[str, Any]:
            """
            下发导航目标（SLAM 模式）。两种用法：

            1. **命名语义点**（``target`` 为地图预设点名，如 狗巢/配电房/灭火器箱）：
               从 MapManager 取该点位姿下发，支持 ``slam_map``/``submap`` 跨图切换与
               ``elevator_*`` 类型处理。
            2. **临时坐标点**（``target == "tmp"``）：不查 MapManager，直接以入参 ``x``/``y``
               为目标；朝向取「当前位置指向目标」的接近方向。到达精度由
               ``xy_tolerance``/``yaw_tolerance`` 控制。

            参数:
            target: 目标点名称；传 ``"tmp"`` 表示使用 ``x``/``y`` 临时坐标。
            x, y: 当 ``target == "tmp"`` 时必填，地图(odom)系坐标。
            slam_map: 可选（仅命名点位），目标所在 SLAM 地图 id；与当前不同会自动切换。
            submap: 可选（仅命名点位），目标所在 submap id；与当前不同会自动换图。
            xy_tolerance: 到达判定的位置容差（米），默认 0.1；下发时随目标记录，供
                          `wait_goal_reached` 取回使用（wait 时不再传容差）。
            yaw_tolerance: 到达判定的朝向容差（弧度），默认 0.1；含义同上。
                           注：``elevator_in``/``LowPrecision_pose`` 类点位会在此自动放宽。

            说明（命名语义点）:
            - 语义点属于 SLAM 导航：本接口会自动确保处于 `slam` 模式（若当前为 `gps` 室外模式则切回）。
            - 适用于 `type` 为 `single_pose` 的点位；调用此接口会使用普通导航下发目标。
            - 如果点位 `type` 为 `elevator_call`：机器人到达该点后应等待 agent 执行呼叫电梯的操作（本接口只负责下发导航）。
            - 如果点位 `type` 为 `elevator_wait`：机器人到达该点后应等待 agent 确认电梯已经到达并开门。
            - 如果点位 `type` 为 `elevator_in`：下发目标时自动锁定 Z 轴（内部 `set_elevator_mode`），在前往电梯途中即生效，无需外部调用。
            - 如果点位 `type` 为 `elevator_out`：出梯后随即换图，Z 锁状态会被重置，无需单独解锁。

            返回:
            Dict: 包含操作结果的字典（含 ``xy_tolerance``/``yaw_tolerance`` 实际生效值）
            """
            try:
                cur_mode = self.ros_node.get_nav_mode()

                # ── target == "tmp"：临时坐标点，不查 MapManager，直接用 x/y ──
                if target == "tmp":
                    # 均属 SLAM：确保处于 slam 模式（室外→室内需先导航到传送门再切换栈）
                    mode_switch = self._ensure_nav_mode(NavNode.NAV_MODE_SLAM)
                    if mode_switch.get("nav_mode") != NavNode.NAV_MODE_SLAM:
                        return {"success": False,
                                "msg": mode_switch.get("msg", "切换到 SLAM 模式失败"),
                                "mode_switch": mode_switch}
                    # 定位未就绪就下发会把目标发给尚未启动完成的新栈而丢失，
                    # 必须等 _ensure_nav_mode 确认新栈 map→base_link 稳定后再发。
                    if not mode_switch.get("localized"):
                        return {"success": False,
                                "msg": "切换到 SLAM 模式后定位未就绪，已放弃下发目标",
                                "mode_switch": mode_switch}
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

                    # 直接以 (x, y) 为目标；朝向取「当前位置→目标」的接近方向。
                    # 到达精度不再用 distance 提前停车，改由 xy_tolerance/yaw_tolerance 控制。
                    yaw = np.arctan2(y - curr_y, x - curr_x)
                    qz = np.sin(yaw / 2)
                    qw = np.cos(yaw / 2)

                    pose = PoseStamped()
                    pose.header.frame_id = "odom"
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw

                    target_str = f"tmp({x},{y})"
                    # 到达容差在下发时确定并记录，wait_goal_reached 取回使用。
                    self.ros_node.set_slam_goal_tolerance(xy_tolerance, yaw_tolerance)
                    self.ros_node.publish_goal(pose, target_str)
                    return {
                        "success": True,
                        "target": "tmp",
                        "target_str": target_str,
                        "mode_switch": mode_switch,
                        "msg": (f"已下发临时目标({x}, {y})，到达精度 "
                                f"xy≤{xy_tolerance}m / yaw≤{yaw_tolerance}rad"),
                        "target_point": {"x": x, "y": y},
                        "xy_tolerance": xy_tolerance,
                        "yaw_tolerance": yaw_tolerance,
                    }

                # ── 命名语义点：从 MapManager 取位姿 ──
                ctx = None
                if cur_mode != NavNode.NAV_MODE_SLAM:
                    # 室外 GPS → SLAM：先把目标 slam_map/submap 设为当前，使 _ensure_nav_mode
                    # 拉起的 SLAM 栈直接加载正确 pcd（新栈已是目标图，无需再 change_map）。
                    if self.map_manager and slam_map:
                        if not self.map_manager.set_current_slam_map(slam_map):
                            return {"success": False, "msg": f"SLAM 地图不存在: {slam_map}"}
                    if self.map_manager and submap:
                        if not self.map_manager.set_current_submap(submap):
                            return {"success": False, "msg": f"submap 不存在: {submap}"}
                    mode_switch = self._ensure_nav_mode(NavNode.NAV_MODE_SLAM)
                    if mode_switch.get("nav_mode") != NavNode.NAV_MODE_SLAM:
                        return {"success": False,
                                "msg": mode_switch.get("msg", "切换到 SLAM 模式失败"),
                                "mode_switch": mode_switch}
                    # 定位未就绪就下发会把目标发给尚未启动完成的新栈而丢失，
                    # 必须等 _ensure_nav_mode 确认新栈 map→base_link 稳定后再发。
                    if not mode_switch.get("localized"):
                        return {"success": False,
                                "msg": "切换到 SLAM 模式后定位未就绪，已放弃下发目标",
                                "mode_switch": mode_switch}
                else:
                    # 已在 SLAM 模式：若目标 submap 与当前不一致，先导航到传送门再换图。
                    mode_switch = {"switched": False, "nav_mode": cur_mode}
                    if (self.map_manager and slam_map
                            and slam_map != self.map_manager.get_current_slam_map()):
                        # 不支持在 SLAM 模式下跨 SLAM 地图切换：直接返回错误。
                        return {"success": False,
                                "msg": (f"不支持跨 SLAM 地图切换：当前 "
                                        f"{self.map_manager.get_current_slam_map()}，目标 {slam_map}")}
                    if submap and self.map_manager:
                        # 同一 SLAM 地图内：换图前必须先导航到当前→目标 submap 的传送门。
                        ctx = self._ensure_submap(submap)
                        if not ctx.get("success"):
                            return {"success": False,
                                    "msg": f"切换到 submap {submap} 失败: {ctx.get('msg')}",
                                    "change_map": ctx}

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
                pose.pose.position.x = float(pose_info["position"]["x"])
                pose.pose.position.y = float(pose_info["position"]["y"])
                pose.pose.position.z = float(pose_info["position"]["z"])
                pose.pose.orientation.x = float(pose_info["orientation"]["x"])
                pose.pose.orientation.y = float(pose_info["orientation"]["y"])
                pose.pose.orientation.z = float(pose_info["orientation"]["z"])
                pose.pose.orientation.w = float(pose_info["orientation"]["w"])

                point_info = self.map_manager.pose_map_current_submap[target]
                if point_info.get("type") == "elevator_in":
                    ok, emsg = self.ros_node.set_elevator_mode(enter=True)
                    logging.info("前往 elevator_in 点位 %s，自动锁 Z: %s %s", target, ok, emsg)

                # elevator_in / LowPrecision_pose：到达无需精确，强制放宽容差
                # （原先在 wait_goal_reached 里做，现随目标一并在下发时确定）。
                if point_info.get("type") in ("elevator_in", "LowPrecision_pose"):
                    xy_tolerance = max(xy_tolerance, 0.6)
                    yaw_tolerance = max(yaw_tolerance, 3.14)
                self.ros_node.set_slam_goal_tolerance(xy_tolerance, yaw_tolerance)

                self.ros_node.publish_goal(pose, target)
                return {"success": True, "msg": f"目标 {target} 已下发", "target": target,
                        "mode_switch": mode_switch,
                        "xy_tolerance": xy_tolerance, "yaw_tolerance": yaw_tolerance}
            except Exception as e:
                return {"success": False, "msg": f"设置目标失败: {str(e)}"}

        def set_goal_by_outdoor(target: str, lon: float = None, lat: float = None,
                                tolerance: float = 0.8) -> Dict[str, Any]:
            """
            按语义点下发室外 GPS 导航目标（统一经 campus_nav 路网规划器 ``campus_goal_pose``）。

            仅支持室外 GPS 模式，调用时自动切到 gps 模式（不再支持 SLAM 换算）。
            campus_nav 会根据距离自动选择路网规划或 Nav2 直达，无需 ``direct_threshold``。

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
            tolerance: 到达判定距离阈值（米），默认 0.8（与路网规划器终点精确逼近的容差
                       同量级）；随目标记录并供 `wait_goal_reached` 使用。

            返回:
            Dict: 操作结果（含 ``channel``: 'campus' 与估算 ``distance``）
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

                # 室内 SLAM → 室外时，_ensure_nav_mode 会先在 SLAM 内导航到该图通往
                # 全局地图的最近传送门，到达后再切换室外栈；失败则不切换。
                mode_switch = self._ensure_nav_mode(NavNode.NAV_MODE_GPS)
                if mode_switch.get("nav_mode") != NavNode.NAV_MODE_GPS:
                    return {"success": False,
                            "msg": mode_switch.get("msg", "切换到室外 GPS 模式失败"),
                            "mode_switch": mode_switch}
                res = self.ros_node.set_outdoor_goal(
                    float(lon), float(lat), target=target, tolerance=float(tolerance))
                ok = res.get("ok")
                channel = res.get("channel")
                dist = res.get("distance")
                dist_str = "未知" if dist is None else f"{dist:.1f}m"
                return {
                    "success": bool(ok),
                    "mode": "gps",
                    "channel": channel,
                    "distance": dist,
                    "mode_switch": mode_switch,
                    "target": target,
                    "goal": {"lon": lon, "lat": lat},
                    "tolerance": tolerance,
                    "msg": (f"已下发室外目标 target={target} ({lon},{lat})，距离={dist_str}，"
                            f"通道=路网规划器(campus_goal_pose)"
                            if ok else "室外目标下发失败"),
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
            - "elevator_in": 进梯点；`set_goal_by_slam` 下发目标时自动锁 Z（内部处理，无需外部调用）。
            - "elevator_out": 出梯点；出梯后随即换图会重置 Z 锁状态，无需单独解锁。
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
            根据路线名称获取巡逻点位列表（统一 waypoint 定义）。

            每个 waypoint 由 ``type`` 唯一确定导航模式：

            - ``type == "slam"``：``{name, type:'slam', slam_map, submap}``（室内 SLAM）；
            - ``type == "gps"``： ``{name, type:'gps', area, lon, lat, heading_deg}``（室外）。

            客户端按 ``type`` 顺序调用 ``set_goal_by_slam`` / ``set_goal_by_outdoor``
            即可，服务端在下发目标时按需自动切换导航模式（无需 nav_mode/segments）。

            - 场景级全局路线：返回展开后的异构统一 waypoint；
            - 当前 SLAM 地图内部路线：返回统一 SLAM 型 waypoint。

            Args:
                route_name: 路线名称（如 线路A）

            Returns:
            Dict: 包含路线名、巡逻点位列表（统一 waypoint，含 name/type）与 count。
            """
            try:
                is_global = False
                try:
                    is_global = route_name in self.map_manager.get_global_route_names()
                except Exception:
                    is_global = False
                if is_global:
                    route_info = self.map_manager.get_global_route_by_name(route_name)
                    if not route_info:
                        return {"success": False, "msg": f"路线不存在: {route_name}"}
                    return {
                        "success": True,
                        "scope": "global",
                        "name": route_info['name'],
                        "description": route_info.get('description', ''),
                        "waypoints": route_info['waypoints'],
                        "count": len(route_info['waypoints']),
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

        async def wait_goal_reached(target: str, timeout: float = 120.0) -> Dict[str, Any]:
            """
            阻塞直到到达目标点或超时

            - 室外 GPS 模式：按 target 关联的室外目标做几何距离判定
              （campus 规划器无“到达”回调，用 `is_outdoor_goal_reached` 的距离阈值，
              阈值取 `set_goal_by_outdoor` 时记录的 tolerance）。
            - SLAM 模式：按 Nav2 + PID 精调结果判定（`check_goal_reached`），
              容差取 `set_goal_by_slam` 下发时记录的 xy/yaw 容差。

            到达容差不再由本接口传入：室外由 `set_goal_by_outdoor(tolerance=...)`、
            SLAM 由 `set_goal_by_slam(xy_tolerance=..., yaw_tolerance=...)` 在下发时确定。

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

            # SLAM 到达容差取下发时记录的值（含 elevator_in/LowPrecision_pose 的放宽）。
            xy_tolerance, yaw_tolerance = self.ros_node.get_slam_goal_tolerance()
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
            获取当前位置最接近的预设点。

            - SLAM 模式：在当前 submap 语义点中按 map 平面 (x, y) 欧氏距离比较。
            - 室外 GPS 模式：在场景语义点中按经纬度近似平面距离比较，返回经纬度及
              场景点字段（lon/lat/area/heading_deg/type 等）。
            """
            try:
                nav_mode = self.ros_node.get_nav_mode()

                # ── 室外 GPS：按经纬度在场景语义点中找最近点 ──
                if nav_mode == NavNode.NAV_MODE_GPS:
                    if not self.map_manager:
                        return {"success": False, "msg": "MapManager 未配置"}
                    gps_fix = self.ros_node.get_latest_gps_fix()
                    if not gps_fix or gps_fix.get("lon") is None or gps_fix.get("lat") is None:
                        return {"success": False, "msg": "无室外 GNSS 定位数据"}
                    cur_lon = float(gps_fix["lon"])
                    cur_lat = float(gps_fix["lat"])

                    scene_points = self.map_manager.get_scene_semantic_points()
                    if not scene_points:
                        return {"success": False, "msg": "场景语义点为空"}

                    nearest_point = None
                    nearest_info = None
                    min_distance = float("inf")

                    for point_name, point_data in scene_points.items():
                        pt_lon = point_data.get("lon")
                        pt_lat = point_data.get("lat")
                        if pt_lon is None or pt_lat is None:
                            continue
                        distance = self._lonlat_dist_m(cur_lon, cur_lat, float(pt_lon), float(pt_lat))
                        if distance < min_distance:
                            min_distance = distance
                            nearest_point = point_name
                            nearest_info = dict(point_data)

                    if nearest_point is None:
                        return {"success": False, "msg": "没有找到有效的场景语义点"}

                    current_position = {
                        "lon": round(cur_lon, 8),
                        "lat": round(cur_lat, 8),
                    }
                    if gps_fix.get("alt") is not None:
                        current_position["alt"] = round(float(gps_fix["alt"]), 3)

                    return {
                        "success": True,
                        "nav_mode": NavNode.NAV_MODE_GPS,
                        "nearest_point": nearest_point,
                        "point_info": nearest_info,
                        "distance": round(min_distance, 3),
                        "current_position": current_position,
                        "msg": (f"当前位置最接近的场景语义点是: {nearest_point} "
                                f"(距离: {min_distance:.3f}m)"),
                    }

                # ── SLAM：在当前 submap 语义点中按 (x, y) 比较 ──
                odom = self.ros_node.current_pose
                if not odom:
                    return {"success": False, "msg": "无实时位姿数据"}

                current_x = odom.pose.position.x
                current_y = odom.pose.position.y

                nearest_point = None
                nearest_info = None
                min_distance = float("inf")

                for point_name, point_data in self.map_manager.pose_map_current_submap.items():
                    pose_info = point_data.get("pose")
                    if not pose_info:
                        continue

                    target_x = pose_info["position"]["x"]
                    target_y = pose_info["position"]["y"]
                    distance = ((current_x - target_x) ** 2 + (current_y - target_y) ** 2) ** 0.5

                    if distance < min_distance:
                        min_distance = distance
                        nearest_point = point_name
                        nearest_info = dict(point_data)

                if nearest_point is None:
                    return {"success": False, "msg": "没有找到有效的预设点"}

                return {
                    "success": True,
                    "nav_mode": NavNode.NAV_MODE_SLAM,
                    "nearest_point": nearest_point,
                    "point_info": nearest_info,
                    "distance": round(min_distance, 3),
                    "current_position": {
                        "x": round(current_x, 3),
                        "y": round(current_y, 3),
                    },
                    "msg": f"当前位置最接近的预设点是: {nearest_point} (距离: {min_distance:.3f}m)",
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
            "get_route_names": get_route_names,
            "get_default_route": get_default_route,
            "get_route": get_route,
            "get_status": get_status,
            "wait_goal_reached": wait_goal_reached,
            "get_navigation_plan": get_navigation_plan,
            "get_nearest_point": get_nearest_point,
            "get_map": get_map,
            "set_xy_goal_tolerance": set_xy_goal_tolerance,
            "get_nav_mode": get_nav_mode,
            "set_nav_mode": set_nav_mode,
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
    依据 --ss 决定导航模式并拉起对应导航栈（仅负责"首次拉起"）：

      * 指定 --ss slam_map_id/submap_id → SLAM 室内栈：
        解析该 submap 的 pcd 路径后 ``start_slam.sh -m <pcd>``；
      * 未指定 --ss → 室外 GPS 栈：``start_outdoor.sh``。

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
            logging.error("指定了 --ss %s 但地图未加载成功，无法拉起 SLAM 栈", args.submap)
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
    parser.add_argument('--ss', dest='submap', type=str, default=None,
                        help='初始定位目标，格式严格为 slam_map_id/submap_id（例如 caihong/2f；'
                             '单文件地图的 slam_map_id 为 default，如 default/2f）。'
                             '给出 --ss 即进入 SLAM 室内模式：server.py 会解析该 submap 的 pcd 并'
                             '拉起 start_slam.sh -m <pcd>；不给 --ss 则进入室外 GPS 模式，拉起 start_outdoor.sh。'
                             '（注意与常用 -s 区分：-s 一般表示 use_sim_time 仿真时钟）')
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

    # --ss 严格要求 slam_map_id/submap_id 形式（区分"哪张 SLAM 地图的哪个 submap"）。
    if args.submap is not None and '/' not in args.submap:
        parser.error("--ss 需为 slam_map_id/submap_id 格式（例如 caihong/2f 或 default/2f）")

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

    # 依据 --ss 决定导航模式并拉起对应导航栈（"首次拉起"统一由 server.py 负责）。
    _launch_nav_stack(args, map_manager)

    mgr = None
    viz = None
    try:
        # 创建导航工具管理器
        mgr = NavigationToolManager(
            map_manager=map_manager,
            submap_specified=args.submap is not None,
            # --ss 给出 → SLAM 室内；否则 → 室外 GPS（与 _launch_nav_stack 决策一致）
            nav_mode=(NavNode.NAV_MODE_SLAM if args.submap else NavNode.NAV_MODE_GPS),
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
    
