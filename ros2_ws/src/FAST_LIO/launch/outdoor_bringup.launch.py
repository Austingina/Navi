"""One-shot outdoor bringup: localization + Nav2 + campus route planner.

按依赖顺序错峰拉起三套 launch：

  t=0s   outdoor_localization  —  FAST-LIO + navsat + map->odom + 静态 TF
                                   (+ 可选 cupy 高程图)
  t=5s   outdoor_nav2          —  Nav2 bringup（需要 localization 先立起 TF）
  t=8s   route_planner         —  campus_nav 路网/路由（send_to_nav2 需 Nav2 就绪）

顶层统一一个 use_sim_time 往下透传（route_planner 原默认 false，这里跟随全局）。
帧关系：localization 提供 map->odom->base_link，campus_nav 追加 map->campus_map，
都挂在 map 下，互不冲突。

常用覆盖示例：
  ros2 launch fast_lio outdoor_bringup.launch.py use_sim_time:=false
  ros2 launch fast_lio outdoor_bringup.launch.py map_odom_source:=ekf
  ros2 launch fast_lio outdoor_bringup.launch.py enable_elevation:=false
  ros2 launch fast_lio outdoor_bringup.launch.py send_to_nav2:=false
"""

import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 上一次 bringup 若被强杀/异常退出，某些节点（尤其 Python 的 elevation_mapping_node）
# 不会随之退出，会变成 PPID=1 的孤儿继续发布。多份同名节点叠在同一个 topic 上会
# 让 /grid_map_from_raw_pointcloud 乱跳，并把 terrain_scan 灌爆导致 /livox_scan2
# 饿死。这里在起新节点之前先按本栈已知可执行名清掉残留（只匹配本栈，不动其它进程）。
_STALE_PATTERNS = [
    'elevation_mapping_node.py',
    'cloud_to_laserscan/terrain_scan',
    'cloud_to_laserscan/cloud_to_2d_laserscan',
]


def _pids_for(pattern):
    try:
        out = subprocess.run(
            ['pgrep', '-f', pattern], capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return []
    self_pid = os.getpid()
    pids = []
    for tok in out.stdout.split():
        try:
            pid = int(tok)
        except ValueError:
            continue
        if pid != self_pid:
            pids.append(pid)
    return pids


def _cleanup_stale_nodes():
    killed = []
    for pat in _STALE_PATTERNS:
        for pid in _pids_for(pat):
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append((pid, pat))
                print(f'[outdoor_bringup] SIGTERM stale node pid={pid} ({pat})')
            except ProcessLookupError:
                pass
            except Exception as exc:  # noqa: BLE001
                print(f'[outdoor_bringup] kill {pid} failed: {exc}')
    if not killed:
        return
    time.sleep(1.5)
    # 顽固残留升级为 SIGKILL。
    for pat in _STALE_PATTERNS:
        for pid in _pids_for(pat):
            try:
                os.kill(pid, signal.SIGKILL)
                print(f'[outdoor_bringup] SIGKILL stubborn node pid={pid} ({pat})')
            except ProcessLookupError:
                pass
            except Exception as exc:  # noqa: BLE001
                print(f'[outdoor_bringup] kill -9 {pid} failed: {exc}')


def generate_launch_description():
    # 起任何节点之前先清理上一次的残留/孤儿（解析 launch 时同步执行一次）。
    _cleanup_stale_nodes()

    fast_lio_launch = os.path.join(
        get_package_share_directory('fast_lio'), 'launch')
    campus_nav_launch = os.path.join(
        get_package_share_directory('campus_nav'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_odom_source = LaunchConfiguration('map_odom_source')
    enable_elevation = LaunchConfiguration('enable_elevation')
    nav_params = LaunchConfiguration('nav_params')
    send_to_nav2 = LaunchConfiguration('send_to_nav2')

    # ---- top-level args (透传给子 launch) ----
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock; shared by all three stacks'),
        DeclareLaunchArgument(
            'map_odom_source', default_value='gps',
            description="localization map->odom source: 'ekf' or 'gps'"),
        DeclareLaunchArgument(
            'enable_elevation', default_value='true',
            description='Build cupy elevation map + terrain-aware LaserScan'),
        DeclareLaunchArgument(
            'nav_params', default_value=os.path.join(
                get_package_share_directory('fast_lio'), 'config',
                'param_outdoor_elevation.yaml'),
            description='Full path to the outdoor Nav2 param file'),
        DeclareLaunchArgument(
            'send_to_nav2', default_value='true',
            description='route_planner: forward campus waypoints to Nav2'),
    ]

    # t=0 —— 定位 / 建图（TF 根基）
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_launch, 'outdoor_localization_dammy.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_odom_source': map_odom_source,
            'enable_elevation': enable_elevation,
        }.items(),
    )

    # t=5 —— Nav2（等 localization 立起 map->odom->base_link）
    nav2 = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_launch, 'outdoor_nav2.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params': nav_params,
            }.items(),
        )],
    )

    # t=8 —— 路网 / 路由（send_to_nav2 需 Nav2 action server 就绪）
    route_planner = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(campus_nav_launch, 'route_planner.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'send_to_nav2': send_to_nav2,
            }.items(),
        )],
    )

    return LaunchDescription(
        declare_args + [localization, nav2, route_planner])
