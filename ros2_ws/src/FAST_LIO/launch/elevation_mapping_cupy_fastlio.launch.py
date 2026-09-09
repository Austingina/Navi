import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    elevation_dir = get_package_share_directory('elevation_mapping_cupy')
    core_params = os.path.join(
        elevation_dir, 'config', 'core', 'core_param.yaml')
    robot_params = os.path.join(
        fast_lio_dir, 'config', 'elevation_mapping_cupy_fastlio.yaml')
    # elevation_mapping 专用虚拟环境（与 vpr 解耦）。节点用系统 python3 跑，
    # 这里只把该 venv 的 site-packages 前置到 PYTHONPATH 提供 torch（cupy/numpy
    # 等系统/用户 site 已有）。重建见 envs/elevation_mapping/install.sh。
    # 路径按 NAVI_ROOT 推导并 glob python 小版本，避免写死 /home/nvidia 与
    # python3.10；ELEVATION_VENV_SITE_PACKAGES 可直接覆盖。
    elevation_site_packages = os.environ.get('ELEVATION_VENV_SITE_PACKAGES', '')
    if not elevation_site_packages:
        navi_root = os.environ.get('NAVI_ROOT')
        if not navi_root:
            # Support arbitrary clone directories.  Walk up from the installed
            # package share directory and use the first repository root.
            navi_root = next(
                (str(parent) for parent in Path(fast_lio_dir).parents
                 if (parent / 'envs' / 'elevation_mapping').is_dir()),
                os.path.join(os.path.expanduser('~'), 'Navi'))
        candidates = sorted(glob.glob(os.path.join(
            navi_root, 'envs', 'elevation_mapping', '.venv', 'lib',
            'python3.*', 'site-packages')))
        if candidates:
            elevation_site_packages = candidates[-1]

    python_path = elevation_site_packages
    if os.environ.get('PYTHONPATH'):
        if python_path:
            python_path += os.pathsep + os.environ['PYTHONPATH']
        else:
            python_path = os.environ['PYTHONPATH']

    use_sim_time = LaunchConfiguration('use_sim_time')

    elevation_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            core_params,
            robot_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/elevation_mapping_node/elevation_map_raw',
             '/grid_map_from_raw_pointcloud'),
        ],
        additional_env={'PYTHONPATH': python_path},
    )

    terrain_scan_node = Node(
        package='cloud_to_laserscan',
        executable='terrain_scan',
        name='terrain_scan_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cloud_topic': '/cloud_registered_body',
            'elevation_topic': '/grid_map_from_raw_pointcloud',
            'elevation_layer': 'elevation',
            'output_topic': '/livox_scan2',
            'grid_output_topic': '/terrain_grid_map',
            'slope_cost_topic': '/slope_costmap',
            'base_frame': 'base_link',
            # 局部地面层 = 邻域低分位(0=最小值,0.5=中位数)。低分位跟随坡面但不跟随
            # 戳出来的障碍顶,静态障碍才不会被自己建的高程图吃掉,斜坡也不会整段误报。
            'ground_percentile': 0.2,
            'ground_radius': 3,             # (2r+1)^2 窗
            'ground_min_cells': 25,
            # 障碍高度带(相对局部地面)：
            'obstacle_min_height': 0.15,
            'obstacle_max_height': 1.0,
            # 地面未知的格子回退到脚底高度带(用 TF odom->base_link 的 z)。
            'fallback_on_unknown_elevation': False,
            'fallback_min_height': 0.3,
            'fallback_max_height': 1.0,
            # 坡度：地面层梯度 -> 分级代价 OccupancyGrid(/slope_costmap)。
            # <=free_deg 记 0；free~lethal 线性升到 99（缓坡代价递增，同等条件优先平地）；
            # >=lethal_deg 记 100(lethal，基本过不去)。不再往 scan 里塞假障碍点。
            'slope_enable': True,
            'slope_window': 3,
            # 地形掩码：把"物体"格子从坡度链剔除，否则人/灌木/墙的台阶会被当成陡坡
            # 写进持久化的 TerrainLayer，人走开也不消失（实测 63.7% 的有效 slope
            # 格 >=15°，而地形相邻格高差中位只有 0.4 cm）。
            # slope_object_height: elevation-ground 超过它即判物体。取值要落在
            #   "obstacle 链开始可靠接管"的高度上，两条链才无缝交接：
            #     <0.3 m 的路沿/缓台阶 -> obstacle 链抓不到(被 h<=0.15 滤掉)，
            #                            必须留给 terrain 链，且需要长记忆；
            #     >0.3 m 的人/灌木/车/墙 -> obstacle 链的高度带(地面上 0.15~1.6 m)
            #                            抓得到，且带 raytrace 清除，该归它。
            #   所以是 0.30 而不是 0.15 —— 用 0.15 会把 20 cm 的路沿判成物体剔除，
            #   等于关掉了 terrain 链本来要抓的东西。
            'slope_object_height': 0.30,
            'slope_terrain_band': 0.0,
            'slope_free_deg': 5.0,
            # 统一阈值：>=15° 判不能通行(TerrainLayer 记 LETHAL 并 60m 持久)，
            # 8~12° 路口斜坡落在软代价区间仍可走。
            'slope_lethal_deg': 12.0,
            # <1 使中等坡度代价更高(更强软避让),不改 lethal 阈值。0.5=明显加重。
            'slope_gamma': 0.99,
            # 地形代价点云(给自定义 TerrainLayer 用)。
            'cost_cloud_enable': True,
            'cost_cloud_topic': '/terrain_cost_cloud',
            # 空格清除：TerrainLayer 只会碰 cost cloud 里出现过的格子，没出现的
            # 旧值永久保留。所以"车附近、连高程都没有"的格子要当成"观测到是空的"
            # 发 cost=0，否则人走开后高程图清了、costmap 上的 lethal 却清不掉。
            # 半径取 4.5 m：略小于高程图半边长(map_length=10 -> 5 m)，避免把
            # 边缘从未扫到的区域也判成空。
            'free_on_empty': True,
            'free_on_empty_radius': 2.0,
            # 多帧累积：把最近若干帧的存活障碍点在 odom 系累起来再投 2D，
            # 单帧稀疏被时间窗补满，障碍不再闪烁/缺角。
            'accumulate_enable': True,
            'accumulate_time': 0.8,        # 滑动窗时长 [s]
            'accumulate_max_frames': 10,   # 帧数硬上限
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.0087,
            'range_min': 0.1,
            'range_max': 30.0,

            # ---- 负障碍(悬崖/下沉)检测 -------------------------------------
            'cliff_enable': True,
            'cliff_depth_thresh': 0.25,   # 种子阈值(高阈)
            'cliff_grow_thresh': 0.10,    # 连通域蔓延阈值(低阈)
            # 参考地面 = z_foot - cliff_base_height。实测 base_link 相对 ground 层
            # 高 0.462m。此值偏大只会让参考更低、drop 更小 => 更保守;偏小会误报
            # (实测 0.30 在阈值 0.25 下产生 7.4 格/帧)。宁可取大。
            'cliff_base_height': 0.46,
            # 连通域门限:整个分量至少这么多种子、这么大面积才判 lethal。
            'cliff_min_seed_cells': 6,
            'cliff_min_area_cells': 10,
            # 时间确认(5Hz -> 2 帧 = 0.4s)。upper_bound 本身已是持久层,故为二阶项。
            'cliff_confirm_frames': 2,
            # 判定环带。内径由几何决定:雷达高约 0.5m、最大俯角约 22°,近于 1.3m
            # 不可能有掠过射线。注意它 > footprint 半长 0.4m,所以 footprint clearing
            # 碰不到悬崖格。外径避开高程图边缘(map_length/2 = 5.0)。
            'cliff_min_range': 1.2,
            'cliff_max_range': 4.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),
        elevation_node,
        terrain_scan_node,
    ])
