import os

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
    # 允许用环境变量 ELEVATION_VENV_SITE_PACKAGES 覆盖路径。
    elevation_site_packages = os.environ.get(
        'ELEVATION_VENV_SITE_PACKAGES',
        '/home/nvidia/Navi/envs/elevation_mapping/.venv/lib/python3.10/site-packages')
    python_path = elevation_site_packages
    if os.environ.get('PYTHONPATH'):
        python_path += os.pathsep + os.environ['PYTHONPATH']

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
            'cloud_topic': '/livox/points',
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
            'ground_min_cells': 4,
            # 障碍高度带(相对局部地面)：地面上方 3cm~1.6m 视为障碍。
            'obstacle_min_height': 0.08,
            'obstacle_max_height': 1.6,
            # 地面未知的格子回退到脚底高度带(用 TF odom->base_link 的 z)。
            'fallback_on_unknown_elevation': True,
            'fallback_min_height': 0.2,
            'fallback_max_height': 1.6,
            # 坡度：地面层梯度 -> 分级代价 OccupancyGrid(/slope_costmap)。
            # <=free_deg 记 0；free~lethal 线性升到 99（缓坡代价递增，同等条件优先平地）；
            # >=lethal_deg 记 100(lethal，基本过不去)。不再往 scan 里塞假障碍点。
            'slope_enable': True,
            'slope_window': 2,
            'slope_free_deg': 3.0,
            # 统一阈值：>=15° 判不能通行(TerrainLayer 记 LETHAL 并 60m 持久)，
            # 8~12° 路口斜坡落在软代价区间仍可走。
            'slope_lethal_deg': 12.0,
            # <1 使中等坡度代价更高(更强软避让),不改 lethal 阈值。0.5=明显加重。
            'slope_gamma': 0.99,
            # 地形代价点云(给自定义 TerrainLayer 用)。
            'cost_cloud_enable': True,
            'cost_cloud_topic': '/terrain_cost_cloud',
            # 多帧累积：把最近若干帧的存活障碍点在 odom 系累起来再投 2D，
            # 单帧稀疏被时间窗补满，障碍不再闪烁/缺角。
            'accumulate_enable': True,
            'accumulate_time': 0.8,        # 滑动窗时长 [s]
            'accumulate_max_frames': 10,   # 帧数硬上限
            'angle_min': -1.57,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.1,
            'range_max': 30.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),
        elevation_node,
        terrain_scan_node,
    ])

