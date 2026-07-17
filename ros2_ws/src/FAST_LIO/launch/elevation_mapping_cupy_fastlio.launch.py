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
    vpr_site_packages = os.path.join(
        '/home/nvidia/Navi/NaviSkill/vpr/.venv/lib/python3.10/site-packages')
    python_path = vpr_site_packages
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
        executable='cloud_to_2d_laserscan',
        name='cloud_to_2d_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cloud_topic': '/livox/points',
            'elevation_topic': '/grid_map_from_raw_pointcloud',
            'elevation_layer': 'elevation',
            'output_topic': '/livox_scan2',
            'ground_tolerance': 0.2,
            'max_height': 1.6,
            'fallback_on_unknown_elevation': False,
            # 多帧累积：把最近若干帧的存活障碍点在 odom 系累起来再投 2D，
            # 单帧稀疏被时间窗补满，障碍不再闪烁/缺角。
            'accumulate_enable': True,     # 重校验必须配合累积才有意义
            'accumulate_time': 0.3,        # 滑动窗时长 [s]
            'accumulate_max_frames': 10,   # 帧数硬上限
            # 邻域中位数地面参照：对 elevation 图里的凹陷格免疫,避免地面误判成障碍。
            'robust_ground_enable': True,
            'robust_ground_radius': 3,      # 7x7 窗(0.7m),凹陷格聚成小块时可加大
            'robust_ground_min_cells': 4,
            # 重校验：每周期用最新 elevation 把窗口里的老点重判,踢掉现在不再是障碍的。
            # 治瞬时误报(图错一帧后自愈);治不了持续凹陷格(图一直错)。
            'revalidate_enable': False,
            # 每帧自标定:减掉 live 云与 elevation 图之间的全局垂直偏移。
            # 专治"整体高度拔高导致周围全变障碍"(z 跳变)。
            'self_calibrate_enable': True,
            'self_calibrate_band': 0.5,
            'self_calibrate_min_points': 100,
            'self_calibrate_max_offset': 1.0,
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

