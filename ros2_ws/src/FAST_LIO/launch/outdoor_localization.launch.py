# =============================================================================
# outdoor_localization.launch.py — 实车 GPS 融合定位（robot_localization）
# =============================================================================
# 起两个节点：
#   1. navsat_transform_node : /fix + 航向 -> /odometry/gps（GPS 在 map 系的位姿）
#   2. ekf_filter_node (map) : 融合 /Odometry(LIO) + /odometry/gps -> 发布 map->odom
#
# 与本 launch 配套但需另行启动：
#   - FAST-LIO（提供 odom->base_link 和 /Odometry）
#   - GPS 驱动（发 /fix）、IMU（/livox/imu）
#   - Nav2：outdoor_nav2.launch.py
#
# ⚠️ 必须确保没有其它节点再发 map->odom（例如 odometry_lslidar.launch.py 里的
#    静态 map->odom，实车 GPS 方案下要去掉），否则 TF 冲突。
#
# 依赖：sudo apt install ros-humble-robot-localization
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    fast_lio_dir = get_package_share_directory('fast_lio')
    params_file = LaunchConfiguration('params', default=os.path.join(
        fast_lio_dir, 'config', 'dual_ekf_navsat.yaml'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')
    declare_params = DeclareLaunchArgument(
        'params', default_value=params_file,
        description='robot_localization params (ekf + navsat)')

    # 全局 EKF：输出 map->odom。内部话题 odometry/filtered 重映射成独立名，
    # 供 navsat_transform 回读（datum/反馈用）。
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered_map')],
    )

    # navsat_transform：/fix -> /odometry/gps。
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/livox/imu'),
            ('gps/fix', '/fix'),
            # 回读全局 EKF 的输出，用于把 GPS 对齐到 map 系
            ('odometry/filtered', '/odometry/filtered_map'),
            # 输出（默认名，显式写出便于排查）
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        ekf_map_node,
        navsat_node,
    ])
