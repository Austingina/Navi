# =============================================================================
# heading_calib.launch.py — 每次初始化的 GPS 航向标定
# =============================================================================
# 作用：起 gps_heading_calib 节点，直行一小段自动求 yaw_offset 并写回
#       dual_ekf_navsat.yaml（navsat_transform 读取的那份）。
#
# 顺序（关键）：
#   1) 先起 FAST-LIO（提供 /Odometry、odom->base_link）+ GPS 驱动（/fix）+ 底盘。
#   2) 跑本 launch 完成标定（节点结束即退出）。
#   3) 再 ros2 launch fast_lio outdoor_localization.launch.py（读新 yaw_offset）。
#
# 默认写入的是 share 目录里的 dual_ekf_navsat.yaml，也正是 outdoor_localization
# 启动时加载的那一份；用 --symlink-install 时它就是源文件（改动持久）。
#
# 用法：
#   ros2 launch fast_lio heading_calib.launch.py
#   ros2 launch fast_lio heading_calib.launch.py auto_drive:=false          # 手动推
#   ros2 launch fast_lio heading_calib.launch.py drive_distance:=5.0 drive_speed:=0.4
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    default_yaml = os.path.join(fast_lio_dir, 'config', 'dual_ekf_navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    auto_drive = LaunchConfiguration('auto_drive', default='true')
    drive_distance = LaunchConfiguration('drive_distance', default='3.0')
    drive_speed = LaunchConfiguration('drive_speed', default='0.3')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    fix_topic = LaunchConfiguration('fix_topic', default='/fix')
    odom_topic = LaunchConfiguration('odom_topic', default='/Odometry')
    yaml_path = LaunchConfiguration('yaml_path', default=default_yaml)

    calib_node = Node(
        package='fast_lio',
        executable='gps_heading_calib.py',
        name='gps_heading_calib',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_drive': auto_drive,
            'drive_distance': drive_distance,
            'drive_speed': drive_speed,
            'cmd_vel_topic': cmd_vel_topic,
            'fix_topic': fix_topic,
            'odom_topic': odom_topic,
            'yaml_path': yaml_path,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_drive', default_value='true',
                              description='true=节点自动发 /cmd_vel 直行；false=手动推'),
        DeclareLaunchArgument('drive_distance', default_value='3.0',
                              description='直行距离(m)，RTK 下 2-3m 足够'),
        DeclareLaunchArgument('drive_speed', default_value='0.3',
                              description='直行线速度(m/s)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('fix_topic', default_value='/fix'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('yaml_path', default_value=default_yaml,
                              description='要写回 yaw_offset 的 yaml；默认=share 里的 dual_ekf_navsat.yaml'),
        calib_node,
    ])
