# =============================================================================
# heading_calib.launch.py — 开阔地 GPS 航向首锁（可选自动直行）
# =============================================================================
# 作用：起 gps_heading_align（auto_drive + exit_after_lock），在开阔地直行一段
#       完成对北首锁后退出。不写 yaml（每次开机朝向都会变）。
#
# 顺序：
#   1) 先起 FAST-LIO（/Odometry）+ GPS（/gps/fix）+ 底盘。
#   2) 跑本 launch 完成首锁（节点 exit_after_lock 后退出）。
#   3) 再 ros2 launch fast_lio outdoor_localization.launch.py
#      （常驻 align 会在已有运动/后续运动下工作；若本标定已 SetDatum，
#       重启 localization 仍需再锁一次或保留 align 常驻）。
#
# 更常见：直接跑 outdoor_localization（内含常驻 align），本 launch 仅用于
# 希望开机立刻自动直行首锁的场景。
#
# 用法：
#   ros2 launch fast_lio heading_calib.launch.py
#   ros2 launch fast_lio heading_calib.launch.py auto_drive:=false
#   ros2 launch fast_lio heading_calib.launch.py min_distance:=5.0 drive_speed:=0.4
# =============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    auto_drive = LaunchConfiguration('auto_drive', default='true')
    drive_speed = LaunchConfiguration('drive_speed', default='0.3')
    min_distance = LaunchConfiguration('min_distance', default='5.0')
    min_speed = LaunchConfiguration('min_speed', default='0.5')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    fix_topic = LaunchConfiguration('fix_topic', default='/gps/fix')
    odom_topic = LaunchConfiguration('odom_topic', default='/Odometry')

    calib_node = Node(
        package='fast_lio',
        executable='gps_heading_align.py',
        name='gps_heading_align',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_drive': auto_drive,
            'drive_speed': drive_speed,
            'min_distance': min_distance,
            'min_speed': min_speed,
            'cmd_vel_topic': cmd_vel_topic,
            'fix_topic': fix_topic,
            'odom_topic': odom_topic,
            'absolute_yaw_topic': '/odometry/absolute_yaw',
            'status_topic': '/heading_align/status',
            'datum_service': '/navsat_transform/datum',
            'exit_after_lock': True,
            'refine_after_lock': False,
            'call_set_datum': True,
            'max_h_var': 25.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_drive', default_value='true',
                              description='true=节点自动发 /cmd_vel 直行；false=手动推'),
        DeclareLaunchArgument('drive_speed', default_value='0.3',
                              description='直行线速度(m/s)'),
        DeclareLaunchArgument('min_distance', default_value='5.0',
                              description='首锁所需 GPS 位移(m)'),
        DeclareLaunchArgument('min_speed', default_value='0.5',
                              description='首锁所需推断速度(m/s)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('fix_topic', default_value='/gps/fix'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        calib_node,
    ])
