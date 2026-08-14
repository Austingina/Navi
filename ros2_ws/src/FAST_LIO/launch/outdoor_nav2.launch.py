import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # True = 全部 nav2 生命周期节点装进单个 component_container_isolated。
    # 非 composition 模式下 9 个节点是 9 个独立进程/9 个 DDS participant，同一秒一起抢
    # 服务发现，lifecycle_manager 会随机卡死在某个节点的 get_state 上
    # （"Waiting for service behavior_server/get_state..." 刷屏，Nav2 一个节点都不 configure）。
    # 室内 slam 栈走 bringup_launch.py 的默认 True，从未出现过该问题。
    use_composition = LaunchConfiguration('use_composition', default='True')

    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_dir, 'launch')

    # 无地图方案下不加载静态地图；此参数仅为满足 bringup 的接口占位，map_server 不会启动。
    map_file = LaunchConfiguration('map', default='')

    param_dir = os.path.join(fast_lio_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'param_outdoor_elevation.yaml'))


    bt_dir = os.path.join(param_dir, 'wheeltec_params', 'behavior_trees')
    bt_nav_through = os.path.join(bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')
    bt_nav_to = os.path.join(bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')

    param_rewrites = {
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': bt_nav_through,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_nav_to,
    }

    rewritten_params = RewrittenYaml(
        source_file=param_file,
        root_key='',
        param_rewrites=param_rewrites,
        convert_types=True)

    # # /cmd_vel -> /motion_target/target_speed_chassis 桥接（与室内方案共用）。
    # cmd_vel_bridge_node = Node(
    #     package='fast_lio',
    #     executable='cmd_vel_bridge.py',
    #     name='cmd_vel_bridge',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='(unused in mapless mode) placeholder for bringup interface'),
        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to the outdoor Nav2 param file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use Nav2 composable container if true'),


        # cmd_vel_bridge_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [fast_lio_launch, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'use_composition': use_composition,
                'params_file': rewritten_params}.items(),
        ),
    ])
