import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_yhs_params(param_dir: str) -> str:
    """煜和森（yhs）底盘的 Nav2 参数文件。

    目前只提供 Jazzy 版 param_yhs.yaml；若将来需要 Humble 专用文件，
    再新增 param_yhs_humble.yaml 并在此按 $ROS_DISTRO 分支返回。
    """
    return os.path.join(param_dir, 'param_yhs.yaml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    wheeltec_nav_dir = get_package_share_directory('fast_lio')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')

    map_file = LaunchConfiguration('map', default="/home/nvidia/work/Navi/maps/yhs.yaml")

    #Modify the model parameter file, the options are:
    #param_mini_akm.yaml/param_mini_4wd.yaml/param_mini_diff.yaml/
    #param_mini_mec.yaml/param_mini_omni.yaml/param_mini_tank.yaml/
    #param_senior_akm.yaml/param_senior_diff.yaml/param_senior_mec_bs.yaml
    #param_senior_mec_dl.yaml/param_top_4wd_bs.yaml/param_top_4wd_dl.yaml
    #param_top_akm_dl.yaml/param_four_wheel_diff_dl.yaml/param_four_wheel_diff_bs.yaml

    param_dir = os.path.join(wheeltec_nav_dir, 'config')
    default_params = _default_yhs_params(param_dir)
    params = LaunchConfiguration('params')

    # Prepare parameter rewrites for BT XML files so paths are not hardcoded in YAML
    slope_pkg_dir = get_package_share_directory('slope_behavior')
    fast_lio_pkg_dir = wheeltec_nav_dir
    bt_nav_through = os.path.join(fast_lio_pkg_dir, 'config', 'wheeltec_params', 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml')
    bt_nav_to = os.path.join(fast_lio_pkg_dir, 'config', 'wheeltec_params', 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    # The parameters in the YAML are under the node name -> ros__parameters -> <param>.
    # RewrittenYaml needs the full path including 'ros__parameters' to override them.
    param_rewrites = {
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': bt_nav_through,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_nav_to,
    }

    rewritten_params = RewrittenYaml(
        source_file=params,
        root_key='',
        param_rewrites=param_rewrites,
        convert_types=True)


    # 煜和森（yhs）底盘直接订阅 /cmd_vel，不需要把 /cmd_vel 转成 TwistStamped 的
    # cmd_vel_bridge（那是 Galaxea 底盘用的）。因此这里不再启动 cmd_vel_bridge。

    # 刹停守卫。整条速度链（controller -> velocity_smoother -> collision_monitor
    # -> /cmd_vel）在栈被拆掉时会一起死，没人补发零速，底盘会保持最后一条速度指令。
    # 该节点平时完全沉默（绝不和 collision_monitor 抢 /cmd_vel），收到
    # SIGINT/SIGTERM/SIGHUP 后连发 1.5s 零速。yhs 直接吃 /cmd_vel，故关闭 chassis_topic。
    safety_stop_node = Node(
        package='fast_lio',
        executable='safety_stop.py',
        name='safety_stop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel',
            # yhs 无 cmd_vel_bridge，底盘直接订阅 /cmd_vel，故置空关闭第二话题。
            'chassis_topic': '',
            'stop_duration_s': 1.5,
            'stop_rate_hz': 20.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=default_params,
            description='Nav2 param file for the yhs (煜和森) chassis '
                        '(default: param_yhs.yaml)'),
        #Node(
        #    name='waypoint_cycle',
        #    package='nav2_waypoint_cycle',
        #    executable='nav2_waypoint_cycle',
        #),
        safety_stop_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': rewritten_params}.items(),
        ),

    ])
