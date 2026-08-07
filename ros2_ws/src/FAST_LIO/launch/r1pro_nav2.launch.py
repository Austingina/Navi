import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_r1pro_params(param_dir: str) -> str:
    """Pick Nav2 params by ROS distro: Jazzy -> param_r1pro.yaml, else Humble file."""
    distro = os.environ.get('ROS_DISTRO', 'humble').lower()
    if distro == 'jazzy':
        return os.path.join(param_dir, 'param_r1pro.yaml')
    return os.path.join(param_dir, 'param_r1pro_humble.yaml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    wheeltec_nav_dir = get_package_share_directory('fast_lio')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')

    map_file = LaunchConfiguration('map', default="/home/nvidia/work/Navi/maps/r1pro.yaml")

    #Modify the model parameter file, the options are:
    #param_mini_akm.yaml/param_mini_4wd.yaml/param_mini_diff.yaml/
    #param_mini_mec.yaml/param_mini_omni.yaml/param_mini_tank.yaml/
    #param_senior_akm.yaml/param_senior_diff.yaml/param_senior_mec_bs.yaml
    #param_senior_mec_dl.yaml/param_top_4wd_bs.yaml/param_top_4wd_dl.yaml
    #param_top_akm_dl.yaml/param_four_wheel_diff_dl.yaml/param_four_wheel_diff_bs.yaml

    param_dir = os.path.join(wheeltec_nav_dir, 'config')
    default_params = _default_r1pro_params(param_dir)
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


    # Lightweight Python bridge: /cmd_vel -> /motion_target/target_speed_chassis.
    # Was previously a standalone bash-launched script (scripts/galaxea/cmd_odom.py);
    # now lives inside fast_lio/scripts/cmd_vel_bridge.py and is brought up here.
    cmd_vel_bridge_node = Node(
        package='fast_lio',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 刹停守卫。整条速度链（controller -> velocity_smoother -> collision_monitor
    # -> /cmd_vel）在栈被拆掉时会一起死，没人补发零速，底盘会保持最后一条速度指令。
    # 室内→室外切换正是这条路径：start_outdoor.sh 会 tmux kill-session -t slam，
    # 把这一整套连 cmd_vel_bridge 一起 SIGHUP 掉。该节点平时完全沉默（绝不和
    # collision_monitor 抢 /cmd_vel），收到 SIGINT/SIGTERM/SIGHUP 后连发 1.5s 零速。
    safety_stop_node = Node(
        package='fast_lio',
        executable='safety_stop.py',
        name='safety_stop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel',
            # 室内这条链有 cmd_vel_bridge 把 /cmd_vel 转成 TwistStamped 发给底盘，
            # 但桥自己也会被一起杀掉，所以两个话题都补零速。
            'chassis_topic': '/motion_target/target_speed_chassis',
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
            description='Nav2 param file (default: param_r1pro.yaml on Jazzy, '
                        'param_r1pro_humble.yaml otherwise, from $ROS_DISTRO)'),
        #Node(
        #    name='waypoint_cycle',
        #    package='nav2_waypoint_cycle',
        #    executable='nav2_waypoint_cycle',
        #),
        cmd_vel_bridge_node,
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
