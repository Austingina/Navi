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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': rewritten_params}.items(),
        ),

    ])
