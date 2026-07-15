
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    fast_lio_dir = get_package_share_directory('fast_lio')
    params_file = LaunchConfiguration('params', default=os.path.join(
        fast_lio_dir, 'config', 'dual_ekf_navsat.yaml'))

    # FAST-LIO sensor config, overridable from CLI.
    config_path = LaunchConfiguration('config_path', default=os.path.join(
        fast_lio_dir, 'config', 'mid360_sim.yaml'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')
    declare_params = DeclareLaunchArgument(
        'params', default_value=params_file,
        description='robot_localization params (ekf + navsat)')
    declare_config_path = DeclareLaunchArgument(
        'config_path', default_value=config_path,
        description='FAST-LIO sensor yaml config path')
    enable_elevation = LaunchConfiguration('enable_elevation')
    declare_enable_elevation = DeclareLaunchArgument(
        'enable_elevation', default_value='true',
        description='Build the cupy elevation map and terrain-aware LaserScan')
    use_cupy_elevation = IfCondition(enable_elevation)

    # Who publishes map->odom:  'ekf' = robot_localization EKF,
    #                           'gps' = simple translation-only node.
    # navsat runs in both cases (it produces /odometry/gps).
    map_odom_source = LaunchConfiguration('map_odom_source', default='gps')
    declare_map_odom_source = DeclareLaunchArgument(
        'map_odom_source', default_value='gps',
        description="map->odom source: 'ekf' or 'gps'")
    use_ekf = IfCondition(PythonExpression(
        ["'", map_odom_source, "' == 'ekf'"]))
    use_gps_simple = IfCondition(PythonExpression(
        ["'", map_odom_source, "' == 'gps'"]))


    tf_odom_to_camera_init = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_odom_to_camera_init',
        arguments=['0','0','0','0','0','0','odom','camera_init'],
        output='screen',
    )
    tf_body_to_base_link = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_body_to_base_link',
        # TODO: 换成 IMU->base_link 实测外参
        arguments=['0','0','0','0','0','0','body','base_link'],
        output='screen',
    )


    # FAST-LIO inertial odometry (pure LIO, builds map online, no global
    # localization / prior map). Outputs /Odometry + /cloud_registered.
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time,
                     'pcd_save.pcd_save_en': False}],
        output='screen',
    )

    cupy_elevation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            fast_lio_dir, 'launch',
            'elevation_mapping_cupy_fastlio.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=use_cupy_elevation,
    )

    # 全局 EKF：输出 map->odom。内部话题 odometry/filtered 重映射成独立名，
    # 供 navsat_transform 回读（datum/反馈用）。
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered_map')],
        condition=use_ekf,
    )

    # 路线二：不用卡尔曼，直接从 GPS 算 map->odom（只纠平移，航向信 LIO）。
    map_odom_gps_node = Node(
        package='fast_lio',
        executable='map_odom_from_gps.py',
        name='map_odom_from_gps',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'gps_topic': '/odometry/gps',
                     'publish_rate': 20.0,
                     'process_noise': 0.05,       # LIO 漂移速率(m^2/s)
                     'gps_variance_default': 1.0,  # GPS 消息无协方差时的兜底
                     'use_msg_covariance': True,   # 优先用 GPS 自带协方差当 R
                     'gps_gate_sigma': 5.0}],      # 突跳门限(sigma)，0=关
        condition=use_gps_simple,
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
            ('gps/fix', '/gps/fix'),
            # ('odometry/filtered', '/odometry/filtered_map'),
            ('odometry/filtered', '/odom_gt'),
            # Output
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_config_path,
        declare_enable_elevation,
        declare_map_odom_source,
        fast_lio_node,
        cupy_elevation,
        ekf_map_node,
        map_odom_gps_node,
        navsat_node,
        tf_odom_to_camera_init,
        tf_body_to_base_link,
    ])
