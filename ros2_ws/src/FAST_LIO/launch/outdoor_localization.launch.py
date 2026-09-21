
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    fast_lio_dir = get_package_share_directory('fast_lio')
    # Heading-align params (wait_for_datum + absolute yaw). Simulation bringup
    # keeps using dual_ekf_navsat.yaml via outdoor_localization_dammy.
    params_file = LaunchConfiguration('params', default=os.path.join(
        fast_lio_dir, 'config', 'dual_ekf_navsat_heading_align.yaml'))

    # sim 与 use_sim_time 是两件事：use_sim_time 只管时钟源，sim 管「传感器话题名 +
    # 雷达安装外参」这类硬件差异。start_outdoor.bash 的 -s 会同时置上两者。
    sim = LaunchConfiguration('sim')
    declare_sim = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Simulation hardware profile: picks the sim sensor topics '
                    '(mid360_sim.yaml) and drops the real robot 90 deg lidar '
                    'mounting rotation. Independent of use_sim_time.')

    # FAST-LIO sensor config, overridable from CLI. 默认值按 sim 选：
    #   mid360.yaml      -> /livox/front_lidar, /livox/front_imu  (robot-sensors 的驱动)
    #   mid360_sim.yaml  -> /livox/lidar,       /livox/imu        (Isaac 转出来的 Livox 消息)
    config_path = LaunchConfiguration('config_path', default=PythonExpression([
        "'", os.path.join(fast_lio_dir, 'config', 'mid360_sim.yaml'), "'",
        " if '", sim, "' == 'true' else ",
        "'", os.path.join(fast_lio_dir, 'config', 'mid360.yaml'), "'",
    ]))

    # 真机 Mid360（连带它的 IMU —— FAST-LIO 的 body 系就是 IMU 系）相对 base_link：
    # 位于前方 0.275 m、左方 0.225 m，绕 Z 逆时针 90°，并向车头前倾 15°。
    # 此处发布的是逆向 body->base_link TF，故真机 yaw=-90°、pitch=-15°；
    # 仿真 USD 里没有这些安装旋转，故仿真下必须回到 0（这也是
    # 0fa863a4 之前的原值，仿真当年就是靠它跑通的）。室内栈同一处补偿写在
    # odometry_lslidar.launch.py 的 imu_link->base_link 上，两栈取值须一致。
    body_base_yaw = LaunchConfiguration('body_base_yaw', default=PythonExpression(
        ["'0.0' if '", sim, "' == 'true' else '-1.5708'"]))
    body_base_pitch = LaunchConfiguration('body_base_pitch', default=PythonExpression(
        ["'0.0' if '", sim, "' == 'true' else '-0.261799'"]))

    # 只给 navsat_transform 的 imu remap 用。必须与所选 config_path 里的
    # common.imu_topic 一致 —— 这里没法从 yaml 里读，只能并行维护一份，改一处记得改另一处。
    # 当前 dual_ekf_navsat_heading_align.yaml 是 use_odometry_yaw: true，
    # navsat_transform 忽略 IMU 数据（订阅悬空、无功能影响），但把仿真话题名写死在
    # 真机路径上迟早埋坑，故一并按 profile 选。
    imu_topic = LaunchConfiguration('imu_topic', default=PythonExpression(
        ["'/livox/imu' if '", sim, "' == 'true' else '/livox/front_imu'"]))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')
    declare_params = DeclareLaunchArgument(
        'params', default_value=params_file,
        description='robot_localization params (ekf + navsat); default=heading_align')
    declare_config_path = DeclareLaunchArgument(
        'config_path', default_value=config_path,
        description='FAST-LIO sensor yaml config path (defaults by `sim`)')
    declare_body_base_yaw = DeclareLaunchArgument(
        'body_base_yaw', default_value=body_base_yaw,
        description='Yaw of the body->base_link static TF. -1.5708 on the real '
                    'robot (Mid-360 mounted 90 deg off), 0 in simulation.')
    declare_body_base_pitch = DeclareLaunchArgument(
        'body_base_pitch', default_value=body_base_pitch,
        description='Pitch of the body->base_link static TF. -0.261799 on the '
                    'real robot (Mid-360 tilted 15 deg forward), 0 in simulation.')
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic', default_value=imu_topic,
        description='IMU topic for navsat_transform (defaults by `sim`); keep '
                    'in sync with common.imu_topic in the selected config_path')
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

    # Fixed site origin for the map frame. MUST equal campus_nav's datum so the
    # OSM overlay + goal lon-lat math line up (gps_heading_align pins the map
    # origin here via SetDatum). Overridden from the top-level bringup.
    # MUST equal campus_nav/config/route_planner.yaml's datum (OSM graph origin).
    datum_lat = LaunchConfiguration('datum_lat', default='22.574357630323675')
    datum_lon = LaunchConfiguration('datum_lon', default='113.93507078035577')
    declare_datum_lat = DeclareLaunchArgument(
        'datum_lat', default_value='22.574357630323675',
        description='Fixed map/campus datum latitude (ENU origin)')
    declare_datum_lon = DeclareLaunchArgument(
        'datum_lon', default_value='113.93507078035577',
        description='Fixed map/campus datum longitude (ENU origin)')


    tf_odom_to_camera_init = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_odom_to_camera_init',
        arguments=['0','0','0','0','0','0','odom','camera_init'],
        output='screen',
    )

    tf_body_to_base_link = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_body_to_base_link',
        # Inverse of base_link->body:
        # position=(+0.275,+0.225,0), rotation=Ry(+15deg)*Rz(+90deg).
        arguments=['-0.225', '0.26563', '-0.071175',
                   body_base_yaw, body_base_pitch, '0', 'body', 'base_link'],
        output='screen',
    )
    tf_base_link_to_gps = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_base_link_to_gps',
        arguments=['0.10','-0.15','0','0','0','0','base_link','gps'],
        output='screen',
    )

    # base_link -> camera1_link。数值在 config/camera_extrinsics.yaml（单一真源，
    # 室内/室外两栈读同一个文件）。没有这条边 semantic_segmentation_layer 会【静默
    # 死亡】—— 它的 tf2_ros::MessageFilter 会在同步器触发前丢掉每一帧点云，
    # 连 .cpp:495-516 那些诊断 WARN 都不会打。
    #
    # 用带名参数而不是位置参数：位置写法的顺序是 x y z yaw pitch roll，极易写错，
    # 且在 Jazzy 上已弃用（本文件其余静态 TF 仍是位置写法，保持原状不动）。
    #
    # ⚠ camera2 现在【不要】加：rs_multi_camera_launch.py:70-78 发了一条全零的
    #   dummy camera1_link -> camera2_link。补上本条边后，camera2 会从"明显断开"
    #   变成"静默地等于 camera1"。要用 camera2 得先去掉 robot-sensors 里那条 dummy。
    with open(os.path.join(get_package_share_directory('fast_lio'),
                           'config', 'camera_extrinsics.yaml')) as _f:
        _cam = yaml.safe_load(_f)['camera_extrinsics']
    tf_base_link_to_camera1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='statictf_base_link_to_camera1_link',
        arguments=['--x', str(_cam['camera1_x']), '--y', str(_cam['camera1_y']),
                   '--z', str(_cam['camera1_z']), '--yaw', str(_cam['camera1_yaw']),
                   '--pitch', str(_cam['camera1_pitch']),
                   '--roll', str(_cam['camera1_roll']),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera1_link'],
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

    # GPS course → absolute ENU yaw for navsat (urban-canyon gated).
    heading_align_node = Node(
        package='fast_lio',
        executable='gps_heading_align.py',
        name='gps_heading_align',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fix_topic': '/gps/fix',
            'odom_topic': '/Odometry',
            'absolute_yaw_topic': '/odometry/absolute_yaw',
            'status_topic': '/heading_align/status',
            'datum_service': '/datum',
            'datum_lat': datum_lat,
            'datum_lon': datum_lon,
            'auto_drive': False,
            'exit_after_lock': False,
            'refine_after_lock': True,
            'refine_min_distance': 30.0,
            'refine_max_distance': 50.0,
            'refine_max_h_var': 0.05,
            'refine_alpha': 0.3,
            'min_distance': 1.5,        # min rolling baseline before chord usable
            'min_total_distance': 8.0,  # baseline before lock; heading_err~=sigma/baseline
            'lock_stable_tol_deg': 3.0,
            'min_fix_status': 2,
            'max_h_var': 1.0,
            'call_set_datum': True,
        }],
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
                     'process_noise': 0.01,       # LIO 漂移速率(m^2/s)
                     'gps_variance_default': 1.0,  # GPS 消息无协方差时的兜底
                     'use_msg_covariance': True,   # 优先用 GPS 自带协方差当 R
                     'gps_gate_sigma': 5.0,        # 突跳门限(sigma)，0=关
                     # 输出限速：map->odom 单周期位移上界 = 该值/publish_rate
                     # (0.3/20 = 0.015m)。这是真正消除"瞬时传送"的一条，0=关。
                     'max_correction_speed': 0.3,
                     # R 信任策略：膨胀倍数 / 下限 / 上限
                     'gps_variance_scale': 4.0,    # sigma x2，抵消上报的乐观
                     'gps_variance_floor': 0.25,   # sigma 0.5m，不信 cm 级
                     'max_gps_variance': 400.0,    # sigma 20m 以上视为无信息
                     # 按 NavSatStatus 追加惩罚（解越差、上报的 sigma 越乐观）
                     'var_scale_status_gbas': 1.0,  # status 2: RTK
                     'var_scale_status_sbas': 2.0,  # status 1
                     'var_scale_status_fix': 9.0,   # status 0: SINGLE
                     'min_fix_status': 2,
                     'max_h_var': 1.0,
                     'max_variance': 4.0,          # var 封顶，防增益趋 1
                     'sustained_offset_sec': 3.0,  # 持续偏移改为限速爬入
                     # 假固定检测：解算精度没跃升却发生位置阶跃，说明整周模糊度
                     # 解错了。这种偏移同样"持续"，会被上面的爬入当成 LIO 漂移
                     # 吃进来；而且 gate 约 2.5m，比它小的假固定压根进不了门限，
                     # 直接就融合了。检测到就暂停位置融合、纯 LIO 推算，直到接
                     # 收机自己跳回来（跳回会被识别为撤销，立即恢复融合）。
                     'step_detect_speed': 3.0,     # GPS 与 LIO 增量失配(m/s)，0=关
                     'step_sigma_ratio': 5.0,      # sigma 改善几倍算真的解恢复
                     # 假固定持续超过该时长则退回原行为(限速爬入)，按实测调整
                     'step_distrust_sec': 600.0,
                     'bad_gps_hold_sec': 3.0,      # 差解超时 -> 纯 LIO 推算
                     'reseed_on_heading_lock_sec': 30.0,
                     'datum_lat': datum_lat,        # 与 gps_heading_align / campus_nav 一致
                     'datum_lon': datum_lon}],
        condition=use_gps_simple,
    )

    # navsat_transform：/fix + absolute_yaw -> /odometry/gps。
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        # Silence the per-cycle "Datum ... is / Datum UTM coordinate is" INFO
        # spam; keep WARN/ERROR.
        arguments=['--ros-args', '--log-level', 'navsat_transform:=warn'],
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', imu_topic),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/absolute_yaw'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    return LaunchDescription([
        # declare_sim 必须排在最前：下面几个 argument 的默认值都是引用 `sim` 的
        # PythonExpression，launch 按顺序访问 action，sim 未声明时它们解析不出来。
        declare_sim,
        declare_use_sim_time,
        declare_params,
        declare_config_path,
        declare_body_base_yaw,
        declare_body_base_pitch,
        declare_imu_topic,
        declare_enable_elevation,
        declare_map_odom_source,
        declare_datum_lat,
        declare_datum_lon,
        fast_lio_node,
        cupy_elevation,
        heading_align_node,
        ekf_map_node,
        map_odom_gps_node,
        navsat_node,
        tf_odom_to_camera_init,
        tf_body_to_base_link,
        tf_base_link_to_gps,
        tf_base_link_to_camera1,
    ])
