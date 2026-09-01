import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node #, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')

    # Create LaunchConfigurations for the parameters with default values (not in config file)
    feature_extract_enable_param = LaunchConfiguration('feature_extract_enable', default='false') # bool
    point_filter_num_param = LaunchConfiguration('point_filter_num', default='3')                 # int
    max_iteration_param = LaunchConfiguration('max_iteration', default='3')                       # int
    filter_size_surf_param = LaunchConfiguration('filter_size_surf', default='0.3')               # double
    filter_size_map_param = LaunchConfiguration('filter_size_map', default='0.3')                 # double
    cube_side_length_param = LaunchConfiguration('cube_side_length', default='1000.0')            # double
    map_file_path = LaunchConfiguration('map_file_path', default='/home/nvidia/maps/share525_1.pcd')         # double
    runtime_pos_log_enable_param = LaunchConfiguration('runtime_pos_log_enable', default='false')  # bool

    # sim 与 use_sim_time 是两件事，故分开：use_sim_time 只管时钟源，sim 管
    # 「传感器话题名 + 雷达安装外参」这类硬件差异。启动脚本的 -s 会同时置上两者
    # （见 start_localization.bash），但手工 launch 时可以只要其中一个。
    sim = LaunchConfiguration('sim')

    # 仿真(Isaac)与真机的雷达/IMU 话题名不同，分别写在两个 yaml 里：
    #   mid360_lslidar.yaml -> /livox/front_lidar, /livox/front_imu  (robot-sensors 的驱动)
    #   mid360_sim.yaml     -> /livox/lidar,       /livox/imu        (Isaac 转出来的 Livox 消息)
    # 真机分支用本 launch 专属的 mid360_lslidar.yaml（从 mid360.yaml 复制而来），
    # 与 mapping/outdoor 等共用的 mid360.yaml 解耦，便于单独调参而不影响其它栈。
    # 只改默认值：显式传 config_path:= 仍然优先（LaunchConfiguration 的固有语义），
    # 保留手工指定任意配置的能力。
    default_config_path = PythonExpression([
        "'", os.path.join(package_path, 'config', 'mid360_sim.yaml'), "'",
        " if '", sim, "' == 'true' else ",
        "'", os.path.join(package_path, 'config', 'mid360_lslidar.yaml'), "'",
    ])
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio-localize.rviz')

    # 真机 imu_link->base_link 静态 TF（已实测校准）。
    # 前雷达装在 base_link 左前方；其数据帧（imu_link；extrinsic_R 单位阵，故
    # imu_link=livox_frame）+X 指向车体左方、并向车头前倾 15°。
    # 物理 base_link->imu = Ry(+15°)·Rz(+90°)，取逆得发布用的 imu_link->base_link：
    #   yaw=-90°（+X 朝左）、pitch=-15°（前倾补偿，扶平 base_link 使其水平）、roll=0。
    # 仿真 USD 无此安装旋转，故 sim 下 yaw/pitch 都回到 0。
    # 平移用 (-0.24, -0.270459, -0.072469)。
    default_imu_base_yaw = PythonExpression(
        ["'0.0' if '", sim, "' == 'true' else '-1.5708'"])
    default_imu_base_pitch = PythonExpression(
        ["'0.0' if '", sim, "' == 'true' else '-0.261799'"])

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    imu_base_yaw = LaunchConfiguration('imu_base_yaw')
    imu_base_pitch = LaunchConfiguration('imu_base_pitch')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_sim_cmd = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Simulation hardware profile: picks the sim sensor topics '
                    '(mid360_sim.yaml) and drops the real robot 90 deg lidar '
                    'mounting rotation. Independent of use_sim_time.'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path (defaults by `sim`)'
    )
    declare_imu_base_yaw_cmd = DeclareLaunchArgument(
        'imu_base_yaw', default_value=default_imu_base_yaw,
        description='Yaw of the imu_link->base_link static TF. -1.5708 on the '
                    'real robot (front lidar; imu_link +X points left), 0 in '
                    'simulation.'
    )
    declare_imu_base_pitch_cmd = DeclareLaunchArgument(
        'imu_base_pitch', default_value=default_imu_base_pitch,
        description='Pitch of the imu_link->base_link static TF. -0.261799 on '
                    'the real robot (front lidar tilted 15 deg forward), 0 in '
                    'simulation.'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    
    tf_imu2base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='statictf_imu_to_base_link',
        arguments=['-0.225', '0.26563', '-0.071175',
                   imu_base_yaw, imu_base_pitch, '0', 'imu_link', 'base_link'],
        output='screen'
    )
    
    tf_baselink_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='statictf_imu_to_laser',
        arguments=['-0.011', '-0.02329', '0.04412', '0', '0', '0', 'imu_link', 'livox_frame'],
        output='screen'
    )

    tf_map2odom = Node(
          package='tf2_ros',
          executable='static_transform_publisher',
          name='statictf_map_to_odom',
          # arguments=['-0.327392', '-0.109034', '0.2', '-0.00125676', '0.00582305', '-0.0174636', '0.99983', 'map', 'odom'],
          # arguments=['-0.327392', '-0.109034', '0.0160183', '-0.00125676', '0.00582305', '-0.0174636', '0.99983', 'map', 'odom'],
          arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
          output='screen'
        )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_odometry',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time,
                    'ndt_thresh': 3.0,
                    'feature_extract_enable': feature_extract_enable_param,
                    'point_filter_num': point_filter_num_param,
                    'max_iteration': max_iteration_param,
                    'filter_size_surf': filter_size_surf_param,
                    'filter_size_map': filter_size_map_param,
                    'cube_side_length': cube_side_length_param,
                    'map_file_path': map_file_path,
                    'runtime_pos_log_enable': runtime_pos_log_enable_param,
                    'pcd_save/pcd_save_en': 'false'}],
        output='screen'
    )

    # Standalone chassis EKF: fuses /motion_control/chassis_speed + the IMU
    # + /Odometry (from fastlio_odometry) into /odom.
    # Runs in its own process to keep the laser_mapping CPU isolated.
    #
    # config_path 也喂给它，是为了拿到里面平铺的 `imu_topic`（chassis_ekf_node.cpp
    # 声明的是平铺名，而 FAST-LIO 读的是嵌套的 common.imu_topic，两者互不干扰）。
    # 不喂的话它就用代码里的默认值 /livox/imu —— 真机上根本没有这个话题，底盘 EKF
    # 会静默地一帧 IMU 都收不到。两个 yaml 顶层都是 /**: 通配，所以话题名只需在
    # yaml 里定义一次，两个节点各取自己那份。
    chassis_ekf_node = Node(
        package='fast_lio',
        executable='chassis_ekf',
        name='chassis_ekf',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # cloud2scan = Node(
    #         package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    #         remappings=[('cloud_in', '/cloud_registered_body'),
    #                     ('scan', '/livox_scan')],
    #         parameters=[{
    #             'target_frame': 'livox_frame',
    #             'transform_tolerance': 0.01,
    #             'min_height': -0.1,
    #             'max_height': 0.1,
    #             'angle_min': -1.57,  # -M_PI/2
    #             'angle_max': 3.14,  # M_PI/2
    #             'angle_increment': 0.0087,  # M_PI/360.0
    #             'scan_time': 0.1,
    #             'range_min': 0.1,
    #             'range_max': 10.0,
    #             'use_inf': True,
    #             'inf_epsilon': 1.0
    #         }],
    #         name='pointcloud_to_laserscan'
    #     )

    cloud2scan2 = Node(
        package='cloud_to_laserscan', executable='cloud_to_2d_laserscan',
        parameters=[{
            'cloud_topic': '/cloud_registered_body',
            'elevation_topic': '/grid_map_from_raw_pointcloud',
            'elevation_layer': 'elevation',
            'output_topic': '/livox_scan2',
            'publish_filtered_pcd': False,
            'ground_tolerance': 0.05,
            'max_height': 1.6,
            'angle_min': -1.57,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,  # 0.5 deg
            'range_min': 0.1,
            'range_max': 30.0
        }],
        name='cloud_to_2d_laserscan'
    )

    grid_map_pcl_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_loader_node',
        name='grid_map_pcl_loader_node',
        parameters=[{'map_file_path': map_file_path}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_imu_base_yaw_cmd)
    ld.add_action(declare_imu_base_pitch_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(tf_imu2base_link)
    ld.add_action(tf_baselink_laser)
    ld.add_action(tf_map2odom)
    ld.add_action(fast_lio_node)
    ld.add_action(chassis_ekf_node)
    ld.add_action(rviz_node)
    # ld.add_action(cloud2scan)
    ld.add_action(cloud2scan2)
    ld.add_action(grid_map_pcl_node)

    return ld
