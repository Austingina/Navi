import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    default_config_path = os.path.join(package_path, 'config', 'mid360.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio-localize.rviz')


    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
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
        arguments=['-0.16', '-0.26', '0', '0', '0', '0', 'imu_link', 'base_link'],
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
            'ground_tolerance': 0.05,
            'max_height': 2.0,
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
        name='grid_map_pcl_loader',
        parameters=[{'pcd_filename': map_file_path}],
        output='screen'
    )

        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(tf_imu2base_link)
    ld.add_action(tf_baselink_laser)
    ld.add_action(tf_map2odom)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    # ld.add_action(cloud2scan)
    ld.add_action(cloud2scan2)
    ld.add_action(grid_map_pcl_node)

    return ld
