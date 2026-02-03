from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    grid_map_pcl_dir = get_package_share_directory('grid_map_pcl')
    param_file = os.path.join(grid_map_pcl_dir, 'config', 'parameters.yaml')
    default_rviz_config_path = os.path.join(
        grid_map_pcl_dir, 'rviz', 'gridmap.rviz')
        
    # 声明命令行参数
    folder_path_arg = DeclareLaunchArgument(
        'folder_path',
        default_value='/home/nvidia/Navi/maps/',
        description='Path to the folder containing PCD files'
    )
    
    pcd_filename_arg = DeclareLaunchArgument(
        'pcd_filename',
        default_value='120w1floor.pcd',
        description='Name of the PCD file to load'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Frame ID for the map'
    )
    
    set_verbosity_arg = DeclareLaunchArgument(
        'set_verbosity_to_debug',
        default_value='false',
        description='Enable debug verbosity'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # 使用 LaunchConfiguration 获取参数值
    node_params = [
        {'folder_path': LaunchConfiguration('folder_path')},
        {'pcd_filename': LaunchConfiguration('pcd_filename')},
        {'map_frame': LaunchConfiguration('map_frame')},
        {'set_verbosity_to_debug': LaunchConfiguration('set_verbosity_to_debug')},
        {'config_file_path': param_file}
    ]

    pcl_loader_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_loader_node',
        name='grid_map_pcl_loader_node',
        output='screen',
        parameters=node_params
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(folder_path_arg)
    ld.add_action(pcd_filename_arg)
    ld.add_action(map_frame_arg)
    ld.add_action(set_verbosity_arg)
    ld.add_action(use_sim_time_arg)
    
    # 添加节点
    ld.add_action(pcl_loader_node)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(rviz_node)

    return ld