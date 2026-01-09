from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    grid_map_pcl_dir = get_package_share_directory('grid_map_pcl')
    param_file = os.path.join(grid_map_pcl_dir, 'config', 'parameters.yaml')
    
    # 声明命令行参数
    folder_path_arg = DeclareLaunchArgument(
        'folder_path',
        default_value='/home/nvidia/maps/',
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

    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(folder_path_arg)
    ld.add_action(pcd_filename_arg)
    ld.add_action(map_frame_arg)
    ld.add_action(set_verbosity_arg)
    
    # 添加节点
    ld.add_action(pcl_loader_node)

    return ld