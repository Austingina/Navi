from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'cloud_topic': '/cloud_registered_body',
        'elevation_topic': '/grid_map_from_raw_pointcloud',
        'elevation_layer': 'elevation',
        'output_topic': '/cloud_as_laserscan',
        'publish_filtered_pcd': False,
        'ground_tolerance': 0.05,
        'max_height': 2.0,
        'angle_min': -3.141592653589793,
        'angle_max': 3.141592653589793,
        'angle_increment': 0.008726646259971648,  # 0.5 deg
        'range_min': 0.1,
        'range_max': 30.0
    }

    node = Node(
        package='cloud_to_laserscan',
        executable='cloud_to_2d_laserscan',
        name='cloud_to_2d_laserscan_node',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([node])
