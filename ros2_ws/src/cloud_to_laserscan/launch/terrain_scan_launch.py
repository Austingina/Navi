from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        # I/O
        'cloud_topic': '/livox/points',
        'elevation_topic': '/grid_map_from_raw_pointcloud',
        'elevation_layer': 'elevation',
        'output_topic': '/livox_scan2',
        'grid_output_topic': '/terrain_grid_map',
        'slope_cost_topic': '/slope_costmap',
        'base_frame': 'base_link',
        # 地面层: 邻域低分位(0=最小值, 0.5=中位数). 低分位对障碍顶免疫又能跟随坡面.
        'ground_percentile': 0.2,
        'ground_radius': 3,        # (2r+1)^2 窗
        'ground_min_cells': 4,
        # 障碍高度带(相对局部地面)
        'obstacle_min_height': 0.03,
        'obstacle_max_height': 1.6,
        # 地面未知时回退到脚底高度带
        'fallback_on_unknown_elevation': True,
        'fallback_min_height': 0.05,
        'fallback_max_height': 1.6,
        # 坡度 -> 分级代价 OccupancyGrid(/slope_costmap)
        'slope_enable': True,
        'slope_window': 2,
        'slope_free_deg': 5.0,
        'slope_lethal_deg': 25.0,
        'slope_gamma': 1.0,
        # 多帧累积
        'accumulate_enable': True,
        'accumulate_time': 0.8,
        'accumulate_max_frames': 10,
        # LaserScan 几何
        'angle_min': -3.141592653589793,
        'angle_max': 3.141592653589793,
        'angle_increment': 0.008726646259971648,  # 0.5 deg
        'range_min': 0.1,
        'range_max': 30.0,
        'publish_filtered_pcd': False,
    }

    node = Node(
        package='cloud_to_laserscan',
        executable='terrain_scan',
        name='terrain_scan_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([node])
