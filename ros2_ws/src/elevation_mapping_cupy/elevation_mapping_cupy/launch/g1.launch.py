from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    elevation_mapping_cupy_dir = get_package_share_directory('elevation_mapping_cupy')

    return LaunchDescription([
        Node(
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node.py',
            name='elevation_mapping',
            parameters=[
                PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'config',
                    'setups',
                    'g1',
                    'g1_parameters.yaml'
                ]),
                PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'config',
                    'setups',
                    'g1',
                    'g1_sensor_parameter.yaml'
                ])
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_to_d435_broadcaster',
            arguments=[
                '0.05734', '0.01750', '0.01369',
                '0', '0.79064', '0',
                'body', 'camera_link',
            ],
            output='screen'
        )
    ])
