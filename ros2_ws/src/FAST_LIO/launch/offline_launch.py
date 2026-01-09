from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=[
            get_package_share_directory("fast_lio") + '/config/mapper_params_offline.yaml'
          ],
          package='slam_toolbox',
          executable='sync_slam_toolbox_node',
          name='slam_toolbox',
          output='screen'
        )
    ])
