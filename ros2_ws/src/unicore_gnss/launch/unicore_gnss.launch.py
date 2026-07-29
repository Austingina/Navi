from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("unicore_gnss")
    default_config = os.path.join(pkg_share, "config", "unicore_gnss.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB4"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        Node(
            package="unicore_gnss",
            executable="unicore_gnss_node",
            name="unicore_gnss",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "port": LaunchConfiguration("port"),
                    "baudrate": LaunchConfiguration("baudrate"),
                },
            ],
        ),
    ])
