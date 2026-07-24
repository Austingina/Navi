"""Launch the campus route planner and its map frame link.

This is the single launch entry point for campus_nav.  It publishes campus
roads/buildings/paths in `frame_id` (default: campus_map) and, by default, a
static parent_frame -> frame_id transform (default: map -> campus_map).

Robot localization remains separate: your GPS/heading/FAST-LIO stack owns
map -> odom -> base_link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg_source_root() -> str:
    """Return the campus_nav package source directory.

    Derived from this launch file's location so paths stay valid with
    colcon --symlink-install without depending on an external nav_test tree.
    """
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    pkg_root = _pkg_source_root()
    pkg_share = get_package_share_directory("campus_nav")
    default_pbf = os.environ.get(
        "CAMPUS_PBF_PATH",
        os.path.join(pkg_root, "xili_5km_finetuned.osm.pbf"),
    )
    default_gpkg_overlay = os.environ.get(
        "CAMPUS_GPKG_OVERLAY",
        os.path.join(pkg_root, "finetuned_road.gpkg"),
    )
    default_patrol_out = os.path.join(pkg_root, "patrol_routes")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (set true for Isaac Sim)",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="campus_map",
                description="TF frame for campus map markers and paths",
            ),
            # ENU origin the OSM graph / goal lon-lat math is anchored to.
            # Default MUST equal config/route_planner.yaml's datum (the value the
            # graph was actually built with); the outdoor heading-align stack
            # pins the map frame to this same point via gps_heading_align SetDatum.
            DeclareLaunchArgument(
                "datum_lat",
                default_value="22.574357630323675",
                description="Campus datum latitude (ENU origin)",
            ),
            DeclareLaunchArgument(
                "datum_lon",
                default_value="113.93507078035577",
                description="Campus datum longitude (ENU origin)",
            ),
            DeclareLaunchArgument(
                "publish_map_to_campus_tf",
                default_value="true",
                description="Publish static TF parent_frame -> frame_id",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="map",
                description="Parent frame for the campus map static TF",
            ),
            DeclareLaunchArgument(
                "campus_tf_x",
                default_value="-3.0",
                description="Static TF translation x from parent_frame to frame_id",
            ),
            DeclareLaunchArgument(
                "campus_tf_y",
                default_value="-0.5",
                description="Static TF translation y from parent_frame to frame_id",
            ),
            DeclareLaunchArgument(
                "campus_tf_z",
                default_value="0.0",
                description="Static TF translation z from parent_frame to frame_id",
            ),
            DeclareLaunchArgument(
                "campus_tf_yaw",
                default_value="0.0",
                description="Static TF yaw from parent_frame to frame_id (rad)",
            ),
            DeclareLaunchArgument(
                "pbf_path",
                default_value=default_pbf,
                description="OSM PBF road network file",
            ),
            DeclareLaunchArgument(
                "dev_overlay",
                default_value="false",
                description="Overlay gpkg edits on top of pbf at runtime",
            ),
            DeclareLaunchArgument(
                "gpkg_overlay",
                default_value=default_gpkg_overlay,
                description="QGIS-edited road overlay GeoPackage",
            ),
            DeclareLaunchArgument(
                "network_z",
                default_value="-0.3",
                description="Z height for /campus_network and visual /campus_path",
            ),
            DeclareLaunchArgument(
                "network_line_width",
                default_value="0.5",
                description="Line width in meters for /campus_network road markers",
            ),
            DeclareLaunchArgument(
                "network_alpha",
                default_value="0.92",
                description="Alpha for /campus_network road markers",
            ),
            DeclareLaunchArgument(
                "goal_topic",
                default_value="campus_goal_pose",
                description="PoseStamped goal topic for campus road routing",
            ),
            DeclareLaunchArgument(
                "send_to_nav2",
                default_value="true",
                description="Send sampled campus waypoints to Nav2 NavigateThroughPoses",
            ),
            DeclareLaunchArgument(
                "require_heading_lock",
                default_value="false",
                description=(
                    "Hold campus waypoints from Nav2 until /heading_align/status "
                    "reports locked (outdoor GPS heading-align stack). Keep false "
                    "for sim / setups without gps_heading_align."
                ),
            ),
            DeclareLaunchArgument(
                "nav_waypoint_spacing_m",
                default_value="10.0",
                description="Sampling distance for Nav2 waypoints along the campus route",
            ),
            DeclareLaunchArgument(
                "patrol_out_dir",
                default_value=default_patrol_out,
                description="Directory for exported patrol routes",
            ),
            Node(
                package="campus_nav",
                executable="route_planner_node",
                name="campus_route_planner",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "route_planner.yaml"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "datum_lat": LaunchConfiguration("datum_lat"),
                        "datum_lon": LaunchConfiguration("datum_lon"),
                        "pbf_path": LaunchConfiguration("pbf_path"),
                        "dev_overlay": LaunchConfiguration("dev_overlay"),
                        "gpkg_overlay": LaunchConfiguration("gpkg_overlay"),
                        "network_line_width": LaunchConfiguration("network_line_width"),
                        "network_z": LaunchConfiguration("network_z"),
                        "network_alpha": LaunchConfiguration("network_alpha"),
                        "goal_topic": LaunchConfiguration("goal_topic"),
                        "send_to_nav2": LaunchConfiguration("send_to_nav2"),
                        "require_heading_lock": LaunchConfiguration(
                            "require_heading_lock"
                        ),
                        "nav_waypoint_spacing_m": LaunchConfiguration(
                            "nav_waypoint_spacing_m"
                        ),
                        "patrol_out_dir": LaunchConfiguration("patrol_out_dir"),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_campus_map_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_map_to_campus_tf")),
                arguments=[
                    "--x",
                    LaunchConfiguration("campus_tf_x"),
                    "--y",
                    LaunchConfiguration("campus_tf_y"),
                    "--z",
                    LaunchConfiguration("campus_tf_z"),
                    "--yaw",
                    LaunchConfiguration("campus_tf_yaw"),
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    LaunchConfiguration("parent_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("frame_id"),
                ],
            ),
        ]
    )
