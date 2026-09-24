import json
import math
import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import LabelInfo
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from campus_nav.semantic_lateral_adapter import (
    LateralTarget,
    RoadBoundaryEstimator,
    RouteCorridor,
)
from campus_nav.route_graph import (
    ENU,
    build_graph,
    build_graph_from_pbf,
    build_graph_merged,
    building_footprints_enu,
    extrude_ring_triangles,
    graph_edges_enu,
    load_building_footprints,
    load_entry_points,
    load_stub_entry_points,
    rasterize_occupancy,
    triangulate_fan,
)

# visualization_msgs is optional: only needed for the vector MarkerArray map.
# Install with: sudo apt-get install -y ros-jazzy-visualization-msgs
try:
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_MARKERS = True
except ImportError:
    HAVE_MARKERS = False


def _pkg_data_root() -> str:
    """Return the directory holding campus_nav data files.

    Honors NAV_TEST_ROOT when set; otherwise derives the package source
    directory from this file's location (realpath so colcon
    --symlink-install resolves back to the source tree).
    Layout: .../src/campus_nav/campus_nav/route_planner_node.py
            -> data root: .../src/campus_nav
    """
    root = os.environ.get("NAV_TEST_ROOT", "")
    if root:
        return root
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def _default_pbf_path() -> str:
    return os.environ.get(
        "CAMPUS_PBF_PATH",
        os.path.join(_pkg_data_root(), "xili_5km_with_extra.osm.pbf"),
    )


def _default_gpkg_overlay() -> str:
    return os.environ.get(
        "CAMPUS_GPKG_OVERLAY",
        os.path.join(_pkg_data_root(), "new_road.gpkg"),
    )


def _default_gpkg_path() -> str:
    return os.path.join(_pkg_data_root(), "campus_nav.gpkg")


def _default_entry_gpkg() -> str:
    return os.environ.get(
        "CAMPUS_ENTRY_GPKG",
        os.path.join(_pkg_data_root(), "campus_entries.gpkg"),
    )


def _default_patrol_out_dir() -> str:
    d = os.path.join(_pkg_data_root(), "patrol_routes")
    os.makedirs(d, exist_ok=True)
    return d


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def rviz_qos():
    """Match RViz2 default display QoS (Reliable + Volatile)."""
    return QoSProfile(
        depth=10,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _latest_tf_stamp():
    """ROS stamp 0 means consumers should use the latest available TF."""
    return Time().to_msg()


if HAVE_MARKERS:

    def _edges_to_line_markers(
        edges,
        frame_id,
        stamp,
        line_width=0.5,
        z=0.0,
        rgba=(0.15, 0.55, 0.95, 0.92),
        max_points_per_marker=30000,
    ):
        """LINE_LIST markers: vector geometry stays sharp at any RViz zoom level."""
        markers = []
        chunk = []
        mid = 0

        def flush():
            nonlocal mid, chunk
            if not chunk:
                return
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "road_network"
            m.id = mid
            mid += 1
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = float(line_width)
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.pose.orientation.w = 1.0
            m.points = chunk
            markers.append(m)
            chunk = []

        for e1, n1, e2, n2 in edges:
            if len(chunk) + 2 > max_points_per_marker:
                flush()
            chunk.append(Point(x=float(e1), y=float(n1), z=float(z)))
            chunk.append(Point(x=float(e2), y=float(n2), z=float(z)))
        flush()
        return markers

    def _flat_triangles_to_points(flat):
        pts = []
        for i in range(0, len(flat), 9):
            pts.extend([
                Point(x=float(flat[i]), y=float(flat[i + 1]), z=float(flat[i + 2])),
                Point(x=float(flat[i + 3]), y=float(flat[i + 4]), z=float(flat[i + 5])),
                Point(x=float(flat[i + 6]), y=float(flat[i + 7]), z=float(flat[i + 8])),
            ])
        return pts

    def _buildings_to_extruded_markers(
        enu_footprints,
        frame_id,
        stamp,
        wall_rgba=(0.72, 0.74, 0.78, 0.92),
        max_points_per_marker=30000,
    ):
        """Extruded building meshes (walls + roof) via TRIANGLE_LIST."""
        markers = []
        chunk = []
        mid = 0

        def flush():
            nonlocal mid, chunk
            if not chunk:
                return
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "buildings_3d"
            m.id = mid
            mid += 1
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color = ColorRGBA(
                r=wall_rgba[0], g=wall_rgba[1], b=wall_rgba[2], a=wall_rgba[3],
            )
            m.pose.orientation.w = 1.0
            m.points = chunk
            markers.append(m)
            chunk = []

        for ring, height_m, holes in enu_footprints:
            flat = extrude_ring_triangles(ring, height_m, holes=holes)
            if not flat:
                continue
            tri_pts = _flat_triangles_to_points(flat)
            if len(chunk) + len(tri_pts) > max_points_per_marker:
                flush()
            chunk.extend(tri_pts)
        flush()
        return markers

    def _buildings_to_flat_markers(
        enu_footprints,
        frame_id,
        stamp,
        rgba=(0.82, 0.82, 0.82, 0.88),
        max_points_per_marker=30000,
    ):
        """Flat gray footprints (QGIS-style) via TRIANGLE_LIST markers."""
        markers = []
        chunk = []
        mid = 0

        def flush():
            nonlocal mid, chunk
            if not chunk:
                return
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "buildings"
            m.id = mid
            mid += 1
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.pose.orientation.w = 1.0
            m.points = chunk
            markers.append(m)
            chunk = []

        for ring, _height_m, holes in enu_footprints:
            flat = triangulate_fan(ring, holes=holes)
            if not flat:
                continue
            tri_pts = []
            for i in range(0, len(flat), 6):
                tri_pts.extend([
                    Point(x=float(flat[i]), y=float(flat[i + 1]), z=0.0),
                    Point(x=float(flat[i + 2]), y=float(flat[i + 3]), z=0.0),
                    Point(x=float(flat[i + 4]), y=float(flat[i + 5]), z=0.0),
                ])
            if len(chunk) + len(tri_pts) > max_points_per_marker:
                flush()
            chunk.extend(tri_pts)
        flush()
        return markers

    def _patrol_waypoint_markers(enu_pts, frame_id, stamp, size=4.0):
        """Numbered spheres + labels for the ordered patrol waypoints.

        enu_pts: list of (east, north, label) in map meters.
        size: sphere diameter in meters; label height and both z offsets scale
        with it so the Nav2 route waypoints can be drawn much smaller than the
        patrol ones without looking lopsided.
        """
        sphere_z = size * 0.25
        label_z = size * 1.25
        label_size = max(0.25, size * 0.125)
        markers = []
        # Clear stale markers first so removed waypoints disappear in RViz.
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        # Keep this ns/id distinct from the actual waypoint markers below.
        clear.ns = "patrol_wp_clear"
        clear.action = Marker.DELETEALL
        markers.append(clear)
        for i, (e, n, label) in enumerate(enu_pts):
            sph = Marker()
            sph.header.frame_id = frame_id
            sph.header.stamp = stamp
            sph.ns = "patrol_wp"
            sph.id = i * 2
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = float(e)
            sph.pose.position.y = float(n)
            sph.pose.position.z = sphere_z
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = float(size)
            # charging point (first) green, others orange
            if i == 0:
                sph.color = ColorRGBA(r=0.1, g=0.85, b=0.2, a=1.0)
            else:
                sph.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
            markers.append(sph)

            txt = Marker()
            txt.header.frame_id = frame_id
            txt.header.stamp = stamp
            txt.ns = "patrol_wp"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(e)
            txt.pose.position.y = float(n)
            txt.pose.position.z = label_z
            txt.pose.orientation.w = 1.0
            txt.scale.z = label_size
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = str(label)
            markers.append(txt)
        return markers

    def _patrol_corner_markers(enu_pts, frame_id, stamp,
                               rgba=(0.95, 0.1, 0.85, 1.0), size=2.5):
        """Small spheres at each corner key point (turning points)."""
        markers = []
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        clear.ns = "patrol_corner"
        clear.action = Marker.DELETEALL
        markers.append(clear)
        if not enu_pts:
            return markers
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "patrol_corner"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = float(size)
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(e), y=float(n), z=0.6) for e, n in enu_pts]
        markers.append(m)
        return markers

    def _entry_point_markers(enu_pts, frame_id, stamp, size=1.0,
                             rgba=(0.1, 0.95, 0.9, 0.95)):
        """Cubes + names at the off-network entry points.

        enu_pts: list of (east, north, label) in map meters. Cyan cubes read
        differently from the route/patrol spheres, so a misplaced entry stands
        out against the road network in RViz.
        """
        label_z = size * 2.0
        label_size = max(0.3, size * 0.6)
        markers = []
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        clear.ns = "campus_entry_clear"
        clear.action = Marker.DELETEALL
        markers.append(clear)
        for i, (e, n, label) in enumerate(enu_pts):
            cube = Marker()
            cube.header.frame_id = frame_id
            cube.header.stamp = stamp
            cube.ns = "campus_entry"
            cube.id = i * 2
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(e)
            cube.pose.position.y = float(n)
            cube.pose.position.z = size * 0.5
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = float(size)
            cube.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            markers.append(cube)

            txt = Marker()
            txt.header.frame_id = frame_id
            txt.header.stamp = stamp
            txt.ns = "campus_entry"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(e)
            txt.pose.position.y = float(n)
            txt.pose.position.z = label_z
            txt.pose.orientation.w = 1.0
            txt.scale.z = label_size
            txt.color = ColorRGBA(r=0.8, g=1.0, b=1.0, a=1.0)
            txt.text = str(label)
            markers.append(txt)
        return markers

    # Nav2 route waypoints: amber ahead of the robot, green once driven past.
    NAV_WP_NS = "nav_wp"
    _NAV_WP_TODO = (1.0, 0.8, 0.05, 1.0)
    _NAV_WP_DONE = (0.1, 0.85, 0.2, 1.0)

    def _nav_waypoint_sphere(i, e, n, frame_id, stamp, size, reached):
        """One route waypoint. Same ns/id on every publish, so recolouring a
        waypoint is just an ADD that RViz overwrites in place."""
        rgba = _NAV_WP_DONE if reached else _NAV_WP_TODO
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = NAV_WP_NS
        m.id = i * 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(e)
        m.pose.position.y = float(n)
        m.pose.position.z = size * 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(size)
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        return m

    def _nav_waypoint_markers(enu_pts, frame_id, stamp, size=1.0, reached=0,
                              stale_upto=0, clear_first=True):
        """Spheres + numbers for the whole route, first `reached` of them green.

        stale_upto is the waypoint count of the previous route: ids past the end
        of this one are deleted explicitly. DELETEALL leads the array and would
        normally suffice, but it is one dropped message away from leaving a
        longer previous route on screen forever, since markers never expire.

        clear_first=False repaints a route in place, for progress updates: the
        ns/id are unchanged so RViz overwrites each marker, and skipping the
        DELETEALL avoids a delete/re-add flicker on every waypoint passed.
        """
        label_z = size * 1.25
        label_size = max(0.25, size * 0.125)
        markers = []
        if clear_first:
            clear = Marker()
            clear.header.frame_id = frame_id
            clear.header.stamp = stamp
            # Distinct ns/id from the waypoints below, which this must not cancel.
            clear.ns = "nav_wp_clear"
            clear.action = Marker.DELETEALL
            markers.append(clear)
        for i, (e, n, label) in enumerate(enu_pts):
            markers.append(
                _nav_waypoint_sphere(i, e, n, frame_id, stamp, size, i < reached)
            )
            txt = Marker()
            txt.header.frame_id = frame_id
            txt.header.stamp = stamp
            txt.ns = NAV_WP_NS
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(e)
            txt.pose.position.y = float(n)
            txt.pose.position.z = label_z
            txt.pose.orientation.w = 1.0
            txt.scale.z = label_size
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = str(label)
            markers.append(txt)
        for i in range(len(enu_pts), stale_upto):
            for mid in (i * 2, i * 2 + 1):
                gone = Marker()
                gone.header.frame_id = frame_id
                gone.header.stamp = stamp
                gone.ns = NAV_WP_NS
                gone.id = mid
                gone.action = Marker.DELETE
                markers.append(gone)
        return markers


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("campus_route_planner")

        # source: "pbf"  -> read xili_5km_with_extra.osm.pbf directly (pyosmium)
        #         "gpkg" -> read campus_nav.gpkg
        self.declare_parameter("source", "pbf")
        self.declare_parameter("pbf_path", _default_pbf_path())
        # Dev: runtime merge pbf + gpkg (no need to re-bake). Prod: dev_overlay=false, use merged pbf only.
        self.declare_parameter("dev_overlay", False)
        self.declare_parameter("gpkg_overlay", _default_gpkg_overlay())
        self.declare_parameter("gpkg_overlay_table", "osmfile")
        self.declare_parameter("gpkg_path", _default_gpkg_path())
        self.declare_parameter("table", "route_edges")
        self.declare_parameter("mode", "walk")
        self.declare_parameter("respect_oneway", False)
        # Keep access=private/no roads (campus internal roads the robot owns).
        self.declare_parameter("allow_private", True)
        # Honor no_*/only_* turn restrictions (meaningful for car; ignored on foot).
        self.declare_parameter("respect_turns", False)
        self.declare_parameter("frame_id", "campus_map")
        # Datum = center of the dense campus area (see build_and_test.py output).
        # NOTE: config/route_planner.yaml overrides these with the real computed
        # center (22.574357630323675, 113.93507078035577); keep them in sync.
        self.declare_parameter("datum_lat", 22.574357630323675)
        self.declare_parameter("datum_lon", 113.93507078035577)
        self.declare_parameter("max_snap_m", 50.0)  # warn if start/goal far from a road
        # Road-network visualization for RViz.
        # OccupancyGrid blurs when zoomed out on large maps; use vector network instead.
        self.declare_parameter("publish_map", False)      # OccupancyGrid raster (optional)
        self.declare_parameter("publish_network", True)   # MarkerArray vector lines (sharp)
        self.declare_parameter("publish_buildings", True)  # building footprints
        self.declare_parameter("building_extruded", True)   # 3D walls+roof vs flat fill
        self.declare_parameter("building_default_height", 12.0)  # m, if OSM has no height
        self.declare_parameter("building_level_height", 3.0)   # m per building:levels
        # Empty -> reuse pbf_path. The merged pbf now contains buildings too,
        # so a single file covers both routing and building visualization.
        self.declare_parameter("buildings_pbf_path", "")
        self.declare_parameter("map_resolution", 3.0)     # meters/cell (only if publish_map)
        self.declare_parameter("network_line_width", 1.0)  # line width in meters
        self.declare_parameter("network_z", -0.3)         # lower road/path viz below robot layers
        self.declare_parameter("network_alpha", 0.92)
        # One-click campus routing: goal from RViz, start from current robot TF.
        self.declare_parameter("goal_navigation_enable", True)
        self.declare_parameter("goal_topic", "campus_goal_pose")
        # 外部取消当前道路级路线：收到一个 Empty 即中止路线、撤销已下发的 Nav2
        # 目标并停止批量重发。NaviSkill 的 pause/cancel_navigation 室外分支用它，
        # 因为规划器是定时器持续喂 navigate_through_poses，只在 Nav2 侧取消会被
        # 下一拍重新喂上（robot 继续走）。
        self.declare_parameter("cancel_topic", "campus_goal_cancel")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("nav_waypoints_topic", "campus_nav_waypoints")
        self.declare_parameter(
            "nav_waypoint_markers_topic", "campus_nav_waypoint_markers"
        )
        self.declare_parameter("nav_waypoint_spacing_m", 10.0)
        # Only these OSM highway types are motor roads for lateral waypoint
        # policy. Pedestrian ways are deliberately excluded.
        self.declare_parameter(
            "road_right_offset_highways",
            ["service", "residential", "tertiary", "tertiary_link"],
        )
        self.declare_parameter("road_right_offset_m", 0.0)
        # Phase 2.1 semantic lateral adapter. Closed loop is the default after
        # bag validation: motor-road targets replace the biased OSM waypoint,
        # while pedestrian/unlisted ways continue to use their reference route.
        self.declare_parameter("semantic_lateral_enabled", True)
        self.declare_parameter("semantic_lateral_shadow_mode", False)
        self.declare_parameter(
            "semantic_lateral_segmentation_topic",
            "/semantic/camera1/segmentation",
        )
        self.declare_parameter(
            "semantic_lateral_confidence_topic",
            "/semantic/camera1/confidence",
        )
        self.declare_parameter(
            "semantic_lateral_cloud_topic", "/semantic/camera1/points"
        )
        self.declare_parameter(
            "semantic_lateral_labels_topic", "/semantic/camera1/label_info"
        )
        self.declare_parameter("semantic_lateral_road_class", "road")
        self.declare_parameter("semantic_lateral_min_pixel_confidence", 128)
        self.declare_parameter("semantic_lateral_pixel_stride", 2)
        self.declare_parameter("semantic_lateral_image_edge_margin_px", 8)
        self.declare_parameter("semantic_lateral_max_frame_age_s", 0.8)
        self.declare_parameter("semantic_lateral_near_m", 1.0)
        self.declare_parameter("semantic_lateral_far_m", 7.0)
        self.declare_parameter("semantic_lateral_bin_size_m", 0.5)
        self.declare_parameter("semantic_lateral_limit_m", 12.0)
        self.declare_parameter("semantic_lateral_min_points_per_bin", 20)
        self.declare_parameter("semantic_lateral_width_min_m", 2.5)
        self.declare_parameter("semantic_lateral_width_max_m", 15.0)
        self.declare_parameter("semantic_lateral_lookahead_m", 4.0)
        self.declare_parameter("semantic_lateral_min_lookahead_m", 0.6)
        self.declare_parameter(
            "semantic_lateral_safety_lookahead_step_m", 0.25
        )
        # Must stay above Nav2's general_goal_checker.xy_goal_tolerance.  Route
        # arc distance can greatly exceed Euclidean distance at a sharp offset
        # junction, so a nominally forward target may already count as reached.
        self.declare_parameter("semantic_lateral_min_goal_distance_m", 1.3)
        self.declare_parameter("semantic_lateral_inside_margin_m", 0.75)
        self.declare_parameter("semantic_lateral_min_target_confidence", 0.45)
        self.declare_parameter("semantic_lateral_hold_last_s", 1.0)
        self.declare_parameter("semantic_lateral_acquire_frames", 3)
        self.declare_parameter("semantic_lateral_max_lateral_step_m", 0.4)
        self.declare_parameter("semantic_lateral_filter_alpha", 0.25)
        self.declare_parameter("semantic_lateral_fit_max_slope", 0.35)
        self.declare_parameter("semantic_lateral_center_innovation_m", 0.6)
        self.declare_parameter("semantic_lateral_width_innovation_m", 0.6)
        self.declare_parameter("semantic_lateral_junction_guard_before_m", 4.0)
        self.declare_parameter("semantic_lateral_junction_guard_after_m", 3.0)
        self.declare_parameter("semantic_lateral_junction_release_frames", 3)
        self.declare_parameter("semantic_lateral_tangent_window_m", 1.5)
        self.declare_parameter("semantic_lateral_target_move_m", 0.35)
        self.declare_parameter("semantic_lateral_target_progress_m", 1.0)
        # A rearward route cannot produce forward-route semantic points until
        # the front camera faces it.  Rotate in place first, then reacquire.
        self.declare_parameter("semantic_lateral_align_heading_enable", True)
        self.declare_parameter("semantic_lateral_align_enter_deg", 60.0)
        self.declare_parameter("semantic_lateral_align_exit_deg", 20.0)
        self.declare_parameter("semantic_lateral_align_action_name", "spin")
        self.declare_parameter("semantic_lateral_align_timeout_s", 45.0)
        self.declare_parameter("semantic_lateral_align_max_retries", 1)
        # The merged local costmap already contains Livox obstacles and terrain;
        # use it as the final footprint gate rather than trusting road semantics.
        self.declare_parameter(
            "semantic_lateral_safety_costmap_topic", "/local_costmap/costmap"
        )
        self.declare_parameter("semantic_lateral_safety_radius_m", 0.75)
        self.declare_parameter("semantic_lateral_safety_inward_step_m", 0.2)
        self.declare_parameter("semantic_lateral_avoid_max_lookahead_m", 5.0)
        self.declare_parameter("semantic_lateral_avoid_release_frames", 3)
        self.declare_parameter("semantic_lateral_lethal_occupancy", 99)
        self.declare_parameter("semantic_lateral_costmap_max_age_s", 1.0)
        self.declare_parameter(
            "semantic_lateral_debug_topic", "/campus_lateral/debug"
        )
        self.declare_parameter(
            "semantic_lateral_target_topic", "/campus_lateral/adapted_path"
        )
        self.declare_parameter(
            "semantic_lateral_status_topic", "/campus_lateral/status"
        )
        # Sphere diameter (m) of the RViz route waypoint markers. Labels and z
        # offsets scale with it. Patrol waypoints keep their own larger size.
        self.declare_parameter("nav_waypoint_marker_size_m", 0.8)
        self.declare_parameter("send_to_nav2", True)
        self.declare_parameter("nav2_action_name", "navigate_through_poses")
        self.declare_parameter("nav2_behavior_tree", "")
        # Heading-lock gate: outdoors the `map` frame is only north-aligned after
        # gps_heading_align first locks. Until then, GPS/OSM goals converted into
        # `map` are rotated by the (arbitrary) LIO startup heading, so driving to
        # them sends the robot the wrong way. When require_heading_lock is true we
        # hold campus waypoints back from Nav2 until /heading_align/status reports
        # locked. Default false so the sim stack (no heading_align node) is
        # unaffected: without the topic it would otherwise never send.
        self.declare_parameter("require_heading_lock", False)
        self.declare_parameter("heading_status_topic", "/heading_align/status")
        # On the first heading lock the map->odom TF snaps (rotates by yaw_offset),
        # so everything accumulated in the map frame before the snap (global
        # costmap obstacle marks, persistent terrain cost) is now misplaced.
        # Clear the costmaps once at that moment so they rebuild north-aligned.
        self.declare_parameter("clear_costmaps_on_lock", True)
        # Delay before the second clear. gps_heading_align sends the status
        # ahead of the yaw_offset in the same tick, and map_odom_from_gps only
        # broadcasts the rotated map->odom one tick later still, so the first
        # clear always lands before the snap and the costmaps keep writing under
        # the old transform for up to a TF period. One more clear once that has
        # settled removes what slipped through. 0 disables the second pass.
        self.declare_parameter("clear_costmaps_settle_s", 1.0)
        self.declare_parameter(
            "global_costmap_clear_service",
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.declare_parameter(
            "local_costmap_clear_service",
            "/local_costmap/clear_entirely_local_costmap",
        )
        # Feed the route to Nav2 in rolling batches so every goal stays inside
        # the (rolling-window) global costmap. Disable to send the whole route.
        self.declare_parameter("nav2_batch_enable", True)
        # Max straight-line distance (m) from the robot for a waypoint to be
        # included in the current batch. Keep it < global_costmap half-size.
        self.declare_parameter("nav2_batch_radius_m", 90.0)
        # When the robot is within this distance (m) of the batch's last
        # waypoint, extend/send the next batch for seamless driving.
        self.declare_parameter("nav2_batch_advance_m", 8.0)
        # A waypoint within this distance (m) counts as reached (cursor moves on).
        # Passed waypoints (overshot along the path) also advance the cursor.
        self.declare_parameter("nav2_reach_tol_m", 3.0)
        # How often (s) to check progress and refresh the batch.
        self.declare_parameter("nav2_batch_period_s", 1.0)
        # -- final-goal specialization --
        # The route end is not a fly-through waypoint: nav2_reach_tol_m would
        # drop it (and cancel Nav2) while still metres away. Instead the cursor
        # stops one short of it and the last pose is driven as a separate
        # NavigateToPose goal, whose behavior tree has no RemovePassedGoals and
        # can select a tighter goal checker. Disable to restore the old path.
        self.declare_parameter("nav2_final_precise_enable", True)
        # Distance (m) to the final pose at which the precise approach starts.
        self.declare_parameter("nav2_final_approach_m", 8.0)
        # Fallback completion distance (m) for the final pose. Keep it below the
        # goal checker tolerance so Nav2 normally reports success on its own.
        self.declare_parameter("nav2_final_reach_tol_m", 0.5)
        self.declare_parameter("nav2_final_action_name", "navigate_to_pose")
        # Absolute path to the final-approach behavior tree (injected by launch).
        self.declare_parameter("nav2_final_behavior_tree", "")
        # Re-sends of the final goal after a Nav2 failure before giving up.
        self.declare_parameter("nav2_final_max_retry", 2)
        # -- off-network goals (plaza / courtyard behind a fence) --
        # Such a goal has no road under it and the fence is not mapped, so
        # snapping it to the geometrically nearest segment can pick the sidewalk
        # on the wrong side. Instead the route ends at a hand-authored entry
        # point and the last leg is driven as one Nav2 goal.
        self.declare_parameter("entry_enable", True)
        self.declare_parameter("entry_gpkg", _default_entry_gpkg())
        self.declare_parameter("entry_table", "entries")
        # A goal further than this (m) from any road counts as off-network.
        self.declare_parameter("off_network_m", 5.0)
        # Only entries this close (m) to the goal are considered; a per-entry
        # radius_m column in the layer overrides it. This is also the cap on the
        # access leg, so it must stay under the global costmap's reach: that
        # costmap is a rolling window centred on the robot (200x200 m), and
        # NavfnPlanner throws GoalOutsideMapBounds for a goal outside it, i.e.
        # more than 100 m away. 99.5 avoids putting a goal exactly on the edge; a
        # goal further than this from every entry warns instead, which is the
        # cue to add another entry point rather than to raise the radius.
        self.declare_parameter("entry_search_radius_m", 99.5)
        # Weight on the off-road leg when comparing entries. >1 favors the entry
        # that leaves less driving outside the road network, even if the road
        # detour to reach it is longer.
        self.declare_parameter("entry_offroad_weight", 1.0)
        self.declare_parameter("reject_entry_network_crossing", True)
        self.declare_parameter("entry_markers_topic", "campus_entry_markers")
        self.declare_parameter("entry_marker_size_m", 1.0)
        # -- road network vs. direct drive --------------------------------------
        # Following the roads is the safe default: the graph encodes topology the
        # robot cannot see, so it never drives into a fenced dead end. But for a
        # goal a few metres away, or one the roads only reach by going around the
        # block, snapping to the network is a pointless excursion. plan_auto
        # weighs the two; see RouteGraph.plan_auto for the ordered rules.
        self.declare_parameter("auto_channel_enable", True)
        # Hard ceiling on any hop handed to Nav2 as a single goal. The global
        # costmap is a rolling 200x200 m window centred on the robot, so
        # NavfnPlanner throws GoalOutsideMapBounds past ~100 m; 99.5 avoids the
        # exact boundary. Same reasoning as entry_search_radius_m —
        # keep the two in step.
        self.declare_parameter("direct_max_m", 99.5)
        # Below this, drive straight there unconditionally: projecting onto a road
        # and back off it costs more than the trip itself.
        self.declare_parameter("always_direct_m", 8.0)
        # Drive direct once the road route is this many times the straight line.
        # 1.0 would always go direct; very large values always follow the roads.
        self.declare_parameter("detour_ratio_threshold", 1.6)
        # An on-road leg no longer than this means both ends attach to the road
        # at the same place, i.e. the route would leave and re-enter through one
        # opening. Kept as a length rather than an entry-name match: names are
        # neither unique nor required, and two entries a metre apart should
        # behave identically.
        self.declare_parameter("same_entry_road_m", 2.0)
        self.declare_parameter("decision_topic", "campus_route_decision")
        # -- patrol route picking (RViz "Publish Point" -> ordered waypoints) --
        self.declare_parameter("patrol_enable", True)
        self.declare_parameter("patrol_name", "patrol_01")
        self.declare_parameter("patrol_out_dir", _default_patrol_out_dir())
        # Close the loop back to the first waypoint (charging point start/end).
        self.declare_parameter("patrol_loop", True)
        # Control points: evenly spaced samples along the route (meters).
        self.declare_parameter("patrol_sample_m", 5.0)
        # Corner key points: keep a route vertex only if heading turns more than
        # this (deg). Endpoints + clicked waypoints are always kept.
        self.declare_parameter("patrol_corner_angle_deg", 12.0)
        # Control channel: set with `ros2 param set ... patrol_cmd <cmd>`.
        #   clear / undo / save / close  (close = close loop + save) / load
        self.declare_parameter("patrol_cmd", "")
        # Existing patrol json to load & visualize (path + corners + waypoints).
        self.declare_parameter("patrol_load", "")

        source = self.get_parameter("source").value
        mode = self.get_parameter("mode").value
        respect_oneway = self.get_parameter("respect_oneway").value
        allow_private = self.get_parameter("allow_private").value
        self.respect_turns = self.get_parameter("respect_turns").value
        self.frame_id = self.get_parameter("frame_id").value
        self.enu = ENU(
            self.get_parameter("datum_lat").value,
            self.get_parameter("datum_lon").value,
        )
        self.max_snap_m = self.get_parameter("max_snap_m").value

        if source == "pbf":
            pbf = self.get_parameter("pbf_path").value
            dev_overlay = self.get_parameter("dev_overlay").value
            overlay = self.get_parameter("gpkg_overlay").value if dev_overlay else ""
            if dev_overlay and overlay:
                table = self.get_parameter("gpkg_overlay_table").value
                self.get_logger().info(
                    f"[dev] Loading merged graph: {pbf} + overlay {overlay} ..."
                )
                self.graph = build_graph_merged(
                    pbf, overlay, gpkg_table=table, mode=mode,
                    respect_oneway=respect_oneway, allow_private=allow_private,
                )
            else:
                if dev_overlay and not overlay:
                    self.get_logger().warn(
                        "dev_overlay=true but gpkg_overlay is empty; using pbf only."
                    )
                self.get_logger().info(f"Loading graph from {pbf} (mode={mode}) ...")
                self.graph = build_graph_from_pbf(
                    pbf, mode=mode, respect_oneway=respect_oneway,
                    allow_private=allow_private,
                )
        else:
            gpkg = self.get_parameter("gpkg_path").value
            table = self.get_parameter("table").value
            self.get_logger().info(f"Loading graph from {gpkg} (mode={mode}) ...")
            self.graph = build_graph(
                gpkg, table=table, mode=mode, respect_oneway=respect_oneway,
                allow_private=allow_private,
            )
        self.get_logger().info(f"Graph ready: {self.graph.stats}")
        if self.respect_turns and not getattr(self.graph, "restrictions", None):
            self.get_logger().warn("respect_turns=True but no turn restrictions loaded.")

        self.start_lonlat = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, "campus_path", rviz_qos())
        self.nav_waypoints_pub = self.create_publisher(
            Path, self.get_parameter("nav_waypoints_topic").value, rviz_qos()
        )
        if HAVE_MARKERS:
            # Latched: markers never expire, so an RViz started mid-route needs
            # the last array replayed. Transient local also stays compatible
            # with volatile subscribers, unlike the reverse.
            self.nav_wp_marker_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter("nav_waypoint_markers_topic").value,
                latched_qos(),
            )
            # Latched: entries are static, so RViz picks them up whenever it
            # subscribes rather than only on the next route.
            self.entry_marker_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter("entry_markers_topic").value,
                latched_qos(),
            )
        # Latched: the last decision stays readable for anything that subscribes
        # after the goal was already dispatched.
        self.decision_pub = self.create_publisher(
            String, self.get_parameter("decision_topic").value, latched_qos()
        )
        self.entries = []
        if self.get_parameter("entry_enable").value:
            self._load_entry_points()
        self.nav_action_client = None
        self.final_action_client = None
        self.route_spin_action_client = None
        # -- rolling waypoint batching state --
        self._nav_route_poses = []      # full sampled route (PoseStamped, frame_id)
        self._nav_cursor = 0            # index of next waypoint to still pursue
        self._nav_running = False       # a route is currently being executed
        self._nav_goal_active = False   # a Nav2 goal is accepted & executing
        self._nav_send_pending = False  # a goal was sent, awaiting accept/reject
        self._nav_goal_handle = None
        self._nav_goal_seq = 0          # bump on each send; ignore stale result CBs
        self._nav_batch_last_xy = None  # xy of last waypoint in the sent batch
        self._nav_sent_key = None       # (cursor, batch_end) of last sent batch
        self._nav_last_succeeded_key = None
        self._nav_last_succeeded_route_s = None
        # "re-send the batch on the next tick, whatever the advance gate says".
        # Deliberately NOT expressed as _nav_sent_key = None: that reads as "we
        # have not sent anything", which makes stale_active False and hands the
        # decision straight back to the nav2_batch_advance_m gate -- i.e. it
        # silently *suppresses* the re-send it was meant to force.
        self._nav_force_resend = False
        self._nav_batch_timer = None
        # Index of the entry pose when the route ends with an off-network access
        # leg (entry -> goal); None for an ordinary on-road route.
        self._nav_entry_index = None
        # Which channel plan_auto picked for the route being driven:
        # 'road' | 'direct' | 'direct_via_entry'. Only affects log wording.
        self._nav_mode = "road"
        # -- waypoint marker state (progress colouring) --
        self._nav_marker_n = 0        # waypoints in the published marker array
        self._nav_marker_reached = 0  # how many of them are already green
        # -- final-goal (precise approach) state --
        self._final_phase = False       # last pose is being driven on its own
        self._final_goal_active = False  # NavigateToPose sent, not yet finished
        self._final_goal_handle = None
        self._final_goal_seq = 0        # bump on each send; ignore stale results
        self._final_retry = 0
        self._final_cancel_pending = False  # waiting for the batch goal to end
        self._final_wait_ticks = 0
        # Initial camera/route heading alignment. This is route-local and must
        # be reset whenever a new Campus route replaces the old one.
        self._route_align_checked = False
        self._route_align_required = False
        self._route_align_complete = False
        self._route_align_pending = False
        self._route_align_active = False
        self._route_align_goal_handle = None
        self._route_align_seq = 0
        self._route_align_retries = 0
        self._route_align_error_rad = None
        # -- semantic route-frame lateral adapter (phase 2.1) --
        self._lateral_route = None
        self._lateral_s = 0.0
        self._lateral_robot_d = 0.0
        self._lateral_robot_xy = None
        self._lateral_target = None
        self._lateral_target_stamp_ns = None
        self._lateral_target_seq = 0
        self._lateral_seq_d = None
        self._lateral_seq_s = None
        self._lateral_state = "REFERENCE_ONLY"
        self._lateral_reason = "no_route"
        self._lateral_good_frames = 0
        self._lateral_costmap = None
        self._lateral_safety_ok = False
        self._lateral_safety_reason = "safety_costmap_missing"
        self._lateral_goal_geometry_ok = False
        self._lateral_last_corridor = None
        self._lateral_avoid_active = False
        self._lateral_avoid_d = None
        self._lateral_avoid_clear_frames = 0
        self._lateral_pending = {}
        self._lateral_future = None
        self._lateral_worker = None
        self._lateral_last_debug_points = np.empty((0, 2), dtype=np.float64)
        # Diagnostics are edge-triggered: planner decisions remain visible in
        # the launch terminal without flooding it at the status timer rate.
        self._lateral_last_log_signature = None
        self._lateral_last_junction_guard = False
        self._lateral_dispatch_blocked_signature = None
        self._init_semantic_lateral_adapter()
        # -- heading-lock state --
        # Two separate things, deliberately not one flag:
        #   _heading_locked      the physical fact — has the heading aligned yet
        #   _require_heading_lock the policy — do we hold goals until it does
        # Conflating them breaks the costmap clear: with the gate off, a flag
        # pre-set to "locked" means the lock transition never fires.
        self._require_heading_lock = bool(
            self.get_parameter("require_heading_lock").value
        )
        # Latch: once aligned it stays aligned (gps_heading_align never unlocks;
        # frozen_bad_gps still keeps a valid frozen heading offset).
        self._heading_locked = False
        # Whether any heading status has arrived at all. Distinguishes "waiting
        # for a lock that is coming" from "no gps_heading_align in this stack"
        # (the sim), so the drive-before-lock warning does not spam the latter.
        self._heading_status_seen = False
        # Costmap-clear service clients (created lazily so we don't pay for them
        # when the feature is off). Populated on first use in _clear_costmaps().
        self._clear_costmaps_on_lock = bool(
            self.get_parameter("clear_costmaps_on_lock").value
        )
        # One-shot timer for the post-snap clear; cancelled as soon as it fires.
        self._settle_clear_timer = None
        self._global_clear_cli = None
        self._local_clear_cli = None
        # Subscribe whenever the topic is configured, independent of the gate
        # policy: the lock transition is what triggers the costmap clear, and
        # that is needed *more* when goals are allowed through early — the robot
        # will have driven on a wrong heading and filled the costmaps with data
        # the snap then displaces.
        status_topic = self.get_parameter("heading_status_topic").value
        if status_topic:
            self.create_subscription(
                String, status_topic, self._on_heading_status, 10
            )
            if self._require_heading_lock:
                self.get_logger().warn(
                    f"require_heading_lock: campus goals will be held from Nav2 "
                    f"until '{status_topic}' reports locked."
                )
            else:
                self.get_logger().warn(
                    f"require_heading_lock=false: campus goals go to Nav2 even "
                    f"before '{status_topic}' reports locked. The robot may "
                    f"initially drive the wrong way — the heading needs ~3 m of "
                    f"motion to lock, which holding the goals would prevent. "
                    f"Costmaps are cleared on lock."
                )
        if self.get_parameter("send_to_nav2").value:
            self.nav_action_client = ActionClient(
                self,
                NavigateThroughPoses,
                self.get_parameter("nav2_action_name").value,
            )
            self.route_spin_action_client = ActionClient(
                self,
                Spin,
                self.get_parameter(
                    "semantic_lateral_align_action_name"
                ).value,
            )
            if self.get_parameter("nav2_final_precise_enable").value:
                self.final_action_client = ActionClient(
                    self,
                    NavigateToPose,
                    self.get_parameter("nav2_final_action_name").value,
                )
            if self.get_parameter("nav2_batch_enable").value:
                period = float(self.get_parameter("nav2_batch_period_s").value)
                self._nav_batch_timer = self.create_timer(
                    max(0.1, period), self._nav_batch_tick
                )
        if self.get_parameter("goal_navigation_enable").value:
            goal_topic = self.get_parameter("goal_topic").value
            self.create_subscription(PoseStamped, goal_topic, self._on_campus_goal, 10)
            self.get_logger().info(
                f"Campus goal routing enabled: set goal on /{goal_topic}; "
                f"start comes from TF {self.frame_id}->"
                f"{self.get_parameter('robot_frame').value}."
            )
            cancel_topic = self.get_parameter("cancel_topic").value
            if cancel_topic:
                self.create_subscription(
                    Empty, cancel_topic, self._on_campus_cancel, 10
                )
                self.get_logger().info(
                    f"Campus route cancel enabled: publish std_msgs/Empty to "
                    f"/{cancel_topic} to abort the active route."
                )

        # -- patrol route picking --
        self.patrol_pts = []  # ordered list of (lon, lat)
        self.patrol_path_pub = self.create_publisher(
            Path, "patrol_path", latched_qos()
        )
        if HAVE_MARKERS:
            self.patrol_wp_pub = self.create_publisher(
                MarkerArray, "patrol_waypoints", latched_qos()
            )
            self.patrol_corner_pub = self.create_publisher(
                MarkerArray, "patrol_corners", latched_qos()
            )
        if self.get_parameter("patrol_enable").value:
            self.create_subscription(
                PointStamped, "clicked_point", self._on_clicked_point, 10
            )
            # Topic command channel (works without the `ros2 param` CLI plugin):
            #   ros2 topic pub --once /patrol_cmd std_msgs/String "{data: close}"
            self.create_subscription(
                String, "patrol_cmd", self._on_patrol_cmd_topic, 10
            )
        self.add_on_set_parameters_callback(self._on_set_params)

        self._map_msg = None
        self._net_msg = None
        self._bldg_msg = None
        self._republish_timer = None
        self._publish_network_views()
        # One-shot republish for RViz opened after node startup.
        self._republish_timer = self.create_timer(3.0, self._republish_network_views_once)

        self.get_logger().info(
            "Ready. Campus goal: publish PoseStamped to /"
            f"{self.get_parameter('goal_topic').value}; "
            "Patrol: 'Publish Point' to drop ordered waypoints (1st = charging "
            "start), then save with EITHER\n"
            "  ros2 topic pub --once /patrol_cmd std_msgs/msg/String \"{data: close}\"\n"
            "  ros2 param set /campus_route_planner patrol_cmd close"
        )

    def _republish_network_views_once(self):
        if self._map_msg is not None and hasattr(self, "map_pub"):
            self._map_msg.header.stamp = _latest_tf_stamp()
            self.map_pub.publish(self._map_msg)
        if self._net_msg is not None and hasattr(self, "net_pub"):
            for m in self._net_msg.markers:
                m.header.stamp = _latest_tf_stamp()
            self.net_pub.publish(self._net_msg)
        if self._bldg_msg is not None and hasattr(self, "bldg_pub"):
            for m in self._bldg_msg.markers:
                m.header.stamp = _latest_tf_stamp()
            self.bldg_pub.publish(self._bldg_msg)
        if self._republish_timer is not None:
            self._republish_timer.cancel()
            self._republish_timer = None
        self.get_logger().info(
            "Republished /campus_map, /campus_network, /campus_buildings for RViz."
        )

    # -- road-network visualization -----------------------------------------
    def _publish_network_views(self):
        want_map = self.get_parameter("publish_map").value
        want_net = self.get_parameter("publish_network").value
        want_bldg = self.get_parameter("publish_buildings").value
        if not (want_map or want_net or want_bldg):
            return
        edges = graph_edges_enu(self.graph, self.enu) if (want_map or want_net) else []

        if want_map:
            self.map_pub = self.create_publisher(
                OccupancyGrid, "campus_map", rviz_qos()
            )
            res = self.get_parameter("map_resolution").value
            grid, res, oe, on, w, h = rasterize_occupancy(edges, resolution=res)
            msg = OccupancyGrid()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = _latest_tf_stamp()
            msg.info.resolution = float(res)
            msg.info.width = w
            msg.info.height = h
            msg.info.origin.position.x = float(oe)
            msg.info.origin.position.y = float(on)
            msg.info.origin.orientation.w = 1.0
            msg.data = grid.flatten().tolist()
            self._map_msg = msg
            self.map_pub.publish(msg)
            cx = oe + w * res / 2
            cy = on + h * res / 2
            self.get_logger().info(
                f"Published OccupancyGrid /campus_map: {w}x{h} @ {res:.1f} m/cell."
            )
            self.get_logger().info(
                f"Map center (map frame): x={cx:.0f} m, y={cy:.0f} m "
                f"(size {w * res:.0f} x {h * res:.0f} m). "
                "Raster map may blur when zoomed out; prefer /campus_network."
            )

        if want_net:
            if not HAVE_MARKERS:
                self.get_logger().warn(
                    "publish_network=True but visualization_msgs missing; "
                    "install ros-jazzy-visualization-msgs to see the vector map."
                )
                return
            self.net_pub = self.create_publisher(
                MarkerArray, "campus_network", rviz_qos()
            )
            stamp = _latest_tf_stamp()
            lw = self.get_parameter("network_line_width").value
            z = self.get_parameter("network_z").value
            alpha = self.get_parameter("network_alpha").value
            markers = _edges_to_line_markers(
                edges, self.frame_id, stamp, line_width=lw, z=z,
                rgba=(0.15, 0.55, 0.95, alpha),
            )
            self._net_msg = MarkerArray(markers=markers)
            self.net_pub.publish(self._net_msg)
            self.get_logger().info(
                f"Published MarkerArray /campus_network: {len(edges)} edges "
                f"in {len(markers)} marker chunk(s) (vector, sharp at any zoom)."
            )

        if want_bldg:
            if not HAVE_MARKERS:
                self.get_logger().warn(
                    "publish_buildings=True but visualization_msgs missing."
                )
            else:
                bldg_pbf = self.get_parameter("buildings_pbf_path").value
                if not bldg_pbf:
                    bldg_pbf = self.get_parameter("pbf_path").value
                extruded = self.get_parameter("building_extruded").value
                default_h = self.get_parameter("building_default_height").value
                level_h = self.get_parameter("building_level_height").value
                self.get_logger().info(
                    f"Loading buildings from {bldg_pbf} "
                    f"({'extruded 3D' if extruded else 'flat'}) ..."
                )
                footprints = load_building_footprints(
                    bldg_pbf, default_height_m=default_h, level_height_m=level_h,
                )
                enu_fps = building_footprints_enu(footprints, self.enu)
                self.bldg_pub = self.create_publisher(
                    MarkerArray, "campus_buildings", rviz_qos()
                )
                stamp = _latest_tf_stamp()
                if extruded:
                    markers = _buildings_to_extruded_markers(
                        enu_fps, self.frame_id, stamp,
                    )
                else:
                    markers = _buildings_to_flat_markers(
                        enu_fps, self.frame_id, stamp,
                    )
                self._bldg_msg = MarkerArray(markers=markers)
                self.bldg_pub.publish(self._bldg_msg)
                h_min = min(h for _r, h, _holes in enu_fps)
                h_max = max(h for _r, h, _holes in enu_fps)
                self.get_logger().info(
                    f"Published MarkerArray /campus_buildings: {len(enu_fps)} "
                    f"footprints in {len(markers)} marker chunk(s), "
                    f"height {h_min:.1f}-{h_max:.1f} m."
                )

    # -- off-network entry points --------------------------------------------
    def _load_entry_points(self):
        """Collect the places where the robot may leave the road network."""
        gpkg = self.get_parameter("entry_gpkg").value
        table = self.get_parameter("entry_table").value
        entries = load_entry_points(gpkg, table=table)
        if entries:
            self.get_logger().info(
                f"Loaded {len(entries)} entry point(s) from {gpkg}:{table}: "
                f"{', '.join(e['name'] for e in entries)}."
            )
        else:
            self.get_logger().info(
                f"No entry points in {gpkg}:{table} (layer missing or empty). "
                "Off-network goals will snap to the nearest road as before."
            )
        stubs = []
        if self.get_parameter("source").value == "pbf":
            stubs = load_stub_entry_points(
                self.get_parameter("pbf_path").value, self.graph
            )
            if stubs:
                self.get_logger().info(
                    f"Loaded {len(stubs)} entry anchor(s) from campus_entry stub "
                    f"way(s): {', '.join(s['name'] for s in stubs)}."
                )
        self.entries = entries + stubs
        self._publish_entry_markers()

    def _publish_entry_markers(self):
        if not (HAVE_MARKERS and hasattr(self, "entry_marker_pub")):
            return
        stamp = _latest_tf_stamp()
        enu_pts = []
        for e in self.entries:
            east, north = self.enu.to_xy(e["lon"], e["lat"])
            enu_pts.append((east, north, e["name"]))
        size = float(self.get_parameter("entry_marker_size_m").value)
        self.entry_marker_pub.publish(
            MarkerArray(
                markers=_entry_point_markers(
                    enu_pts, self.frame_id, stamp, size=size
                )
            )
        )

    # -- callbacks -----------------------------------------------------------
    def _on_campus_goal(self, msg: PoseStamped):
        start_lonlat = self._current_robot_lonlat()
        if start_lonlat is None:
            return
        goal_xy = self._pose_xy_in_frame(msg, self.frame_id)
        if goal_xy is None:
            return
        goal_lonlat = self._snap("campus goal", goal_xy[0], goal_xy[1])
        if goal_lonlat is None:
            return
        self._plan_and_send_to_nav(start_lonlat, goal_lonlat)

    def _on_campus_cancel(self, _msg: Empty):
        """外部请求中止当前道路级路线（NaviSkill 室外 pause/cancel 走这里）。

        规划器靠 ``_nav_batch_timer`` 持续把 navigate_through_poses 目标喂给 Nav2，
        只在 Nav2 侧取消目标会被下一拍重新喂上。这里把 ``_nav_running`` 落下并撤销
        已下发的 Nav2 目标，才能真正让底盘停住。``_finish_route`` 已完成撤销 +
        清运行标志；无活跃路线时收到取消是良性 no-op。
        """
        if not (self._nav_running or self._final_phase):
            self.get_logger().info("Campus cancel received but no route is active.")
            return
        self._finish_route("cancelled by external request", warn=True)

    # -- helpers -------------------------------------------------------------
    def _pose_xy_in_frame(self, msg: PoseStamped, target_frame: str):
        source_frame = msg.header.frame_id or target_frame
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if source_frame == target_frame:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot transform goal from {source_frame} to {target_frame}: {exc}"
            )
            return None
        yaw = _yaw_from_quat(tf.transform.rotation)
        cy, sy = math.cos(yaw), math.sin(yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        return tx + cy * x - sy * y, ty + sy * x + cy * y

    def _current_robot_lonlat(self):
        robot_frame = self.get_parameter("robot_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                robot_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot get current robot pose {self.frame_id}->{robot_frame}: {exc}"
            )
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        lon, lat = self.enu.to_lonlat(x, y)
        snapped = self._snap("current robot", x, y)
        return snapped if snapped is not None else (lon, lat)

    def _snap(self, label, x, y):
        """Project a map click onto the nearest road segment (any point, not
        just a vertex). Returns the clicked (lon, lat); projection happens at
        plan time so the temporary node lands exactly on the road."""
        lon, lat = self.enu.to_lonlat(x, y)
        proj = self.graph.project_to_edge(lon, lat)
        if proj is None:
            self.get_logger().error(f"{label}: graph has no road segments.")
            return None
        d = proj["dist"]
        self.get_logger().info(
            f"{label}: map({x:.1f},{y:.1f}) -> on road {d:.1f} m away "
            f"(segment {proj['u']}-{proj['v']}, t={proj['t']:.2f})"
        )
        if d > self.max_snap_m:
            self.get_logger().warn(
                f"{label} is {d:.1f} m from nearest road (> {self.max_snap_m} m)."
            )
        return (lon, lat)

    # -- patrol route picking ------------------------------------------------
    def _on_clicked_point(self, msg: PointStamped):
        """RViz 'Publish Point': append a waypoint to the patrol route."""
        x, y = msg.point.x, msg.point.y
        snapped = self._snap(f"waypoint #{len(self.patrol_pts) + 1}", x, y)
        if snapped is None:
            return
        self.patrol_pts.append(snapped)
        self.get_logger().info(
            f"Patrol waypoints: {len(self.patrol_pts)} "
            f"(1st is charging start). "
            f"`patrol_cmd undo` to remove last, `close` to finish & save."
        )
        self._publish_patrol_preview(close=False)

    def _on_set_params(self, params):
        """React to `ros2 param set ... patrol_cmd <cmd>`."""
        for p in params:
            if p.name != "patrol_cmd":
                continue
            self._patrol_dispatch(p.value or "")
        return SetParametersResult(successful=True)

    def _on_patrol_cmd_topic(self, msg):
        """React to `ros2 topic pub /patrol_cmd std_msgs/String "{data: close}"`.

        Topic entry point: works even when the `ros2 param` CLI plugin is
        missing (some minimal ros2cli installs lack it).
        """
        self._patrol_dispatch(msg.data or "")

    def _patrol_dispatch(self, raw_cmd):
        """Shared command handler for both the param and topic entry points."""
        cmd = (raw_cmd or "").strip().lower()
        if not cmd:
            return
        if cmd == "clear":
            self.patrol_pts = []
            self.get_logger().info("Patrol route cleared.")
            self._publish_patrol_preview(close=False)
        elif cmd == "undo":
            if self.patrol_pts:
                self.patrol_pts.pop()
            self.get_logger().info(
                f"Removed last waypoint; {len(self.patrol_pts)} left."
            )
            self._publish_patrol_preview(close=False)
        elif cmd in ("save", "close"):
            loop = cmd == "close" or self.get_parameter("patrol_loop").value
            self._patrol_save(close=loop)
        elif cmd == "load":
            self._patrol_load_json(self.get_parameter("patrol_load").value)
        else:
            self.get_logger().warn(
                f"Unknown patrol_cmd '{cmd}' (use clear/undo/save/close/load)."
            )

    def _patrol_build(self, close):
        """Connect consecutive waypoints via A* into one continuous route.

        Returns (lonlat_path, total_len_m, leg_lengths) or None.
        """
        pts = list(self.patrol_pts)
        if len(pts) < 2:
            self.get_logger().warn("Need >= 2 waypoints to build a patrol route.")
            return None
        if close:
            pts.append(pts[0])  # back to charging point

        full = []
        legs = []
        total = 0.0
        for i in range(len(pts) - 1):
            (slon, slat), (glon, glat) = pts[i], pts[i + 1]
            res = self.graph.plan_points(
                slon, slat, glon, glat, respect_turns=self.respect_turns
            )
            if res is None:
                self.get_logger().error(
                    f"Patrol leg {i + 1} ({i}->{i + 1}) not connected on road graph."
                )
                return None
            leg, length, _sp, _gp = res
            legs.append(length)
            total += length
            if full and leg and full[-1] == leg[0]:
                leg = leg[1:]  # drop duplicate joint vertex
            full.extend(leg)
        return full, total, legs

    def _densify(self, lonlat_path, step_m):
        """Insert points so consecutive samples are <= step_m apart (in meters)."""
        if step_m <= 0 or len(lonlat_path) < 2:
            return list(lonlat_path)
        out = [lonlat_path[0]]
        for (lon0, lat0), (lon1, lat1) in zip(lonlat_path, lonlat_path[1:]):
            e0, n0 = self.enu.to_xy(lon0, lat0)
            e1, n1 = self.enu.to_xy(lon1, lat1)
            seg = math.hypot(e1 - e0, n1 - n0)
            steps = int(seg // step_m)
            for k in range(1, steps + 1):
                f = k * step_m / seg
                if f >= 1.0:
                    break
                out.append((lon0 + (lon1 - lon0) * f, lat0 + (lat1 - lat0) * f))
            out.append((lon1, lat1))
        return out

    def _corner_points(self, lonlat_path, angle_deg, force_pts=None):
        """Keep only turning points: vertices where heading changes more than
        angle_deg. First/last and any point in force_pts (e.g. clicked
        waypoints) are always kept."""
        if len(lonlat_path) <= 2:
            return list(lonlat_path)
        thr = math.radians(angle_deg)
        force = set(force_pts or [])
        keep = [lonlat_path[0]]
        for i in range(1, len(lonlat_path) - 1):
            if lonlat_path[i] in force:
                keep.append(lonlat_path[i])
                continue
            ax, ay = self.enu.to_xy(*lonlat_path[i - 1])
            bx, by = self.enu.to_xy(*lonlat_path[i])
            cx, cy = self.enu.to_xy(*lonlat_path[i + 1])
            a1 = math.atan2(by - ay, bx - ax)
            a2 = math.atan2(cy - by, cx - bx)
            d = abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi)
            if d > thr:
                keep.append(lonlat_path[i])
        keep.append(lonlat_path[-1])
        return keep

    def _publish_patrol_preview(self, close):
        """Publish the current patrol route + numbered waypoint markers."""
        stamp = _latest_tf_stamp()
        if HAVE_MARKERS and hasattr(self, "patrol_wp_pub"):
            enu_pts = []
            for i, (lon, lat) in enumerate(self.patrol_pts):
                e, n = self.enu.to_xy(lon, lat)
                label = "CHG" if i == 0 else str(i)
                enu_pts.append((e, n, label))
            self.patrol_wp_pub.publish(
                MarkerArray(
                    markers=_patrol_waypoint_markers(enu_pts, self.frame_id, stamp)
                )
            )
        if len(self.patrol_pts) < 2:
            self._publish_corner_markers([])
            return
        built = self._patrol_build(close=close)
        if built is None:
            return
        full, total, _legs = built
        path_z = float(self.get_parameter("network_z").value)
        self._publish_path_msg(
            self.patrol_path_pub, full, total, label="patrol", z=path_z
        )
        angle = self.get_parameter("patrol_corner_angle_deg").value
        corners = self._corner_points(full, angle, force_pts=self.patrol_pts)
        self._publish_corner_markers(corners)

    def _publish_corner_markers(self, corners_lonlat):
        if not (HAVE_MARKERS and hasattr(self, "patrol_corner_pub")):
            return
        stamp = _latest_tf_stamp()
        enu_pts = [self.enu.to_xy(lon, lat) for lon, lat in corners_lonlat]
        self.patrol_corner_pub.publish(
            MarkerArray(
                markers=_patrol_corner_markers(enu_pts, self.frame_id, stamp)
            )
        )

    def _publish_path_msg(self, pub, lonlat_path, length, label="path", z=0.0):
        msg = self._path_msg_from_lonlat(lonlat_path, z=z)
        pub.publish(msg)
        self.get_logger().info(
            f"Published {label}: {len(msg.poses)} poses, {length:.1f} m."
        )
        return msg

    def _path_msg_from_lonlat(self, lonlat_path, z=0.0):
        msg = Path()
        msg.header.frame_id = self.frame_id
        # Use time 0 so RViz/Nav2 transform with the latest available TF instead
        # of failing when TF is behind sim time.
        msg.header.stamp = _latest_tf_stamp()
        xy = [self.enu.to_xy(lon, lat) for lon, lat in lonlat_path]
        for i, (e, n) in enumerate(xy):
            if len(xy) <= 1:
                yaw = 0.0
            elif i < len(xy) - 1:
                ne, nn = xy[i + 1]
                yaw = math.atan2(nn - n, ne - e)
            else:
                pe, pn = xy[i - 1]
                yaw = math.atan2(n - pn, e - pe)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = e
            ps.pose.position.y = n
            # Visual paths may sit below the robot (match network_z); Nav2 goals
            # must stay at z=0 so /plan does not droop to the marker height.
            ps.pose.position.z = float(z)
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        return msg

    def _publish_nav_waypoint_markers(self, path_msg):
        if not (HAVE_MARKERS and hasattr(self, "nav_wp_marker_pub")):
            return
        enu_pts = [
            (p.pose.position.x, p.pose.position.y, str(i + 1))
            for i, p in enumerate(path_msg.poses)
        ]
        size = float(self.get_parameter("nav_waypoint_marker_size_m").value)
        markers = _nav_waypoint_markers(
            enu_pts, self.frame_id, path_msg.header.stamp, size=size,
            reached=0, stale_upto=self._nav_marker_n,
        )
        self._nav_marker_n = len(enu_pts)
        self._nav_marker_reached = 0
        self.nav_wp_marker_pub.publish(MarkerArray(markers=markers))

    def _recolor_nav_waypoints(self, reached):
        """Turn the waypoints the robot has passed green.

        The whole route is republished rather than just the waypoints that
        changed: the topic is latched with depth 1, so the last message has to
        stand on its own for an RViz that connects mid-route.
        """
        if not (HAVE_MARKERS and hasattr(self, "nav_wp_marker_pub")):
            return
        n = min(self._nav_marker_n, len(self._nav_route_poses))
        reached = max(0, min(reached, n))
        if n == 0 or reached == self._nav_marker_reached:
            return
        enu_pts = [
            (p.pose.position.x, p.pose.position.y, str(i + 1))
            for i, p in enumerate(self._nav_route_poses[:n])
        ]
        size = float(self.get_parameter("nav_waypoint_marker_size_m").value)
        markers = _nav_waypoint_markers(
            enu_pts, self.frame_id, _latest_tf_stamp(), size=size,
            reached=reached, clear_first=False,
        )
        self._nav_marker_reached = reached
        self.nav_wp_marker_pub.publish(MarkerArray(markers=markers))

    def _patrol_save(self, close):
        built = self._patrol_build(close=close)
        if built is None:
            return
        full, total, legs = built
        step = self.get_parameter("patrol_sample_m").value
        angle = self.get_parameter("patrol_corner_angle_deg").value
        dense = self._densify(full, step)
        corners = self._corner_points(full, angle, force_pts=self.patrol_pts)

        out_dir = self.get_parameter("patrol_out_dir").value
        name = self.get_parameter("patrol_name").value
        os.makedirs(out_dir, exist_ok=True)

        def _to_rec(lon, lat):
            e, n = self.enu.to_xy(lon, lat)
            return {"lon": lon, "lat": lat, "map_x": e, "map_y": n}

        # ordered waypoints (the points the operator clicked)
        waypoints = []
        for i, (lon, lat) in enumerate(self.patrol_pts):
            rec = _to_rec(lon, lat)
            rec.update(index=i, role="charging" if i == 0 else "via")
            waypoints.append(rec)
        # corner key points: turning points of the route
        corner_pts = [_to_rec(lon, lat) for lon, lat in corners]
        # control points: evenly spaced every `step` meters
        path_pts = [_to_rec(lon, lat) for lon, lat in dense]

        bundle = {
            "name": name,
            "frame_id": self.frame_id,
            "datum": {
                "lat": self.get_parameter("datum_lat").value,
                "lon": self.get_parameter("datum_lon").value,
            },
            "projection": self.enu.fingerprint(),
            "loop": bool(close),
            "length_m": total,
            "leg_lengths_m": legs,
            "control_step_m": step,
            "corner_angle_deg": angle,
            "waypoints": waypoints,    # operator-clicked points
            "corners": corner_pts,     # turning key points
            "path": path_pts,          # control points every control_step_m
        }
        json_path = os.path.join(out_dir, f"{name}.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(bundle, f, ensure_ascii=False, indent=2)

        # GeoJSON LineString for QGIS verification
        geojson = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {"name": name, "length_m": total, "loop": bool(close)},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[lon, lat] for lon, lat in dense],
                },
            }],
        }
        geojson_path = os.path.join(out_dir, f"{name}.geojson")
        with open(geojson_path, "w", encoding="utf-8") as f:
            json.dump(geojson, f, ensure_ascii=False, indent=2)

        def _write_csv(fname, recs):
            p = os.path.join(out_dir, fname)
            with open(p, "w", encoding="utf-8") as f:
                f.write("map_x,map_y,lon,lat\n")
                for r in recs:
                    f.write(
                        f"{r['map_x']:.3f},{r['map_y']:.3f},"
                        f"{r['lon']:.8f},{r['lat']:.8f}\n"
                    )
            return p

        # control points (every step m) and corner key points -> separate CSVs
        ctrl_csv = _write_csv(f"{name}_control.csv", path_pts)
        corner_csv = _write_csv(f"{name}_corners.csv", corner_pts)

        self._publish_patrol_preview(close=close)
        self.get_logger().info(
            f"Saved patrol '{name}': {len(self.patrol_pts)} waypoints, "
            f"{len(corner_pts)} corners, {len(path_pts)} control points "
            f"(every {step:.0f} m), {total:.1f} m, "
            f"({'loop' if close else 'open'}).\n"
            f"  -> {json_path}\n  -> {geojson_path}\n"
            f"  -> {ctrl_csv}\n  -> {corner_csv}"
        )

    def _patrol_load_json(self, path):
        """Load a saved patrol json and visualize its path + corners + waypoints
        in RViz, using the file's stored map coordinates."""
        if not path:
            self.get_logger().warn(
                "patrol_cmd load: set `patrol_load` to a saved <name>.json first."
            )
            return
        try:
            with open(path, encoding="utf-8") as f:
                d = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error(f"patrol load failed: {e}")
            return

        # Stored map_x/map_y are only valid under the datum + projection they
        # were written with. Every record also carries lon/lat, which is datum
        # independent, so a mismatch is recoverable: reproject from lon/lat
        # instead of trusting the stale metric coordinates. That makes changing
        # the datum (or the projection, as the switch from the linearized
        # tangent plane to UTM did) a non-event for saved patrols.
        d_lat = d.get("datum", {}).get("lat")
        d_lon = d.get("datum", {}).get("lon")
        proj = d.get("projection") or {}
        reasons = []
        if d_lat is None or d_lon is None:
            reasons.append("no datum recorded")
        elif (abs(d_lat - self.enu.lat0) > 1e-9
                or abs(d_lon - self.enu.lon0) > 1e-9):
            reasons.append(
                f"datum ({d_lat},{d_lon}) != node datum "
                f"({self.enu.lat0},{self.enu.lon0})")
        if proj != self.enu.fingerprint():
            # Pre-UTM files carry {r_east, r_north}; older ones nothing at all.
            reasons.append(
                f"projection {proj or 'unrecorded'} != {self.enu.fingerprint()}")
        stale = bool(reasons)
        if stale:
            self.get_logger().warn(
                f"Loaded patrol was written under a different frame "
                f"({'; '.join(reasons)}); reprojecting from the stored lon/lat. "
                "Run `patrol_cmd save` to refresh the file itself."
            )

        def _xy(rec):
            """Map coords for one stored record, reprojected when stale."""
            if stale and rec.get("lon") is not None and rec.get("lat") is not None:
                return self.enu.to_xy(rec["lon"], rec["lat"])
            return float(rec["map_x"]), float(rec["map_y"])

        stamp = _latest_tf_stamp()
        # restore clicked waypoints so undo/save keep working after a load
        self.patrol_pts = [(w["lon"], w["lat"]) for w in d.get("waypoints", [])]

        # path (control points) -> /patrol_path
        path_recs = d.get("path", [])
        if path_recs:
            path_z = float(self.get_parameter("network_z").value)
            msg = Path()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            for r in path_recs:
                x, y = _xy(r)
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = path_z
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            self.patrol_path_pub.publish(msg)

        if HAVE_MARKERS:
            wp_enu = []
            for i, w in enumerate(d.get("waypoints", [])):
                label = "CHG" if i == 0 else str(i)
                x, y = _xy(w)
                wp_enu.append((x, y, label))
            if hasattr(self, "patrol_wp_pub"):
                self.patrol_wp_pub.publish(MarkerArray(
                    markers=_patrol_waypoint_markers(wp_enu, self.frame_id, stamp)
                ))
            corner_enu = [_xy(c) for c in d.get("corners", [])]
            if hasattr(self, "patrol_corner_pub"):
                self.patrol_corner_pub.publish(MarkerArray(
                    markers=_patrol_corner_markers(corner_enu, self.frame_id, stamp)
                ))

        self.get_logger().info(
            f"Loaded patrol '{d.get('name')}': {len(d.get('waypoints', []))} "
            f"waypoints, {len(d.get('corners', []))} corners, "
            f"{len(path_recs)} control points -> /patrol_path, "
            f"/patrol_waypoints, /patrol_corners."
        )

    # -- semantic lateral route adapter ------------------------------------
    def _init_semantic_lateral_adapter(self):
        """Subscribe to exact-stamp semantic frames and start shadow output."""
        self._lateral_route_generation = 0
        self._nav_route_s = []
        self._nav_route_motor = []
        self._lateral_estimator = RoadBoundaryEstimator(
            near=self.get_parameter("semantic_lateral_near_m").value,
            far=self.get_parameter("semantic_lateral_far_m").value,
            bin_size=self.get_parameter("semantic_lateral_bin_size_m").value,
            lateral_limit=self.get_parameter("semantic_lateral_limit_m").value,
            min_points=self.get_parameter(
                "semantic_lateral_min_points_per_bin"
            ).value,
            width_min=self.get_parameter("semantic_lateral_width_min_m").value,
            width_max=self.get_parameter("semantic_lateral_width_max_m").value,
            lookahead=self.get_parameter("semantic_lateral_lookahead_m").value,
            inside_margin=self.get_parameter(
                "semantic_lateral_inside_margin_m"
            ).value,
            temporal_alpha=self.get_parameter(
                "semantic_lateral_filter_alpha"
            ).value,
            fit_max_slope=self.get_parameter(
                "semantic_lateral_fit_max_slope"
            ).value,
            center_innovation=self.get_parameter(
                "semantic_lateral_center_innovation_m"
            ).value,
            width_innovation=self.get_parameter(
                "semantic_lateral_width_innovation_m"
            ).value,
            junction_guard_before=self.get_parameter(
                "semantic_lateral_junction_guard_before_m"
            ).value,
            junction_guard_after=self.get_parameter(
                "semantic_lateral_junction_guard_after_m"
            ).value,
            junction_release_frames=self.get_parameter(
                "semantic_lateral_junction_release_frames"
            ).value,
        )
        self._lateral_target_pub = self.create_publisher(
            Path,
            self.get_parameter("semantic_lateral_target_topic").value,
            rviz_qos(),
        )
        self._lateral_status_pub = self.create_publisher(
            String,
            self.get_parameter("semantic_lateral_status_topic").value,
            rviz_qos(),
        )
        if HAVE_MARKERS:
            self._lateral_debug_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter("semantic_lateral_debug_topic").value,
                rviz_qos(),
            )
        if not self.get_parameter("semantic_lateral_enabled").value:
            self.get_logger().info("Semantic lateral adapter disabled.")
            return
        self._lateral_worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="campus_lateral"
        )
        self.create_subscription(
            LabelInfo,
            self.get_parameter("semantic_lateral_labels_topic").value,
            self._on_lateral_labels,
            latched_qos(),
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter(
                "semantic_lateral_safety_costmap_topic"
            ).value,
            self._on_lateral_costmap,
            qos_profile_sensor_data,
        )
        for msg_type, parameter, kind in (
            (Image, "semantic_lateral_segmentation_topic", "segmentation"),
            (Image, "semantic_lateral_confidence_topic", "confidence"),
            (PointCloud2, "semantic_lateral_cloud_topic", "cloud"),
        ):
            self.create_subscription(
                msg_type,
                self.get_parameter(parameter).value,
                lambda msg, k=kind: self._cache_lateral_frame(k, msg),
                qos_profile_sensor_data,
            )
        self.create_timer(0.1, self._lateral_tick)
        mode = (
            "SHADOW (debug only)"
            if self.get_parameter("semantic_lateral_shadow_mode").value
            else "CLOSED LOOP"
        )
        self.get_logger().info(
            f"Semantic lateral adapter enabled in {mode}: route direction "
            "defines right; sensor acquisition-time TF transforms road points."
        )

    @staticmethod
    def _stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec
        )

    def _on_lateral_labels(self, msg):
        wanted = self.get_parameter("semantic_lateral_road_class").value
        road_id = None
        for semantic_class in msg.class_map:
            if semantic_class.class_name == wanted:
                road_id = int(semantic_class.class_id)
                break
        self._lateral_road_class_id = road_id
        if road_id is None:
            self.get_logger().error(
                f"Semantic lateral adapter cannot find class '{wanted}' in LabelInfo."
            )
        else:
            self.get_logger().info(
                f"Semantic lateral road class '{wanted}' has id {road_id}."
            )

    def _on_lateral_costmap(self, msg):
        self._lateral_costmap = msg

    def _cache_lateral_frame(self, kind, msg):
        stamp_ns = self._stamp_ns(msg.header)
        slot = self._lateral_pending.setdefault(stamp_ns, {})
        slot[kind] = msg
        # Exact synchronization is intentional: the organized cloud and mask
        # share a pixel index only when they came from the same acquisition.
        while len(self._lateral_pending) > 8:
            del self._lateral_pending[min(self._lateral_pending)]
        self._try_start_lateral_job()

    def _try_start_lateral_job(self):
        if self._lateral_worker is None or self._lateral_route is None:
            return
        if self._route_align_required and not self._route_align_complete:
            return
        if self._lateral_future is not None:
            return
        road_id = getattr(self, "_lateral_road_class_id", None)
        if road_id is None:
            return
        ready = [
            stamp for stamp, slot in self._lateral_pending.items()
            if {"segmentation", "confidence", "cloud"} <= slot.keys()
        ]
        if not ready:
            return
        stamp_ns = max(ready)
        slot = self._lateral_pending.pop(stamp_ns)
        for old_stamp in [s for s in self._lateral_pending if s <= stamp_ns]:
            del self._lateral_pending[old_stamp]

        now_ns = self.get_clock().now().nanoseconds
        max_age = float(
            self.get_parameter("semantic_lateral_max_frame_age_s").value
        )
        if now_ns - stamp_ns > max_age * 1e9:
            self._lateral_state = (
                "HOLD_LAST" if self._lateral_target is not None
                else "ACQUIRE_BOUNDARY"
            )
            self._lateral_reason = "stale_semantic_frame"
            return
        cloud = slot["cloud"]
        try:
            sensor_tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.08),
            )
            robot_tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.get_parameter("robot_frame").value,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.08),
            )
        except TransformException as exc:
            self._lateral_state = (
                "HOLD_LAST" if self._lateral_target is not None
                else "ACQUIRE_BOUNDARY"
            )
            self._lateral_reason = "acquisition_tf_unavailable"
            self.get_logger().warn(
                f"Semantic lateral acquisition-time TF unavailable: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        route = self._lateral_route
        robot_projection = route.project(
            robot_tf.transform.translation.x,
            robot_tf.transform.translation.y,
            s_hint=self._lateral_s,
        )
        self._lateral_s = max(self._lateral_s, robot_projection.s)
        self._lateral_robot_d = robot_projection.d
        if not route.sample(self._lateral_s).motor_road:
            self._lateral_target = None
            self._lateral_target_stamp_ns = None
            self._lateral_safety_ok = False
            self._lateral_safety_reason = "non_motor_segment"
            self._lateral_state = "REFERENCE_ONLY"
            self._lateral_reason = "non_motor_segment"
            return
        rotation, translation = self._transform_arrays(sensor_tf.transform)
        self._lateral_future = self._lateral_worker.submit(
            self._process_lateral_frame,
            slot["segmentation"],
            slot["confidence"],
            cloud,
            rotation,
            translation,
            route,
            self._lateral_s,
            road_id,
            self._lateral_route_generation,
            stamp_ns,
            max(1, int(self.get_parameter(
                "semantic_lateral_pixel_stride"
            ).value)),
            int(self.get_parameter(
                "semantic_lateral_min_pixel_confidence"
            ).value),
            self._lateral_robot_d,
            int(self.get_parameter(
                "semantic_lateral_image_edge_margin_px"
            ).value),
        )

    @staticmethod
    def _transform_arrays(transform):
        q = transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        return rotation, translation

    def _process_lateral_frame(
        self, segmentation, confidence, cloud, rotation, translation,
        route, s_robot, road_id, generation, stamp_ns, stride,
        min_pixel_confidence, robot_d=0.0, image_edge_margin_px=8,
    ):
        """Heavy pixel/cloud projection; runs on the single worker thread."""
        if segmentation.encoding not in ("mono8", "8UC1"):
            raise ValueError(
                f"unsupported segmentation encoding {segmentation.encoding}"
            )
        if confidence.encoding not in ("mono8", "8UC1"):
            raise ValueError(
                f"unsupported confidence encoding {confidence.encoding}"
            )
        shape = (int(segmentation.height), int(segmentation.width))
        if shape != (int(confidence.height), int(confidence.width)):
            raise ValueError("segmentation/confidence dimensions differ")
        if shape != (int(cloud.height), int(cloud.width)):
            raise ValueError("semantic images/cloud dimensions differ")
        segmentation_array = np.ndarray(
            shape, dtype=np.uint8, buffer=segmentation.data,
            strides=(int(segmentation.step), 1),
        )
        confidence_array = np.ndarray(
            shape, dtype=np.uint8, buffer=confidence.data,
            strides=(int(confidence.step), 1),
        )
        field_map = {field.name: field for field in cloud.fields}
        for name in ("x", "y", "z"):
            if name not in field_map:
                raise ValueError(f"organized cloud has no '{name}' field")
            if field_map[name].datatype != PointField.FLOAT32:
                raise ValueError(f"organized cloud '{name}' is not float32")
        dtype = np.dtype(">f4" if cloud.is_bigendian else "<f4")
        coordinates = []
        for name in ("x", "y", "z"):
            coordinates.append(np.ndarray(
                shape,
                dtype=dtype,
                buffer=cloud.data,
                offset=int(field_map[name].offset),
                strides=(int(cloud.row_step), int(cloud.point_step)),
            ))
        sample = np.s_[::stride, ::stride]
        mask = (
            (segmentation_array[sample] == road_id)
            & (confidence_array[sample] >= min_pixel_confidence)
        )
        sampled_columns = np.arange(0, shape[1], stride, dtype=np.int32)
        mask_columns = np.nonzero(mask)[1]
        pixel_columns = sampled_columns[mask_columns]
        edge_margin = max(0, int(image_edge_margin_px))
        image_edge = (
            (pixel_columns < edge_margin)
            | (pixel_columns >= shape[1] - edge_margin)
        )
        camera_points = np.column_stack([
            coordinate[sample][mask] for coordinate in coordinates
        ]).astype(np.float64, copy=False)
        finite = np.isfinite(camera_points).all(axis=1)
        camera_points = camera_points[finite]
        image_edge = image_edge[finite]
        map_points = camera_points @ rotation.T + translation
        road_xy = map_points[:, :2]
        target, reason = self._lateral_estimator.estimate(
            route, road_xy, s_robot, image_edge=image_edge, robot_d=robot_d
        )
        return generation, target, reason, road_xy, stamp_ns

    def _lateral_tick(self):
        if self._lateral_future is not None and self._lateral_future.done():
            future = self._lateral_future
            self._lateral_future = None
            try:
                generation, target, reason, road_xy, stamp_ns = future.result()
            except Exception as exc:  # noqa: BLE001 -- bad sensor frame is recoverable
                self._lateral_state = "DEGRADED_STOP"
                self._lateral_reason = f"frame_error:{exc}"
                self.get_logger().error(
                    f"Semantic lateral frame rejected: {exc}",
                    throttle_duration_sec=5.0,
                )
            else:
                if generation == self._lateral_route_generation:
                    self._consume_lateral_result(
                        target, reason, road_xy, stamp_ns
                    )
                else:
                    # A frame from the previous route may have finished after
                    # _configure_lateral_route() reset the state. Clear its
                    # fitted model before admitting a frame for the new route.
                    self._lateral_estimator.reset()
        self._try_start_lateral_job()

        if self._lateral_target is None:
            now_ns = self.get_clock().now().nanoseconds
            fallback = self._junction_occlusion_target(
                "junction_semantic_target_missing"
            )
            if fallback is not None:
                self._consume_lateral_result(
                    fallback,
                    fallback.state_reason,
                    self._lateral_last_debug_points,
                    now_ns,
                )
        if self._lateral_target_stamp_ns is not None:
            age = (
                self.get_clock().now().nanoseconds
                - self._lateral_target_stamp_ns
            ) / 1e9
            hold = float(self.get_parameter("semantic_lateral_hold_last_s").value)
            if age > hold:
                now_ns = self.get_clock().now().nanoseconds
                fallback = self._junction_occlusion_target(
                    "junction_occlusion_timeout"
                )
                if fallback is not None:
                    self._consume_lateral_result(
                        fallback,
                        fallback.state_reason,
                        self._lateral_last_debug_points,
                        now_ns,
                    )
                else:
                    self._lateral_target = None
                    self._lateral_safety_ok = False
                    self._lateral_safety_reason = "target_expired"
                    self._lateral_state = "DEGRADED_STOP"
                    self._lateral_reason = "target_expired"
        self._publish_lateral_status()

    def _consume_lateral_result(self, target, reason, road_xy, stamp_ns):
        self._lateral_last_debug_points = road_xy
        if target is None:
            fallback = self._junction_occlusion_target(reason)
            if fallback is not None:
                target = fallback
                reason = fallback.state_reason
        if (
            target is not None
            and target.width > 0.0
            and target.state_reason not in (
                "junction_occlusion_hold",
                "junction_osm_current_offset",
                "junction_occlusion_timeout",
            )
        ):
            self._lateral_last_corridor = {
                "left": float(target.left),
                "right": float(target.right),
                "center": float(target.center),
                "width": float(target.width),
                "confidence": float(target.confidence),
            }
        minimum = float(
            self.get_parameter("semantic_lateral_min_target_confidence").value
        )
        if target is not None and target.confidence >= minimum:
            target = self._limit_lateral_target_from_robot(target)
            # The slew limiter changes the actual commanded point, so safety
            # must be evaluated after it.  Doing this in the opposite order can
            # find a safe point near the center and then move it back into a
            # lethal cell while retaining the misleading inward-shift reason.
            target, receded_for_window = self._fit_target_in_safety_window(
                target
            )
            if receded_for_window:
                self.get_logger().warning(
                    "Dynamic target exceeded the local costmap; shortened it "
                    f"to {target.s - self._lateral_s:.2f} m lookahead at "
                    f"d={target.d:.2f} m.",
                    throttle_duration_sec=2.0,
                )
            target, avoiding_obstacle = self._avoid_obstacle_in_corridor(
                target
            )
            if avoiding_obstacle:
                self.get_logger().warning(
                    "Preferred road target is occupied; selected a safe "
                    f"corridor target at forward="
                    f"{target.s - self._lateral_s:.2f} m, d={target.d:.2f} m.",
                    throttle_duration_sec=2.0,
                )
            geometry_target, extended_for_goal, geometry_reason = (
                self._ensure_target_beyond_goal_tolerance(target)
            )
            geometry_ok = geometry_target is not None
            if geometry_ok:
                target = geometry_target
                if extended_for_goal:
                    self.get_logger().warning(
                        "Dynamic target fell inside Nav2 goal tolerance; "
                        f"extended it along the selected OSM branch to "
                        f"{target.s - self._lateral_s:.2f} m lookahead "
                        f"({self._target_robot_distance(target):.2f} m away).",
                        throttle_duration_sec=2.0,
                    )
            lateral_change = (
                float("inf") if self._lateral_seq_d is None
                else abs(target.d - self._lateral_seq_d)
            )
            route_progress = (
                float("inf") if self._lateral_seq_s is None
                else target.s - self._lateral_seq_s
            )
            if (
                lateral_change >= float(self.get_parameter(
                    "semantic_lateral_target_move_m"
                ).value)
                or route_progress >= float(self.get_parameter(
                    "semantic_lateral_target_progress_m"
                ).value)
            ):
                self._lateral_target_seq += 1
                self._lateral_seq_d = target.d
                self._lateral_seq_s = target.s
            self._lateral_target = target
            self._lateral_target_stamp_ns = stamp_ns
            self._lateral_goal_geometry_ok = geometry_ok
            if geometry_ok:
                (
                    self._lateral_safety_ok,
                    self._lateral_safety_reason,
                ) = self._lateral_target_is_safe(target.x, target.y)
            else:
                self._lateral_safety_ok = False
                self._lateral_safety_reason = geometry_reason
            self._lateral_good_frames += 1
            required = max(1, int(self.get_parameter(
                "semantic_lateral_acquire_frames"
            ).value))
            if not geometry_ok:
                self._lateral_state = "DEGRADED_STOP"
                self._lateral_reason = geometry_reason
            elif avoiding_obstacle:
                self._lateral_state = "AVOID_OBSTACLE"
                self._lateral_reason = target.state_reason
            elif target.junction_guard:
                self._lateral_state = "JUNCTION_GUARD"
                self._lateral_reason = target.state_reason
            elif (
                target.state_reason == "seek_right_edge"
                and self._lateral_good_frames >= required
            ):
                self._lateral_state = "SEEK_RIGHT_EDGE"
                self._lateral_reason = target.state_reason
            elif self._lateral_good_frames >= required:
                self._lateral_state = "TRACK_RIGHT"
                self._lateral_reason = target.state_reason
            else:
                self._lateral_state = "ACQUIRE_BOUNDARY"
                self._lateral_reason = (
                    f"acquiring:{self._lateral_good_frames}/{required}"
                )
        else:
            self._lateral_reason = reason if target is None else "low_confidence"
            self._lateral_state = (
                "HOLD_LAST" if self._lateral_target is not None
                else "ACQUIRE_BOUNDARY"
            )
        debug_target = (
            self._lateral_target
            if self._lateral_state == "HOLD_LAST"
            else target
        )
        self._publish_lateral_debug(debug_target, road_xy, stamp_ns)

    def _junction_occlusion_target(self, cause):
        """Follow the selected OSM branch when a vehicle hides the road mask."""
        route = self._lateral_route
        if route is None:
            return None
        before = float(self.get_parameter(
            "semantic_lateral_junction_guard_before_m"
        ).value)
        after = float(self.get_parameter(
            "semantic_lateral_junction_guard_after_m"
        ).value)
        if route.junction_in_guard(self._lateral_s, before, after) is None:
            return None
        lookahead = float(self.get_parameter(
            "semantic_lateral_lookahead_m"
        ).value)
        target_s = min(route.total_length, self._lateral_s + lookahead)
        sample = route.sample(target_s)
        margin = float(self.get_parameter(
            "semantic_lateral_inside_margin_m"
        ).value)
        corridor = self._lateral_last_corridor
        if corridor is None:
            # No visual model was acquired before entering the intersection.
            # Preserve only the achieved offset; do not invent a lane width.
            left = self._lateral_robot_d - margin
            right = self._lateral_robot_d + margin
            center = self._lateral_robot_d
            width = 2.0 * margin
            confidence = float(self.get_parameter(
                "semantic_lateral_min_target_confidence"
            ).value)
            target_d = self._lateral_robot_d
            state_reason = "junction_osm_current_offset"
        else:
            left = corridor["left"]
            right = corridor["right"]
            center = corridor["center"]
            width = corridor["width"]
            confidence = max(
                corridor["confidence"],
                float(self.get_parameter(
                    "semantic_lateral_min_target_confidence"
                ).value),
            )
            preferred_d = (
                self._lateral_avoid_d
                if self._lateral_avoid_active
                and self._lateral_avoid_d is not None
                else self._lateral_robot_d
            )
            target_d = float(np.clip(
                preferred_d, left + margin, right - margin
            ))
            state_reason = (
                "junction_occlusion_timeout"
                if cause == "junction_occlusion_timeout"
                else "junction_occlusion_hold"
            )
        return LateralTarget(
            x=float(sample.x + target_d * math.sin(sample.yaw)),
            y=float(sample.y - target_d * math.cos(sample.yaw)),
            yaw=float(sample.yaw),
            s=float(target_s),
            d=float(target_d),
            left=float(left),
            right=float(right),
            confidence=float(confidence),
            bins=(),
            center=float(center),
            width=float(width),
            junction_guard=True,
            state_reason=state_reason,
        )

    def _publish_lateral_status(self):
        target = self._lateral_target
        payload = {
            "state": self._lateral_state,
            "reason": self._lateral_reason,
            "shadow_mode": bool(
                self.get_parameter("semantic_lateral_shadow_mode").value
            ),
            "route_s_m": round(self._lateral_s, 3),
            "road_class_id": getattr(self, "_lateral_road_class_id", None),
            "safety_ok": self._lateral_safety_ok,
            "safety_reason": self._lateral_safety_reason,
            "route_align_required": self._route_align_required,
            "route_align_complete": self._route_align_complete,
            "route_heading_error_deg": (
                None if self._route_align_error_rad is None
                else round(math.degrees(self._route_align_error_rad), 2)
            ),
        }
        if target is not None:
            age = (
                self.get_clock().now().nanoseconds
                - self._lateral_target_stamp_ns
            ) / 1e9
            payload.update({
                "target_x": round(target.x, 3),
                "target_y": round(target.y, 3),
                "target_d_m": round(target.d, 3),
                "robot_d_m": round(self._lateral_robot_d, 3),
                "target_forward_m": round(target.s - self._lateral_s, 3),
                "target_distance_m": round(
                    self._target_robot_distance(target), 3
                ),
                "left_d_m": round(target.left, 3),
                "right_d_m": round(target.right, 3),
                "road_center_d_m": round(target.center, 3),
                "road_width_m": round(target.width, 3),
                "junction_guard": target.junction_guard,
                "obstacle_avoidance": self._lateral_avoid_active,
                "obstacle_avoid_d_m": (
                    None if self._lateral_avoid_d is None
                    else round(self._lateral_avoid_d, 3)
                ),
                "confidence": round(target.confidence, 3),
                "observation_age_s": round(max(0.0, age), 3),
            })
        self._log_lateral_status_transition(payload)
        self._lateral_status_pub.publish(String(data=json.dumps(payload)))

    def _log_lateral_status_transition(self, payload):
        """Print actionable lateral decisions only when their state changes."""
        junction_guard = bool(payload.get("junction_guard", False))
        if junction_guard != self._lateral_last_junction_guard:
            event = "ENTER" if junction_guard else "EXIT"
            self.get_logger().info(
                f"[campus_lateral] junction guard {event}: "
                f"route_s={payload['route_s_m']:.2f} m, "
                f"road_width={payload.get('road_width_m', float('nan')):.2f} m; "
                + (
                    "holding the achieved lateral offset and frozen road width."
                    if junction_guard
                    else "releasing the frozen corridor after stable observations."
                )
            )
        self._lateral_last_junction_guard = junction_guard

        signature = (
            payload.get("state"),
            payload.get("reason"),
            payload.get("safety_ok"),
            payload.get("safety_reason"),
            junction_guard,
            payload.get("route_align_required"),
            payload.get("route_align_complete"),
            payload.get("shadow_mode"),
        )
        if signature == self._lateral_last_log_signature:
            return
        self._lateral_last_log_signature = signature

        safety = (
            "OK" if payload.get("safety_ok")
            else f"BLOCKED({payload.get('safety_reason', 'unknown')})"
        )
        target_text = "target=none"
        if "target_x" in payload:
            target_text = (
                f"target=({payload['target_x']:.2f},{payload['target_y']:.2f}), "
                f"forward={payload['target_forward_m']:.2f} m, "
                f"robot_d={payload['robot_d_m']:.2f} m, "
                f"target_d={payload['target_d_m']:.2f} m, "
                f"bounds=[{payload['left_d_m']:.2f},{payload['right_d_m']:.2f}] m, "
                f"width={payload['road_width_m']:.2f} m, "
                f"confidence={payload['confidence']:.2f}, "
                f"age={payload['observation_age_s']:.2f} s"
            )
        message = (
            f"[campus_lateral] state={payload['state']} "
            f"reason={payload['reason']}, route_s={payload['route_s_m']:.2f} m, "
            f"junction={junction_guard}, safety={safety}, {target_text}"
        )
        operational_states = {
            "TRACK_RIGHT", "SEEK_RIGHT_EDGE", "JUNCTION_GUARD",
            "AVOID_OBSTACLE", "HOLD_LAST", "DEGRADED_STOP",
        }
        if (
            payload.get("state") == "DEGRADED_STOP"
            or (
                not payload.get("safety_ok")
                and payload.get("state") in operational_states
                and not payload.get("shadow_mode")
            )
        ):
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _publish_lateral_debug(self, target, road_xy, stamp_ns):
        if target is not None:
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            # This topic represents the point we can actually command, not the
            # ideal right-edge trace (already shown by the boundary markers).
            # Joining raw boundary bins directly to a slew-limited target made
            # the visualization appear to notch into the road center even
            # though Nav2 receives only the final target pose.
            span = max(0.0, target.s - self._lateral_s)
            count = max(2, int(math.ceil(span / 0.5)) + 1)
            for ratio in np.linspace(0.0, 1.0, count):
                sample_s = self._lateral_s + ratio * span
                smooth = ratio * ratio * (3.0 - 2.0 * ratio)
                sample_d = (
                    self._lateral_robot_d
                    + smooth * (target.d - self._lateral_robot_d)
                )
                route_sample = self._lateral_route.sample(sample_s)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(
                    route_sample.x + sample_d * math.sin(route_sample.yaw)
                )
                pose.pose.position.y = float(
                    route_sample.y - sample_d * math.cos(route_sample.yaw)
                )
                qx, qy, qz, qw = _quat_from_yaw(route_sample.yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path.poses.append(pose)
            self._lateral_target_pub.publish(path)
        if not HAVE_MARKERS or not hasattr(self, "_lateral_debug_pub"):
            return
        stamp = Time(nanoseconds=stamp_ns).to_msg()
        markers = []
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.append(clear)

        points_marker = Marker()
        points_marker.header.frame_id = self.frame_id
        points_marker.header.stamp = stamp
        points_marker.ns = "semantic_road_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = 0.08
        points_marker.scale.y = 0.08
        points_marker.color = ColorRGBA(r=0.1, g=0.55, b=1.0, a=0.45)
        if len(road_xy):
            step = max(1, len(road_xy) // 2500)
            points_marker.points = [
                Point(x=float(x), y=float(y), z=0.04)
                for x, y in road_xy[::step]
            ]
        markers.append(points_marker)
        if self._lateral_route is not None:
            osm_route = Marker()
            osm_route.header.frame_id = self.frame_id
            osm_route.header.stamp = stamp
            osm_route.ns = "osm_route_reference"
            osm_route.id = 5
            osm_route.type = Marker.LINE_STRIP
            osm_route.action = Marker.ADD
            osm_route.pose.orientation.w = 1.0
            osm_route.scale.x = 0.10
            osm_route.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            osm_route.points = [
                Point(x=float(x), y=float(y), z=0.14)
                for x, y in self._lateral_route.xy
            ]
            markers.append(osm_route)

            # Show the exact route-s intervals in which the fitted corridor is
            # frozen. Close junctions are displayed as the same merged band,
            # matching RouteCorridor.junction_in_guard() rather than drawing a
            # misleading fixed-radius circle around every OSM vertex.
            estimator = getattr(self, "_lateral_estimator", None)
            before = getattr(estimator, "junction_guard_before", 4.0)
            after = getattr(estimator, "junction_guard_after", 3.0)
            active_guard = self._lateral_route.junction_in_guard(
                getattr(self, "_lateral_s", 0.0), before, after
            )
            intervals = self._lateral_route.junction_guard_intervals(
                before, after
            )
            for marker_id, (start_s, end_s, guard_id) in enumerate(intervals):
                zone = Marker()
                zone.header.frame_id = self.frame_id
                zone.header.stamp = stamp
                zone.ns = "junction_freeze_zones"
                zone.id = marker_id
                zone.type = Marker.LINE_STRIP
                zone.action = Marker.ADD
                zone.pose.orientation.w = 1.0
                active = active_guard == guard_id
                zone.scale.x = 0.75 if active else 0.45
                zone.color = (
                    ColorRGBA(r=1.0, g=0.05, b=0.05, a=0.90)
                    if active else
                    ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.55)
                )
                span = max(0.0, end_s - start_s)
                samples = max(2, int(math.ceil(span / 0.5)) + 1)
                for index in range(samples):
                    route_s = start_s + span * index / (samples - 1)
                    route_sample = self._lateral_route.sample(route_s)
                    zone.points.append(Point(
                        x=float(route_sample.x),
                        y=float(route_sample.y),
                        z=0.20,
                    ))
                markers.append(zone)

                label_sample = self._lateral_route.sample(
                    0.5 * (start_s + end_s)
                )
                label = Marker()
                label.header = zone.header
                label.ns = "junction_freeze_labels"
                label.id = marker_id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = float(label_sample.x)
                label.pose.position.y = float(label_sample.y)
                label.pose.position.z = 0.85
                label.pose.orientation.w = 1.0
                label.scale.z = 0.32
                label.color = zone.color
                label.text = (
                    f"JUNCTION FREEZE {start_s:.1f}-{end_s:.1f}m"
                    + (" ACTIVE" if active else "")
                )
                markers.append(label)
        if target is not None and self._lateral_route is not None:
            for marker_id, side, color in (
                (1, "left", (1.0, 0.65, 0.0)),
                (2, "right", (0.0, 1.0, 0.35)),
            ):
                line = Marker()
                line.header.frame_id = self.frame_id
                line.header.stamp = stamp
                line.ns = "road_boundaries"
                line.id = marker_id
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.12
                line.color = ColorRGBA(
                    r=color[0], g=color[1], b=color[2], a=0.95
                )
                for boundary_bin in target.bins:
                    route_sample = self._lateral_route.sample(boundary_bin.s)
                    d = getattr(boundary_bin, side)
                    line.points.append(Point(
                        x=route_sample.x + d * math.sin(route_sample.yaw),
                        y=route_sample.y - d * math.cos(route_sample.yaw),
                        z=0.10,
                    ))
                markers.append(line)
            goal = Marker()
            goal.header.frame_id = self.frame_id
            goal.header.stamp = stamp
            goal.ns = "right_target"
            goal.id = 3
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = float(target.x)
            goal.pose.position.y = float(target.y)
            # Lift the target above road points, costmaps and green OSM waypoint
            # spheres.  At ground level the old 0.45 m sphere was effectively
            # invisible in the outdoor RViz scene.
            goal.pose.position.z = 0.65
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.80
            goal.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            markers.append(goal)

            label = Marker()
            label.header = goal.header
            label.ns = "right_target_label"
            label.id = 6
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(target.x)
            label.pose.position.y = float(target.y)
            label.pose.position.z = 1.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.42
            label.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            label.text = "DYNAMIC TARGET"
            markers.append(label)
            route_sample = self._lateral_route.sample(self._lateral_s)
            right = Marker()
            right.header.frame_id = self.frame_id
            right.header.stamp = stamp
            right.ns = "route_right"
            right.id = 4
            right.type = Marker.ARROW
            right.action = Marker.ADD
            right.pose.orientation.w = 1.0
            right.scale.x = 0.10
            right.scale.y = 0.22
            right.scale.z = 0.30
            right.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            right.points = [
                Point(x=route_sample.x, y=route_sample.y, z=0.25),
                Point(
                    x=route_sample.x + 2.0 * math.sin(route_sample.yaw),
                    y=route_sample.y - 2.0 * math.cos(route_sample.yaw),
                    z=0.25,
                ),
            ]
            markers.append(right)
        self._lateral_debug_pub.publish(MarkerArray(markers=markers))

    def _configure_lateral_route(
        self, lonlat_path, road_start_index, road_end_index, mode
    ):
        """Build the directed route frame and mark only motor-road segments."""
        # Heading alignment is local to one directed route.  In particular, a
        # new goal may reverse the same OSM edge: carrying ``checked/complete``
        # over from the previous route would make the front camera keep looking
        # at the old direction while the estimator searches behind the robot.
        self._reset_route_alignment()
        self._lateral_route_generation += 1
        self._lateral_estimator.reset()
        self._lateral_pending.clear()
        self._lateral_route = None
        self._lateral_s = 0.0
        self._lateral_robot_d = 0.0
        self._lateral_robot_xy = None
        self._lateral_target = None
        self._lateral_target_stamp_ns = None
        self._lateral_seq_d = None
        self._lateral_seq_s = None
        self._lateral_good_frames = 0
        self._lateral_safety_ok = False
        self._lateral_safety_reason = "no_target"
        self._lateral_goal_geometry_ok = False
        self._lateral_last_corridor = None
        self._lateral_avoid_active = False
        self._lateral_avoid_d = None
        self._lateral_avoid_clear_frames = 0
        self._lateral_state = "REFERENCE_ONLY"
        self._lateral_reason = "route_has_no_motor_segment"
        self._lateral_last_log_signature = None
        self._lateral_last_junction_guard = False
        self._lateral_dispatch_blocked_signature = None
        if len(lonlat_path) < 2 or mode != "road":
            return
        start = road_start_index if road_start_index is not None else 0
        end = (
            road_end_index
            if road_end_index is not None
            else len(lonlat_path) - 1
        )
        motor_highways = set(
            self.get_parameter("road_right_offset_highways").value
        )
        points = [self.enu.to_xy(*lonlat_path[0])]
        junction_s = (
            [0.0] if self.graph.is_junction(
                *lonlat_path[0], highways=motor_highways
            ) else []
        )
        route_s = 0.0
        motor_segments = []
        for i in range(len(lonlat_path) - 1):
            next_xy = self.enu.to_xy(*lonlat_path[i + 1])
            segment_length = math.hypot(
                next_xy[0] - points[-1][0], next_xy[1] - points[-1][1]
            )
            if segment_length <= 1e-5:
                continue
            lon = (lonlat_path[i][0] + lonlat_path[i + 1][0]) * 0.5
            lat = (lonlat_path[i][1] + lonlat_path[i + 1][1]) * 0.5
            is_motor = (
                start <= i < end
                and self.graph.highway_at(lon, lat) in motor_highways
            )
            points.append(next_xy)
            motor_segments.append(is_motor)
            route_s += segment_length
            if self.graph.is_junction(
                *lonlat_path[i + 1], highways=motor_highways
            ):
                junction_s.append(route_s)
        if len(points) < 2:
            return
        self._lateral_route = RouteCorridor(
            points, motor_segments, junction_s=junction_s,
            tangent_window=self.get_parameter(
                "semantic_lateral_tangent_window_m"
            ).value,
        )
        if any(motor_segments):
            self._lateral_state = "ACQUIRE_BOUNDARY"
            self._lateral_reason = "waiting_for_semantic_frame"

    def _plan_and_send_to_nav(self, start_lonlat, goal_lonlat):
        road_kwargs = dict(
            entries=self.entries,
            off_network_m=float(self.get_parameter("off_network_m").value),
            search_radius_m=float(
                self.get_parameter("entry_search_radius_m").value
            ),
            offroad_weight=float(self.get_parameter("entry_offroad_weight").value),
            reject_entry_network_crossing=bool(
                self.get_parameter("reject_entry_network_crossing").value
            ),
            respect_turns=self.respect_turns,
        )
        if self.get_parameter("auto_channel_enable").value:
            plan = self.graph.plan_auto(
                start_lonlat[0], start_lonlat[1],
                goal_lonlat[0], goal_lonlat[1],
                direct_max_m=float(self.get_parameter("direct_max_m").value),
                always_direct_m=float(
                    self.get_parameter("always_direct_m").value
                ),
                detour_ratio_threshold=float(
                    self.get_parameter("detour_ratio_threshold").value
                ),
                same_entry_road_m=float(
                    self.get_parameter("same_entry_road_m").value
                ),
                **road_kwargs,
            )
        else:
            plan = self.graph.plan_to_goal(
                start_lonlat[0], start_lonlat[1],
                goal_lonlat[0], goal_lonlat[1], **road_kwargs
            )
            if plan is not None:
                plan = dict(plan, mode="road", decision_reason="auto_disabled")
        if plan is None:
            self.get_logger().error(
                "No path: the goal is beyond the direct-drive limit "
                f"({self.get_parameter('direct_max_m').value:.0f} m) and not "
                "reachable on the road network either. If both ends sit inside "
                "a fenced area, add an entry point in "
                f"{self.get_parameter('entry_gpkg').value}."
            )
            return
        # plan_to_goal may fall back to the goal's nearest road projection when
        # the goal is off-network and has no authored entry.  That is still a
        # useful road route, but it must not become the actual navigation end:
        # always include the requested goal as the final access leg.
        lonlat_path = list(plan["path"])
        length = plan["length_m"]
        road_start_index = plan["road_start_index"]
        road_end_index = plan["road_end_index"]
        if lonlat_path:
            end_x, end_y = self.enu.to_xy(*lonlat_path[-1])
            goal_x, goal_y = self.enu.to_xy(*goal_lonlat)
            final_leg_m = math.hypot(goal_x - end_x, goal_y - end_y)
            if final_leg_m > 0.05:
                # The old last point is the end of the road section.  Marking it
                # lets _densify_route leave this off-network hop intact and lets
                # batched navigation hand the true goal to NavigateToPose.
                if road_end_index is None:
                    road_end_index = len(lonlat_path) - 1
                lonlat_path.append(goal_lonlat)
                length += final_leg_m
                plan["path"] = lonlat_path
                plan["length_m"] = length
                plan["road_end_index"] = road_end_index
                plan["offroad_m"] = plan.get("offroad_m", 0.0) + final_leg_m
                self.get_logger().info(
                    f"Goal is off the planned road endpoint; appended the "
                    f"remaining {final_leg_m:.1f} m access leg."
                )
        mode = plan["mode"]
        entry_enable = self.get_parameter("entry_enable").value
        radius_str = f"{self.get_parameter('entry_search_radius_m').value:.0f}"
        self._log_channel_decision(plan)
        self._publish_channel_decision(plan)
        # Entry / off-network narration describes a road route only. Both direct
        # modes reuse road_end_index to mark their un-densified hops, so without
        # this guard they would trip the same branches and report driving through
        # an opening they never touch.
        if mode == "road":
            if road_start_index is not None:
                s_lead = plan["start_entry_lead_m"]
                s_through = (
                    f"{s_lead:.0f} m through the opening + " if s_lead else ""
                )
                self.get_logger().info(
                    f"Start is {plan['start_offnetwork_m']:.1f} m off-network -> "
                    f"exiting via '{plan['start_entry_name']}' "
                    f"({s_through}{plan['start_offroad_m']:.0f} m to road network)."
                )
            elif plan["start_entry_missing"] and entry_enable:
                self.get_logger().warn(
                    f"Start is {plan['start_offnetwork_m']:.1f} m off-network but "
                    f"no entry point is within {radius_str} m of it; snapping to "
                    f"the nearest road, which may sit on the wrong side of a "
                    f"fence. Add a point closer to the start in "
                    f"{self.get_parameter('entry_gpkg').value}."
                )
            if road_end_index is not None:
                lead = plan["entry_lead_m"]
                through = f"{lead:.0f} m through the opening + " if lead else ""
                self.get_logger().info(
                    f"Goal is {plan['goal_offnetwork_m']:.1f} m off-network -> "
                    f"entering via '{plan['entry_name']}' "
                    f"({plan['road_m']:.0f} m on roads + "
                    f"{through}{plan['offroad_m']:.0f} m access leg)."
                )
            elif plan["entry_missing"] and entry_enable:
                self.get_logger().warn(
                    f"Goal is {plan['goal_offnetwork_m']:.1f} m off-network but no "
                    f"entry point is within {radius_str} m "
                    f"of it (none authored, or the nearest opening is too far); "
                    f"using the nearest road followed by a straight final "
                    f"access leg, which may cross a fence. Add a point closer "
                    f"to the goal in "
                    f"{self.get_parameter('entry_gpkg').value}."
                )
        # Match road-network marker height for RViz; keep Nav2 goals at z=0.
        path_z = float(self.get_parameter("network_z").value)
        self._publish_path_msg(
            self.path_pub, lonlat_path, length, label="campus path", z=path_z
        )
        # Preserve the unshifted, directed OSM geometry as the longitudinal
        # reference. Semantic observations may change d, never the route order.
        self._configure_lateral_route(
            lonlat_path, road_start_index, road_end_index, mode
        )

        step = self.get_parameter("nav_waypoint_spacing_m").value
        nav_lonlat, nav_entry_index = self._densify_route(
            lonlat_path, step, road_end_index,
            road_start_index=road_start_index,
        )
        # The first point is the current robot position; Nav2 should drive to
        # upcoming waypoints, not re-goal itself at its current pose.
        if len(nav_lonlat) > 1:
            nav_lonlat = nav_lonlat[1:]
            if nav_entry_index is not None:
                nav_entry_index = max(0, nav_entry_index - 1)
        nav_msg = self._publish_path_msg(
            self.nav_waypoints_pub, nav_lonlat, length, label="nav waypoints", z=0.0
        )
        self._publish_nav_waypoint_markers(nav_msg)
        if self.get_parameter("send_to_nav2").value:
            if self.get_parameter("nav2_batch_enable").value:
                self._start_batched_route(
                    nav_msg.poses, entry_index=nav_entry_index, mode=mode
                )
            else:
                self._send_nav2_waypoints(nav_msg.poses)

    # -- channel decision reporting -----------------------------------------
    def _log_channel_decision(self, plan):
        """One line saying which way we are going and what decided it."""
        rt = plan["road_total_m"]
        ratio = plan["detour_ratio"]
        rt_s = "unreachable" if rt is None else f"{rt:.0f} m"
        ratio_s = "n/a" if ratio is None else f"{ratio:.2f}x"
        head = (
            f"straight {plan['straight_m']:.0f} m, road route {rt_s} "
            f"({ratio_s}) -> "
        )
        mode = plan["mode"]
        reason = plan["decision_reason"]
        if mode == "road":
            self.get_logger().info(f"{head}following the road network [{reason}]")
        elif mode == "direct":
            self.get_logger().info(
                f"{head}driving straight to the goal, bypassing the road "
                f"network [{reason}]"
            )
        else:  # direct_via_entry
            self.get_logger().info(
                f"{head}too far for one hop, so driving it in two via "
                f"'{plan['entry_name']}' [{reason}]"
            )

    def _publish_channel_decision(self, plan):
        if not hasattr(self, "decision_pub"):
            return
        rt = plan["road_total_m"]
        ratio = plan["detour_ratio"]
        self.decision_pub.publish(String(data=(
            f"mode={plan['mode']} reason={plan['decision_reason']} "
            f"straight_m={plan['straight_m']:.1f} "
            f"road_total_m={'' if rt is None else f'{rt:.1f}'} "
            f"road_m={plan['road_m']:.1f} "
            f"detour_ratio={'' if ratio is None else f'{ratio:.3f}'}"
        )))

    def _densify_route(self, lonlat_path, step_m, road_end_index,
                       road_start_index=None):
        """Densify the on-road segment only; return (points, goal_entry_index).

        Segments before road_start_index (start off-road prefix) and after
        road_end_index (goal off-road tail) are left un-densified — Nav2 plans
        each off-road hop through its local costmap. goal_entry_index is the
        handover pose for the precise final approach (second-to-last waypoint),
        or None when there is no goal-side off-road tail.
        """
        start_idx = road_start_index if road_start_index is not None else 0
        end_idx = road_end_index if road_end_index is not None else len(lonlat_path) - 1
        prefix = list(lonlat_path[:start_idx])
        road_part = lonlat_path[start_idx:end_idx + 1]
        suffix = list(lonlat_path[end_idx + 1:])
        dense_road = self._densify(road_part, step_m)
        offset_m = float(self.get_parameter("road_right_offset_m").value)
        motor_highways = set(
            self.get_parameter("road_right_offset_highways").value
        )
        if abs(offset_m) > 1e-6 and len(dense_road) >= 2:
            selected = [
                self.graph.highway_at(lon, lat) in motor_highways
                for lon, lat in dense_road
            ]
            dense_road = self._offset_selected_route_right(
                dense_road, offset_m, selected
            )
        out = prefix + dense_road + suffix
        goal_entry_index = len(out) - 2 if suffix else None
        return out, goal_entry_index

    def _offset_selected_route_right(self, lonlat_path, offset_m, selected):
        """Right-offset only points belonging to selected OSM road types."""
        if len(lonlat_path) != len(selected):
            raise ValueError("selected flags must match route points")
        xy = [self.enu.to_xy(lon, lat) for lon, lat in lonlat_path]
        shifted = []
        for i, ((x, y), should_offset) in enumerate(zip(xy, selected)):
            if not should_offset:
                shifted.append(lonlat_path[i])
                continue
            tangents = []
            if i > 0:
                dx, dy = x - xy[i - 1][0], y - xy[i - 1][1]
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    tangents.append((dx / norm, dy / norm))
            if i + 1 < len(xy):
                dx, dy = xy[i + 1][0] - x, xy[i + 1][1] - y
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    tangents.append((dx / norm, dy / norm))
            tx = sum(t[0] for t in tangents)
            ty = sum(t[1] for t in tangents)
            norm = math.hypot(tx, ty)
            if norm <= 1e-6:
                tx, ty = tangents[-1] if tangents else (1.0, 0.0)
            else:
                tx, ty = tx / norm, ty / norm
            shifted.append(self.enu.to_lonlat(
                x + offset_m * ty, y - offset_m * tx
            ))
        return shifted

    # -- rolling waypoint batching ------------------------------------------
    @staticmethod
    def _pose_xy(pose):
        return pose.pose.position.x, pose.pose.position.y

    @staticmethod
    def _segment_tangent(pose_a, pose_b):
        ax, ay = RoutePlanner._pose_xy(pose_a)
        bx, by = RoutePlanner._pose_xy(pose_b)
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None
        return dx / length, dy / length

    def _waypoint_passed(self, rx, ry, poses, i, reach_tol):
        """True when the robot reached or overshot waypoint i along the route."""
        wx, wy = self._pose_xy(poses[i])
        dx, dy = rx - wx, ry - wy
        dist = math.hypot(dx, dy)
        if dist <= reach_tol:
            return True
        passed_margin = max(0.5, reach_tol * 0.3)
        # A waypoint can be overshot along either leg, so accept whichever matches:
        # the robot drove past it on the way in, or it cut the corner and is already
        # down the way out. Testing only the outgoing leg made an entry's road
        # projection unpassable, since that leg leaves the road at a right angle and
        # keeps the dot product near zero no matter how far along the road we drive.
        tangents = []
        if i > 0:
            tangents.append(self._segment_tangent(poses[i - 1], poses[i]))
        if i < len(poses) - 1:
            tangents.append(self._segment_tangent(poses[i], poses[i + 1]))
        return any(
            t is not None and (dx * t[0] + dy * t[1]) > passed_margin
            for t in tangents
        )

    def _advance_nav_cursor(self, rx, ry, poses, cursor, reach_tol, stop_index=None):
        """Drop reached/overshot waypoints so Nav2 is not sent goals behind the robot.

        stop_index bounds the cursor: the precise final approach keeps the last
        pose out of reach of the fly-through tolerance so it is never dropped.
        """
        limit = len(poses) if stop_index is None else stop_index
        while cursor < limit and self._waypoint_passed(rx, ry, poses, cursor, reach_tol):
            cursor += 1
        return cursor

    def _advance_nav_cursor_by_route_progress(
        self, rx, ry, poses, cursor, reach_tol, stop_index=None
    ):
        """Retire motor-road anchors by s, not by hitting their OSM lateral d."""
        limit = len(poses) if stop_index is None else stop_index
        while cursor < limit:
            if self._nav_route_motor[cursor]:
                passed = self._nav_route_s[cursor] <= self._lateral_s + reach_tol
            else:
                # Entry/access/direct portions retain exact geometric semantics.
                passed = self._waypoint_passed(
                    rx, ry, poses, cursor, reach_tol
                )
            if not passed:
                break
            cursor += 1
        return cursor

    def _lateral_closed_loop_required(self):
        # Before the GPS heading snap, campus-map yaw is not trustworthy and
        # the robot deliberately has to move to establish it.  Do not deadlock
        # that bootstrap on a camera corridor expressed in the unaligned frame.
        if self._heading_status_seen and not self._heading_locked:
            return False
        if (
            not self.get_parameter("semantic_lateral_enabled").value
            or self.get_parameter("semantic_lateral_shadow_mode").value
            or self._nav_mode != "road"
            or self._lateral_route is None
        ):
            return False
        return self._lateral_route.sample(self._lateral_s).motor_road

    def _lateral_closed_loop_pose(self):
        if self._lateral_target is None:
            self._lateral_reason = "dynamic_target_missing"
            return None
        if self._lateral_state not in (
            "TRACK_RIGHT", "SEEK_RIGHT_EDGE", "JUNCTION_GUARD",
            "AVOID_OBSTACLE", "HOLD_LAST"
        ):
            return None
        age = (
            self.get_clock().now().nanoseconds - self._lateral_target_stamp_ns
        ) / 1e9
        if age > float(self.get_parameter("semantic_lateral_hold_last_s").value):
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = "dynamic_target_stale"
            return None
        target = self._lateral_target
        # Reject a result that has fallen behind current route progress while a
        # slow worker was running.
        if target.s + 0.5 < self._lateral_s:
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = "dynamic_target_behind_robot"
            return None
        geometry_target, extended, geometry_reason = (
            self._ensure_target_beyond_goal_tolerance(target)
        )
        if geometry_target is None:
            self._lateral_goal_geometry_ok = False
            self._lateral_safety_ok = False
            self._lateral_safety_reason = geometry_reason
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = geometry_reason
            return None
        if extended:
            target = geometry_target
            self._lateral_target = target
            self._lateral_target_seq += 1
            self._lateral_seq_d = target.d
            self._lateral_seq_s = target.s
            self._lateral_reason = target.state_reason
        self._lateral_goal_geometry_ok = True
        safe, reason = self._lateral_target_is_safe(target.x, target.y)
        self._lateral_safety_ok = safe
        self._lateral_safety_reason = reason
        if not safe:
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = reason
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target.x)
        pose.pose.position.y = float(target.y)
        qx, qy, qz, qw = _quat_from_yaw(target.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _lateral_target_is_safe(self, x, y):
        """Check a circular footprint in the fused Nav2 local costmap."""
        grid = self._lateral_costmap
        if grid is None:
            return False, "safety_costmap_missing"
        stamp_ns = self._stamp_ns(grid.header)
        max_age = float(
            self.get_parameter("semantic_lateral_costmap_max_age_s").value
        )
        if stamp_ns and self.get_clock().now().nanoseconds - stamp_ns > max_age * 1e9:
            return False, "safety_costmap_stale"
        target_frame = grid.header.frame_id or self.frame_id
        if target_frame != self.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, self.frame_id, Time(),
                    timeout=Duration(seconds=0.05),
                ).transform
            except TransformException:
                return False, "safety_costmap_tf_unavailable"
            yaw = _yaw_from_quat(transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = (
                transform.translation.x + c * x - s * y,
                transform.translation.y + s * x + c * y,
            )
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return False, "safety_costmap_invalid"
        origin = grid.info.origin
        origin_yaw = _yaw_from_quat(origin.orientation)
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        center_x = int(math.floor(local_x / resolution))
        center_y = int(math.floor(local_y / resolution))
        width = int(grid.info.width)
        height = int(grid.info.height)
        radius = float(
            self.get_parameter("semantic_lateral_safety_radius_m").value
        )
        cells = int(math.ceil(radius / resolution))
        if (
            center_x - cells < 0 or center_y - cells < 0
            or center_x + cells >= width or center_y + cells >= height
        ):
            return False, "target_outside_safety_costmap"
        lethal = int(
            self.get_parameter("semantic_lateral_lethal_occupancy").value
        )
        radius2 = radius * radius
        for gy in range(center_y - cells, center_y + cells + 1):
            for gx in range(center_x - cells, center_x + cells + 1):
                cell_dx = (gx + 0.5) * resolution - local_x
                cell_dy = (gy + 0.5) * resolution - local_y
                if cell_dx * cell_dx + cell_dy * cell_dy > radius2:
                    continue
                value = int(grid.data[gy * width + gx])
                if value < 0:
                    return False, "target_footprint_unknown"
                if value >= lethal:
                    return False, "target_footprint_lethal"
        return True, "ok"

    def _limit_lateral_target_from_robot(self, target):
        """Limit lateral motion from the achieved robot pose, not old goals.

        Advancing from the previous target allows an immobile robot's target to
        walk farther away on every semantic frame.  Anchoring the command to
        the measured route projection makes lateral progress contingent on the
        vehicle actually following it.
        """
        max_step = max(0.0, float(self.get_parameter(
            "semantic_lateral_max_lateral_step_m"
        ).value))
        delta_d = target.d - self._lateral_robot_d
        if abs(delta_d) <= max_step:
            return target
        limited_d = self._lateral_robot_d + math.copysign(max_step, delta_d)
        route_sample = self._lateral_route.sample(target.s)
        return replace(
            target,
            d=float(limited_d),
            x=float(route_sample.x + limited_d * math.sin(route_sample.yaw)),
            y=float(route_sample.y - limited_d * math.cos(route_sample.yaw)),
        )

    def _fit_target_in_safety_window(self, target):
        """Find the farthest safe receding target inside the local costmap.

        Outside the rolling grid is not treated as free space.  The target is
        pulled back along the selected OSM branch, and if necessary toward the
        achieved robot offset, until its full footprint is represented and
        safe.  Lethal/unknown/missing costmaps retain their existing fail-closed
        behavior.
        """
        safe, reason = self._lateral_target_is_safe(target.x, target.y)
        if safe or reason != "target_outside_safety_costmap":
            return target, False

        forward = max(0.0, target.s - self._lateral_s)
        minimum = max(0.0, float(self.get_parameter(
            "semantic_lateral_min_lookahead_m"
        ).value))
        minimum = min(forward, minimum)
        longitudinal_step = max(0.05, float(self.get_parameter(
            "semantic_lateral_safety_lookahead_step_m"
        ).value))
        forwards = []
        candidate_forward = forward
        while candidate_forward > minimum + 1e-6:
            forwards.append(candidate_forward)
            candidate_forward -= longitudinal_step
        forwards.append(minimum)

        lateral_step = max(0.05, float(self.get_parameter(
            "semantic_lateral_safety_inward_step_m"
        ).value))
        lateral_delta = self._lateral_robot_d - target.d
        lateral_values = [target.d]
        if abs(lateral_delta) > 1e-6:
            count = int(math.ceil(abs(lateral_delta) / lateral_step))
            direction = math.copysign(1.0, lateral_delta)
            lateral_values.extend(
                target.d + direction * min(abs(lateral_delta), i * lateral_step)
                for i in range(1, count + 1)
            )

        # Progress takes priority, then closeness to the desired lateral
        # offset. This preserves the held junction lane whenever it fits.
        for candidate_forward in forwards:
            candidate_s = min(
                self._lateral_route.total_length,
                self._lateral_s + candidate_forward,
            )
            route_sample = self._lateral_route.sample(candidate_s)
            for candidate_d in lateral_values:
                candidate_x = (
                    route_sample.x + candidate_d * math.sin(route_sample.yaw)
                )
                candidate_y = (
                    route_sample.y - candidate_d * math.cos(route_sample.yaw)
                )
                candidate_safe, _ = self._lateral_target_is_safe(
                    candidate_x, candidate_y
                )
                if candidate_safe:
                    return replace(
                        target,
                        x=float(candidate_x),
                        y=float(candidate_y),
                        yaw=float(route_sample.yaw),
                        s=float(candidate_s),
                        d=float(candidate_d),
                        state_reason="costmap_receding_target",
                    ), True
        return target, False

    def _target_robot_distance(self, target):
        robot_xy = getattr(self, "_lateral_robot_xy", None)
        if robot_xy is None:
            return float("nan")
        rx, ry = robot_xy
        return math.hypot(target.x - rx, target.y - ry)

    def _ensure_target_beyond_goal_tolerance(self, target):
        """Extend a folded junction target beyond Nav2's reached radius."""
        if getattr(self, "_lateral_robot_xy", None) is None:
            return target, False, "robot_pose_unavailable"
        minimum = max(0.0, float(self.get_parameter(
            "semantic_lateral_min_goal_distance_m"
        ).value))
        if self._target_robot_distance(target) >= minimum:
            return target, False, "ok"

        step = max(0.05, float(self.get_parameter(
            "semantic_lateral_safety_lookahead_step_m"
        ).value))
        max_forward = max(
            target.s - self._lateral_s,
            float(self.get_parameter("semantic_lateral_far_m").value),
        )
        lateral_step = max(0.05, float(self.get_parameter(
            "semantic_lateral_safety_inward_step_m"
        ).value))
        lateral_delta = self._lateral_robot_d - target.d
        lateral_values = [target.d]
        if abs(lateral_delta) > 1e-6:
            count = int(math.ceil(abs(lateral_delta) / lateral_step))
            direction = math.copysign(1.0, lateral_delta)
            lateral_values.extend(
                target.d + direction * min(abs(lateral_delta), i * lateral_step)
                for i in range(1, count + 1)
            )

        forward = max(0.0, target.s - self._lateral_s) + step
        while forward <= max_forward + 1e-6:
            candidate_s = min(
                self._lateral_route.total_length,
                self._lateral_s + forward,
            )
            route_sample = self._lateral_route.sample(candidate_s)
            for candidate_d in lateral_values:
                candidate = replace(
                    target,
                    x=float(
                        route_sample.x
                        + candidate_d * math.sin(route_sample.yaw)
                    ),
                    y=float(
                        route_sample.y
                        - candidate_d * math.cos(route_sample.yaw)
                    ),
                    yaw=float(route_sample.yaw),
                    s=float(candidate_s),
                    d=float(candidate_d),
                    state_reason="goal_distance_extended",
                )
                if self._target_robot_distance(candidate) < minimum:
                    continue
                safe, _ = self._lateral_target_is_safe(
                    candidate.x, candidate.y
                )
                if safe:
                    return candidate, True, "ok"
            if candidate_s >= self._lateral_route.total_length - 1e-6:
                break
            forward += step
        return None, False, "target_inside_goal_tolerance"

    def _shift_target_inward_for_safety(self, target):
        """Move an obstructed right-side target toward road center, if safe.

        The semantic edge remains the preferred target.  This search is only
        used when that target's footprint overlaps a lethal fused-costmap cell;
        it approaches the fitted road center from either side and never crosses
        it.  Supporting both sides matters while the lateral slew limiter is
        still moving a robot from the OSM-biased side toward the right edge.
        Unknown, stale, missing, or out-of-window failures remain fail-closed.
        """
        safe, reason = self._lateral_target_is_safe(target.x, target.y)
        if safe or reason != "target_footprint_lethal":
            return target, False
        distance = target.center - target.d
        if abs(distance) <= 1e-6:
            return target, False
        step = max(
            0.05,
            float(self.get_parameter(
                "semantic_lateral_safety_inward_step_m"
            ).value),
        )
        route_sample = self._lateral_route.sample(target.s)
        direction = math.copysign(1.0, distance)
        count = int(math.ceil(abs(distance) / step))
        for index in range(1, count + 1):
            move = min(abs(distance), index * step)
            candidate_d = target.d + direction * move
            candidate_x = (
                route_sample.x + candidate_d * math.sin(route_sample.yaw)
            )
            candidate_y = (
                route_sample.y - candidate_d * math.cos(route_sample.yaw)
            )
            candidate_safe, _ = self._lateral_target_is_safe(
                candidate_x, candidate_y
            )
            if candidate_safe:
                return replace(
                    target,
                    x=float(candidate_x),
                    y=float(candidate_y),
                    d=float(candidate_d),
                    state_reason="costmap_inward_shift",
                ), True
        return target, False

    def _avoid_obstacle_in_corridor(self, target):
        """Select a safe 2-D target inside the frozen road corridor.

        A parked vehicle invalidates one preferred endpoint, not the complete
        route.  Search lateral offsets at the nominal progress first, then
        nearby longitudinal stations. Unknown/out-of-grid cells remain unsafe.
        The chosen side is held for several clear frames to prevent oscillation.
        """
        safe, reason = self._lateral_target_is_safe(target.x, target.y)
        release_frames = max(1, int(self.get_parameter(
            "semantic_lateral_avoid_release_frames"
        ).value))
        if safe:
            if not self._lateral_avoid_active:
                return target, False
            held = self._corridor_candidate(
                target, target.s, self._lateral_avoid_d,
                "obstacle_avoid_hold",
            )
            held_safe, _ = self._lateral_target_is_safe(held.x, held.y)
            if held_safe:
                self._lateral_avoid_clear_frames += 1
                if self._lateral_avoid_clear_frames < release_frames:
                    return held, True
            self._lateral_avoid_active = False
            self._lateral_avoid_d = None
            self._lateral_avoid_clear_frames = 0
            return target, False
        if reason != "target_footprint_lethal":
            return target, False

        self._lateral_avoid_clear_frames = 0
        margin = float(self.get_parameter(
            "semantic_lateral_inside_margin_m"
        ).value)
        left_limit = target.left + margin
        right_limit = target.right - margin
        if left_limit > right_limit:
            return target, False
        lateral_step = max(0.05, float(self.get_parameter(
            "semantic_lateral_safety_inward_step_m"
        ).value))
        grid_values = np.arange(
            left_limit, right_limit + 0.5 * lateral_step, lateral_step,
            dtype=np.float64,
        ).tolist()
        preferred = float(np.clip(target.d, left_limit, right_limit))
        lateral_values = [preferred]
        lateral_values.extend(sorted(
            grid_values,
            key=lambda value: (
                abs(value - (
                    self._lateral_avoid_d
                    if self._lateral_avoid_active
                    and self._lateral_avoid_d is not None
                    else preferred
                )),
                abs(value - preferred),
            ),
        ))
        # Deduplicate the clipped preferred value and floating-point grid hits.
        lateral_values = list(dict.fromkeys(
            round(float(value), 6) for value in lateral_values
        ))

        base_forward = max(0.0, target.s - self._lateral_s)
        forward_step = max(0.1, float(self.get_parameter(
            "semantic_lateral_safety_lookahead_step_m"
        ).value))
        max_forward = max(base_forward, float(self.get_parameter(
            "semantic_lateral_avoid_max_lookahead_m"
        ).value))
        min_forward = max(0.0, float(self.get_parameter(
            "semantic_lateral_min_lookahead_m"
        ).value))
        forward_values = [base_forward]
        value = base_forward + forward_step
        while value <= max_forward + 1e-6:
            forward_values.append(value)
            value += forward_step
        value = base_forward - forward_step
        while value >= min_forward - 1e-6:
            forward_values.append(value)
            value -= forward_step

        minimum_distance = float(self.get_parameter(
            "semantic_lateral_min_goal_distance_m"
        ).value)
        for forward in forward_values:
            candidate_s = min(
                self._lateral_route.total_length,
                self._lateral_s + forward,
            )
            for candidate_d in lateral_values:
                candidate = self._corridor_candidate(
                    target, candidate_s, candidate_d,
                    "corridor_obstacle_avoid",
                )
                distance = self._target_robot_distance(candidate)
                if math.isfinite(distance) and distance < minimum_distance:
                    continue
                candidate_safe, _ = self._lateral_target_is_safe(
                    candidate.x, candidate.y
                )
                if candidate_safe:
                    self._lateral_avoid_active = True
                    self._lateral_avoid_d = float(candidate_d)
                    return candidate, True
            if candidate_s >= self._lateral_route.total_length - 1e-6:
                break
        return target, False

    def _corridor_candidate(self, target, candidate_s, candidate_d, reason):
        sample = self._lateral_route.sample(candidate_s)
        return replace(
            target,
            x=float(sample.x + candidate_d * math.sin(sample.yaw)),
            y=float(sample.y - candidate_d * math.cos(sample.yaw)),
            yaw=float(sample.yaw),
            s=float(candidate_s),
            d=float(candidate_d),
            state_reason=reason,
        )

    @staticmethod
    def _shortest_angle(target, current):
        return math.atan2(
            math.sin(float(target) - float(current)),
            math.cos(float(target) - float(current)),
        )

    def _robot_pose(self):
        robot_frame = self.get_parameter("robot_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, robot_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException:
            return None
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            _yaw_from_quat(tf.transform.rotation),
        )

    def _robot_xy(self):
        pose = self._robot_pose()
        return None if pose is None else pose[:2]

    def _cancel_route_spin(self):
        """Cancel only the route-alignment behavior, invalidating callbacks."""
        self._route_align_seq += 1
        handle = self._route_align_goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 -- best-effort cancellation
                self.get_logger().warn(
                    f"Failed to cancel route-alignment Spin: {exc}"
                )
        self._route_align_goal_handle = None
        self._route_align_pending = False
        self._route_align_active = False

    def _reset_route_alignment(self):
        """Start a fresh heading check for a newly selected directed route."""
        self._cancel_route_spin()
        self._route_align_checked = False
        self._route_align_required = False
        self._route_align_complete = False
        self._route_align_retries = 0
        self._route_align_error_rad = None

    def _complete_route_alignment(self):
        """Admit only fresh, post-turn camera frames into the road fit."""
        self._route_align_required = False
        self._route_align_complete = True
        self._route_align_pending = False
        self._route_align_active = False
        self._route_align_goal_handle = None
        self._lateral_route_generation += 1
        self._lateral_estimator.reset()
        self._lateral_pending.clear()
        self._lateral_target = None
        self._lateral_target_stamp_ns = None
        self._lateral_good_frames = 0
        self._lateral_seq_d = None
        self._lateral_seq_s = None
        self._lateral_safety_ok = False
        self._lateral_safety_reason = "waiting_for_post_align_target"
        self._lateral_avoid_active = False
        self._lateral_avoid_d = None
        self._lateral_avoid_clear_frames = 0
        self._lateral_state = "ACQUIRE_BOUNDARY"
        self._lateral_reason = "waiting_for_post_align_semantic_frame"
        self.get_logger().info(
            "Route heading aligned; discarded pre-turn semantic frames and "
            "started fresh boundary acquisition."
        )

    def _route_heading_alignment_tick(self, robot_yaw):
        """Return True while Nav2 driving must wait for an initial in-place turn."""
        if (
            not self.get_parameter(
                "semantic_lateral_align_heading_enable"
            ).value
            or self._lateral_route is None
            or not self.get_parameter("semantic_lateral_enabled").value
            or self.get_parameter("semantic_lateral_shadow_mode").value
            or self._nav_mode != "road"
            or not self._lateral_route.sample(self._lateral_s).motor_road
        ):
            self._route_align_complete = True
            return False

        # Before GPS heading lock, campus_map yaw is not physically meaningful.
        # Preserve the existing bootstrap (ordinary OSM navigation) until it
        # locks; _lateral_closed_loop_required() is disabled in the same phase.
        if self._heading_status_seen and not self._heading_locked:
            return False

        route_yaw = self._lateral_route.sample(self._lateral_s).yaw
        error = self._shortest_angle(route_yaw, robot_yaw)
        self._route_align_error_rad = error
        enter = math.radians(float(self.get_parameter(
            "semantic_lateral_align_enter_deg"
        ).value))
        exit_angle = math.radians(float(self.get_parameter(
            "semantic_lateral_align_exit_deg"
        ).value))

        if not self._route_align_checked:
            self._route_align_checked = True
            if abs(error) <= enter:
                self._route_align_complete = True
                return False
            self._route_align_required = True
            self._route_align_complete = False
            self.get_logger().info(
                f"Route begins {math.degrees(error):.1f} deg from camera "
                "heading; aligning in place before semantic right tracking."
            )

        if self._route_align_complete:
            return False
        self._lateral_state = "ALIGN_ROUTE_HEADING"
        self._lateral_reason = (
            f"route_heading_error:{math.degrees(error):.1f}deg"
        )
        self._lateral_target = None
        self._lateral_target_stamp_ns = None

        if self._route_align_pending or self._route_align_active:
            return True
        if abs(error) <= exit_angle:
            self._complete_route_alignment()
            return False

        maximum = int(self.get_parameter(
            "semantic_lateral_align_max_retries"
        ).value)
        if self._route_align_retries > maximum:
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = "route_heading_alignment_failed"
            return True
        if self.route_spin_action_client is None:
            self._lateral_state = "DEGRADED_STOP"
            self._lateral_reason = "route_spin_client_missing"
            return True
        action_name = self.get_parameter(
            "semantic_lateral_align_action_name"
        ).value
        if not self.route_spin_action_client.wait_for_server(timeout_sec=0.2):
            self._lateral_reason = f"route_spin_unavailable:{action_name}"
            self.get_logger().warn(
                f"Holding rearward Campus route: Spin action '{action_name}' "
                "is unavailable.",
                throttle_duration_sec=5.0,
            )
            return True

        goal = Spin.Goal()
        goal.target_yaw = float(error)
        goal.time_allowance = Duration(
            seconds=float(self.get_parameter(
                "semantic_lateral_align_timeout_s"
            ).value)
        ).to_msg()
        self._route_align_pending = True
        self._route_align_seq += 1
        seq = self._route_align_seq
        future = self.route_spin_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, s=seq: self._on_route_spin_response(f, s)
        )
        return True

    def _on_route_spin_response(self, future, seq):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 -- action transport failure
            if seq == self._route_align_seq:
                self._route_align_pending = False
                self._route_align_retries += 1
                self._lateral_reason = f"route_spin_send_error:{exc}"
            return
        if seq != self._route_align_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self._route_align_pending = False
        if not handle.accepted:
            self._route_align_retries += 1
            self._lateral_reason = "route_spin_rejected"
            return
        self._route_align_goal_handle = handle
        self._route_align_active = True
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_route_spin_result(f, s)
        )

    def _on_route_spin_result(self, future, seq):
        if seq != self._route_align_seq:
            return
        self._route_align_goal_handle = None
        self._route_align_active = False
        try:
            wrapped = future.result()
            result = wrapped.result
            status = getattr(wrapped, "status", GoalStatus.STATUS_SUCCEEDED)
            error_code = int(getattr(result, "error_code", 0))
            error_msg = str(getattr(result, "error_msg", ""))
        except Exception as exc:  # noqa: BLE001 -- action transport failure
            status = GoalStatus.STATUS_ABORTED
            error_code = -1
            error_msg = str(exc)
        if status != GoalStatus.STATUS_SUCCEEDED or error_code:
            self._route_align_retries += 1
            self._lateral_reason = (
                f"route_spin_failed:{status}/{error_code}:{error_msg}"
            )
            self.get_logger().warn(
                "Route-alignment Spin failed; holding navigation and retrying "
                f"within policy ({self._lateral_reason})."
            )
            return

        pose = self._robot_pose()
        if pose is None:
            self._lateral_reason = "route_spin_done_waiting_for_tf"
            return
        route_yaw = self._lateral_route.sample(self._lateral_s).yaw
        error = self._shortest_angle(route_yaw, pose[2])
        self._route_align_error_rad = error
        exit_angle = math.radians(float(self.get_parameter(
            "semantic_lateral_align_exit_deg"
        ).value))
        if abs(error) <= exit_angle:
            self._complete_route_alignment()
        else:
            self._route_align_retries += 1
            self._lateral_reason = (
                f"route_spin_residual:{math.degrees(error):.1f}deg"
            )

    def _start_batched_route(self, poses, entry_index=None, mode="road"):
        """Store the full route and let the timer feed it to Nav2 in batches."""
        self._cancel_nav2_goal()
        self._cancel_final_goal()
        self._cancel_route_spin()
        self._route_align_checked = False
        self._route_align_required = False
        self._route_align_complete = False
        self._route_align_retries = 0
        self._route_align_error_rad = None
        self._final_phase = False
        self._final_retry = 0
        self._final_cancel_pending = False
        self._final_wait_ticks = 0
        self._nav_mode = mode
        self._nav_entry_index = entry_index
        self._nav_route_poses = list(poses)
        self._nav_route_s = []
        self._nav_route_motor = []
        if self._lateral_route is not None:
            hint = 0.0
            for pose in self._nav_route_poses:
                px, py = self._pose_xy(pose)
                projection = self._lateral_route.project(
                    px, py, s_hint=hint, search_back=8.0,
                    search_forward=100.0,
                )
                hint = max(hint, projection.s)
                self._nav_route_s.append(projection.s)
                self._nav_route_motor.append(
                    self._lateral_route.sample(projection.s).motor_road
                )
        self._nav_cursor = 0
        self._nav_goal_active = False
        self._nav_send_pending = False
        self._nav_batch_last_xy = None
        self._nav_sent_key = None
        self._nav_last_succeeded_key = None
        self._nav_last_succeeded_route_s = None
        self._nav_force_resend = False
        self._nav_running = bool(poses)
        if self._nav_running:
            self.get_logger().info(
                f"Batched route started: {len(poses)} waypoints, feeding Nav2 "
                f"within {self.get_parameter('nav2_batch_radius_m').value:.0f} m."
            )
            self._nav_batch_tick()

    def _nav_batch_tick(self):
        if not self._nav_running or not self._nav_route_poses:
            return
        robot_pose = self._robot_pose()
        if robot_pose is None:
            return
        rx, ry, robot_yaw = robot_pose
        self._lateral_robot_xy = (rx, ry)
        poses = self._nav_route_poses
        n = len(poses)

        def dist(i):
            px, py = self._pose_xy(poses[i])
            return math.hypot(px - rx, py - ry)

        # Checked before _nav_send_pending: the proximity fallback must still be
        # able to end the route if a final goal send never gets a response.
        if self._final_phase:
            self._final_phase_tick(rx, ry, dist(n - 1))
            return
        if self._nav_send_pending:
            return

        precise_final = self._precise_final_enabled()
        reach_tol = float(self.get_parameter("nav2_reach_tol_m").value)
        prev_cursor = self._nav_cursor
        if self._lateral_route is not None:
            projection = self._lateral_route.project(
                rx, ry, s_hint=self._lateral_s
            )
            # Route progress is monotonic for a forward Campus Nav route. The
            # hint window prevents a nearby parallel/self-crossing segment from
            # stealing the projection.
            self._lateral_s = max(self._lateral_s, projection.s)
            self._lateral_robot_d = projection.d
        if self._route_heading_alignment_tick(robot_yaw):
            return
        # With the precise approach the last pose belongs to the final phase, so
        # the fly-through tolerance must not consume it.
        stop_index = n - 1 if precise_final else n
        if len(self._nav_route_s) == n:
            self._nav_cursor = self._advance_nav_cursor_by_route_progress(
                rx, ry, poses, self._nav_cursor, reach_tol,
                stop_index=stop_index,
            )
        else:
            self._nav_cursor = self._advance_nav_cursor(
                rx, ry, poses, self._nav_cursor, reach_tol,
                stop_index=stop_index,
            )
        cursor_advanced = self._nav_cursor > prev_cursor
        if cursor_advanced:
            self.get_logger().info(
                f"Nav cursor advanced {prev_cursor} -> {self._nav_cursor} "
                f"(dropped {self._nav_cursor - prev_cursor} reached/overshot waypoint(s))."
            )
            self._recolor_nav_waypoints(self._nav_cursor)

        if precise_final and self._nav_cursor >= n - 1:
            approach = float(self.get_parameter("nav2_final_approach_m").value)
            final_dist = dist(n - 1)
            # On an access leg the goal sits off the road network, normally well
            # beyond the approach window: the handover happens the moment the
            # entry point is reached instead.
            if self._nav_entry_index is not None or final_dist <= approach:
                self._enter_final_phase(poses[n - 1], final_dist)
                return

        if self._nav_cursor >= n:
            # Cancel leftover Nav2 goal so /plan is not chased back to the end.
            self._cancel_nav2_goal()
            self._nav_running = False
            self._nav_goal_active = False
            self.get_logger().info("Batched route complete (all waypoints reached).")
            return

        lateral_required = self._lateral_closed_loop_required()
        lateral_pose = self._lateral_closed_loop_pose() if lateral_required else None
        if lateral_required and lateral_pose is None:
            # On a motor segment, closed-loop mode must fail closed. Falling
            # through to the biased OSM waypoint recreates the original hazard.
            blocked_signature = (
                self._lateral_state,
                self._lateral_reason,
                self._lateral_safety_reason,
            )
            if blocked_signature != self._lateral_dispatch_blocked_signature:
                target = self._lateral_target
                target_text = "no dynamic target"
                if target is not None:
                    age = (
                        self.get_clock().now().nanoseconds
                        - self._lateral_target_stamp_ns
                    ) / 1e9
                    target_text = (
                        f"target_forward={target.s - self._lateral_s:.2f} m, "
                        f"lateral_delta={target.d - self._lateral_robot_d:.2f} m, "
                        f"target_age={max(0.0, age):.2f} s"
                    )
                self.get_logger().warning(
                    "[campus_lateral] Nav2 dispatch BLOCKED (fail-closed): "
                    f"state={self._lateral_state}, reason={self._lateral_reason}, "
                    f"safety={self._lateral_safety_reason}, {target_text}; "
                    "cancelling the active Nav2 goal."
                )
                self._lateral_dispatch_blocked_signature = blocked_signature
            self._cancel_nav2_goal()
            return
        self._lateral_dispatch_blocked_signature = None
        if lateral_pose is not None:
            key = (
                self._nav_cursor,
                self._nav_cursor + 1,
                "semantic_lateral",
                self._lateral_target_seq,
            )
            if (
                key == self._nav_last_succeeded_key
                and self._nav_last_succeeded_route_s is not None
                and self._lateral_s
                <= self._nav_last_succeeded_route_s + 0.05
            ):
                self.get_logger().warning(
                    "[campus_lateral] Suppressing repeated Nav2 goal already "
                    "reported reached without route progress.",
                    throttle_duration_sec=2.0,
                )
                return
            if key == self._nav_sent_key and self._nav_goal_active:
                return
            if not self._send_nav2_waypoints([lateral_pose], preempt=True):
                return
            self._nav_sent_key = key
            self._nav_batch_last_xy = self._pose_xy(lateral_pose)
            self._nav_force_resend = False
            self.get_logger().info(
                f"Feeding Nav2 semantic right-side target at route "
                f"s={self._lateral_target.s:.1f} m."
            )
            return

        # Active Nav2 goal still starts at a waypoint we already dropped -> must
        # preempt immediately. Waiting for nav2_batch_advance_m here is what made
        # the robot turn around onto a stale /plan: the goal Nav2 holds still lists
        # waypoints this node has already retired (and painted green in RViz), and
        # ComputePathThroughPoses happily plans *backwards* to reach them.
        stale_active = self._nav_force_resend or (
            self._nav_goal_active
            and self._nav_sent_key is not None
            and self._nav_cursor > self._nav_sent_key[0]
        )

        # Keep driving the current batch until we near its end — unless the
        # batch tail was overshot or the active goal is already stale.
        advance = float(self.get_parameter("nav2_batch_advance_m").value)
        if (
            self._nav_goal_active
            and self._nav_batch_last_xy is not None
            and not stale_active
        ):
            lx, ly = self._nav_batch_last_xy
            dist_to_end = math.hypot(lx - rx, ly - ry)
            batch_end_passed = False
            if self._nav_sent_key is not None:
                batch_end_idx = self._nav_sent_key[1] - 1
                if 0 <= batch_end_idx < n:
                    batch_end_passed = self._waypoint_passed(
                        rx, ry, poses, batch_end_idx, reach_tol
                    )
            if dist_to_end > advance and not batch_end_passed:
                return

        # Build a contiguous batch from the cursor that stays within the radius.
        radius = float(self.get_parameter("nav2_batch_radius_m").value)
        end = self._nav_cursor
        while end < n and dist(end) <= radius:
            end += 1
        if end == self._nav_cursor:  # nearest ahead already beyond radius
            end = self._nav_cursor + 1

        key = (self._nav_cursor, end)
        # `not stale_active` matters for the forced case: the heading snap asks for
        # the *same* batch to be re-sent on the corrected frame, so the cursor has
        # not moved and the key is identical. Without this the force flag would be
        # swallowed here and never consumed.
        if key == self._nav_sent_key and self._nav_goal_active and not stale_active:
            return  # identical batch still executing

        batch = poses[self._nav_cursor:end]
        self.get_logger().info(
            f"Feeding Nav2 waypoints [{self._nav_cursor}:{end}] of {n} "
            f"(batch {len(batch)}{', preempt stale' if stale_active else ''})."
        )
        # Only record the batch as sent once it really went out. Booking it before
        # the send meant an early return inside _send_nav2_waypoints (action server
        # not up within 0.2 s, goals still gated on the heading lock) left
        # _nav_sent_key describing a goal Nav2 never received: stale_active then
        # read False on every later tick and the genuinely stale goal kept running
        # until the robot happened to near the *old* batch end.
        if not self._send_nav2_waypoints(batch, preempt=True):
            return
        self._nav_sent_key = key
        self._nav_batch_last_xy = self._pose_xy(poses[end - 1])
        self._nav_force_resend = False

    def _cancel_nav2_goal(self):
        # Invalidate both accepted goals and send requests whose response has
        # not arrived yet. Otherwise a rearward route could start Spin while a
        # stale NavigateThroughPoses request becomes accepted concurrently.
        self._nav_goal_seq += 1
        self._nav_send_pending = False
        handle = self._nav_goal_handle
        if handle is None:
            self._nav_goal_active = False
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 — best-effort preempt
            self.get_logger().warn(f"Failed to cancel Nav2 goal: {exc}")
        self._nav_goal_handle = None
        self._nav_goal_active = False

    # -- precise final approach ---------------------------------------------
    def _precise_final_enabled(self):
        return bool(
            self.get_parameter("nav2_final_precise_enable").value
            and self.final_action_client is not None
        )

    def _enter_final_phase(self, final_pose, distance):
        """Hand the last pose to NavigateToPose for a tight, single-goal approach."""
        self._nav_sent_key = None
        self._nav_batch_last_xy = None
        self._final_phase = True
        self._final_retry = 0
        self._final_wait_ticks = 0
        # Everything up to the hand-over pose is behind us now; only the goal
        # itself stays amber until it is reached.
        self._recolor_nav_waypoints(len(self._nav_route_poses) - 1)
        if self._nav_mode == "direct":
            what = "direct drive to the goal"
        elif self._nav_mode == "direct_via_entry":
            what = "second hop, from the entry to the goal"
        elif self._nav_entry_index is not None:
            what = "access leg from the entry point"
        else:
            what = "route end"
        self.get_logger().info(
            f"Final approach: {distance:.1f} m of {what}, switching from batched "
            f"waypoints to '{self.get_parameter('nav2_final_action_name').value}'."
        )
        # bt_navigator rejects a second navigator while the through-poses goal is
        # still running, and cancel_goal_async only *requests* the cancel. Wait
        # for its result callback (or the tick timeout) before sending.
        had_goal = self._nav_goal_handle is not None
        self._final_cancel_pending = had_goal
        self._cancel_nav2_goal()
        if not had_goal:
            self._send_final_goal(final_pose)

    def _final_phase_tick(self, rx, ry, distance):
        """Watch the final goal: finish on proximity, re-send after a failure."""
        reach_tol = float(self.get_parameter("nav2_final_reach_tol_m").value)
        if distance <= reach_tol:
            self._finish_route(f"final pose reached ({distance:.2f} m).")
            return
        if self._final_cancel_pending:
            # Normally cleared by the through-poses result callback; this guards
            # against a lost result leaving the final goal permanently unsent.
            self._final_wait_ticks += 1
            if self._final_wait_ticks < 3:
                return
            self.get_logger().warn(
                "Still waiting on the through-poses cancel; sending the final "
                "goal anyway."
            )
            self._final_cancel_pending = False
        if self._final_goal_active or self._nav_send_pending:
            return
        max_retry = int(self.get_parameter("nav2_final_max_retry").value)
        if self._final_retry > max_retry:
            self._finish_route(
                f"giving up on the final pose after {max_retry} retries; "
                f"stopped {distance:.2f} m short.",
                warn=True,
            )
            return
        self._send_final_goal(self._nav_route_poses[-1])

    def _goals_allowed(self):
        """Whether Nav2 may be driven right now.

        Locked is always fine. Unlocked is fine too unless the policy says to
        hold: the heading only locks after ~3 m of motion, so holding every goal
        until then means the robot never moves and never locks.
        """
        return self._heading_locked or not self._require_heading_lock

    def _warn_if_driving_unlocked(self, what):
        """Say out loud that this is going out on an unaligned map frame."""
        if self._heading_locked or not self._heading_status_seen:
            return
        self.get_logger().warn(
            f"{what} before the GPS heading locked: the map frame is not "
            f"north-aligned yet, so the robot may drive the wrong way until it "
            f"has moved ~3 m. If it circles without locking, drive it straight "
            f"by hand for ~5 m.",
            throttle_duration_sec=10.0,
        )

    def _send_final_goal(self, pose):
        if self.final_action_client is None:
            return
        if self._final_goal_active or self._nav_send_pending:
            return
        if not self._goals_allowed():
            self.get_logger().warn(
                "Holding the final campus goal: GPS heading not locked yet "
                "(/heading_align/status).",
                throttle_duration_sec=5.0,
            )
            return
        self._warn_if_driving_unlocked("Driving the final campus pose")
        action_name = self.get_parameter("nav2_final_action_name").value
        if not self.final_action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f"Nav2 action server '{action_name}' is not available; "
                "cannot run the precise final approach.",
                throttle_duration_sec=5.0,
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._poses_for_nav2([pose])[0]
        goal.behavior_tree = self.get_parameter("nav2_final_behavior_tree").value
        self._nav_send_pending = True
        self._final_goal_seq += 1
        seq = self._final_goal_seq
        future = self.final_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, s=seq: self._on_final_goal_response(f, s)
        )
        self.get_logger().info(
            f"Sent the final campus pose to Nav2 ({action_name}, "
            f"attempt {self._final_retry + 1})."
        )

    def _on_final_goal_response(self, future, seq):
        goal_handle = future.result()
        if seq != self._final_goal_seq:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._nav_send_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the final campus goal.")
            self._final_goal_active = False
            self._final_retry += 1
            return
        self._final_goal_handle = goal_handle
        self._final_goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=seq: self._on_final_result(f, s)
        )

    def _on_final_result(self, future, seq):
        if seq != self._final_goal_seq:
            return  # preempted / superseded; ignore
        wrapped = future.result()
        result = wrapped.result
        self._final_goal_active = False
        self._final_goal_handle = None
        # An aborted NavigateToPose leaves error_code at 0, so only the status
        # distinguishes a real arrival from bt_navigator dropping the goal.
        status = getattr(wrapped, "status", GoalStatus.STATUS_SUCCEEDED)
        error_code = getattr(result, "error_code", 0)
        error_msg = getattr(result, "error_msg", "")
        if status != GoalStatus.STATUS_SUCCEEDED or error_code:
            # The batch timer re-sends; the proximity check may also end the
            # route first if we are already close enough.
            self._final_retry += 1
            self.get_logger().warn(
                f"Final approach did not succeed (status {status}, "
                f"error {error_code}: {error_msg})"
            )
            return
        self._finish_route("Nav2 reported the final pose reached.")

    def _cancel_final_goal(self):
        self._final_goal_seq += 1
        self._nav_send_pending = False
        handle = self._final_goal_handle
        if handle is None:
            self._final_goal_active = False
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 — best-effort preempt
            self.get_logger().warn(f"Failed to cancel the final Nav2 goal: {exc}")
        self._final_goal_handle = None
        self._final_goal_active = False

    def _finish_route(self, reason, warn=False):
        self._cancel_final_goal()
        self._cancel_nav2_goal()
        self._cancel_route_spin()
        self._nav_running = False
        self._final_phase = False
        self._final_cancel_pending = False
        if not warn:
            self._recolor_nav_waypoints(len(self._nav_route_poses))
        message = f"Campus route complete: {reason}"
        if warn:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _poses_for_nav2(self, poses):
        """Copy poses for Nav2 with z flattened. Keep the original frame
        (campus_map); Nav2's global_frame=map transforms it, matching the
        visual /campus_path so the /plan endpoint stays on the waypoint.
        Note: build a fresh header — never alias p.header, or setting fields
        here would mutate the stored route poses."""
        out = []
        for p in poses:
            q = PoseStamped()
            q.header.frame_id = p.header.frame_id
            q.header.stamp = p.header.stamp
            q.pose.position.x = p.pose.position.x
            q.pose.position.y = p.pose.position.y
            q.pose.position.z = 0.0
            q.pose.orientation = p.pose.orientation
            out.append(q)
        return out

    def _on_heading_status(self, msg: String):
        # 'locked' or 'frozen_bad_gps' both mean the heading offset is valid and
        # the map frame is north-aligned; 'unlocked' means not yet aligned.
        self._heading_status_seen = True
        aligned = msg.data.strip() in ("locked", "frozen_bad_gps")
        if aligned and not self._heading_locked:
            self._heading_locked = True
            self.get_logger().info(
                "GPS heading locked; the map frame is north-aligned now."
            )
            # The map->odom TF is about to snap to north; wipe pre-snap
            # map-frame costmap data so it rebuilds correctly. Twice: see
            # _settle_clear_costmaps for why once is not enough.
            if self._clear_costmaps_on_lock:
                self._clear_costmaps(pass_label="immediate")
                settle = float(
                    self.get_parameter("clear_costmaps_settle_s").value
                )
                if settle > 0.0:
                    self._settle_clear_timer = self.create_timer(
                        settle, self._settle_clear_costmaps
                    )
            self._recover_heading_from_snap()

    def _recover_heading_from_snap(self):
        """Re-issue whatever Nav2 is driving, now that the TF is north-aligned.

        The stored route poses are in campus_map and their ENU coordinates never
        changed, so the route itself is still right. What is stale is Nav2's own
        plan: it was built through the pre-snap transform. Rather than replan
        here, drop the sent-batch key so the batch timer re-sends on its next
        tick (with preempt=True, cancelling the stale goal first) — the existing
        machinery, no new driving path.

        Only relevant when goals were allowed out before the lock; with the gate
        on there is nothing in flight to correct.
        """
        if not self._nav_running:
            return
        if self._final_phase:
            # The final approach owns the goal; cancel it and _final_phase_tick
            # re-sends on its next tick.
            self.get_logger().info(
                "heading snap: re-issuing the final campus pose on the "
                "north-aligned frame."
            )
            self._cancel_final_goal()
            return
        self.get_logger().info(
            "heading snap: re-issuing the current waypoint batch on the "
            "north-aligned frame."
        )
        # Must be the force flag, not _nav_sent_key = None. This is the one place
        # that asks for a re-send while a Nav2 goal is still active, and
        # stale_active requires _nav_sent_key is not None -- clearing it therefore
        # disabled the very re-send it was asking for, and the
        # nav2_batch_advance_m gate then held the pre-snap goal until the robot
        # neared the old batch end. Measured: 26 s of stale goal and three
        # waypoints retired (green in RViz) while Nav2 still had them as goals
        # (logs/outdoor_20260904_152821.log, 1788506941.87 -> 1788506966.73).
        self._nav_force_resend = True

    def _clear_costmaps(self, pass_label="immediate"):
        """Best-effort clear of the Nav2 global/local costmaps.

        Never blocks the caller: if a service is not up (e.g. Nav2 not running)
        we warn and move on. The result is reported from a done-callback rather
        than assumed, so a server-side failure is visible instead of being
        logged as a success.
        """
        if self._global_clear_cli is None:
            self._global_clear_cli = self.create_client(
                ClearEntireCostmap,
                self.get_parameter("global_costmap_clear_service").value,
            )
            self._local_clear_cli = self.create_client(
                ClearEntireCostmap,
                self.get_parameter("local_costmap_clear_service").value,
            )
        for name, cli in (
            ("global", self._global_clear_cli),
            ("local", self._local_clear_cli),
        ):
            if cli.service_is_ready() or cli.wait_for_service(timeout_sec=1.0):
                future = cli.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(
                    f"heading snap [{pass_label}]: requested {name} costmap clear."
                )
                future.add_done_callback(
                    lambda f, n=name, p=pass_label: self._on_clear_done(f, n, p)
                )
            else:
                self.get_logger().warn(
                    f"heading snap [{pass_label}]: {name} costmap clear service "
                    f"'{cli.srv_name}' unavailable; skipping."
                )

    def _on_clear_done(self, future, target, pass_label):
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001 — report, never propagate
            self.get_logger().error(
                f"heading snap [{pass_label}]: {target} clear FAILED: {exc}"
            )
            return
        self.get_logger().info(
            f"heading snap [{pass_label}]: {target} cleared."
        )

    def _settle_clear_costmaps(self):
        """Second clear, once the rotated map->odom TF has definitely landed.

        gps_heading_align publishes /heading_align/status before
        /heading_align/yaw_offset in the same tick, and map_odom_from_gps only
        broadcasts the rotated map->odom on its *next* timer tick after that. So
        the first clear provably runs before the snap, and whatever the costmaps
        write in the interval (global 10 Hz, local 5 Hz vs. a TF period of up to
        50 ms) is laid down under the old, unrotated transform and then displaced
        by it. TerrainLayer accumulates persistently, so a single such cycle
        leaves a lasting ghost. Clearing again after the dust settles removes it.
        """
        if self._settle_clear_timer is not None:
            self._settle_clear_timer.cancel()
            self._settle_clear_timer = None
        self._clear_costmaps(pass_label="post-snap")

    def _send_nav2_waypoints(self, poses, preempt=False):
        """Send a NavigateThroughPoses goal.

        Returns True only when the goal was actually handed to the action client,
        so the caller knows whether it may record the batch as sent.
        """
        if not poses:
            self.get_logger().warn("No nav waypoints to send.")
            return False
        if self.nav_action_client is None:
            return False
        if not self._goals_allowed():
            # Route is still computed/published for RViz; we just hold it back
            # from Nav2 until the map frame is north-aligned. The batch timer
            # keeps retrying, so it flushes automatically once locked.
            self.get_logger().warn(
                "Holding campus waypoints: GPS heading not locked yet "
                "(/heading_align/status). Robot would drive the wrong way.",
                throttle_duration_sec=5.0,
            )
            return False
        self._warn_if_driving_unlocked("Sending campus waypoints")
        if not self.nav_action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f"Nav2 action server '{self.get_parameter('nav2_action_name').value}' "
                "is not available; published /campus_nav_waypoints only."
            )
            return False
        if preempt:
            self._cancel_nav2_goal()
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._poses_for_nav2(poses)
        goal.behavior_tree = self.get_parameter("nav2_behavior_tree").value
        self._nav_send_pending = True
        self._nav_goal_seq += 1
        seq = self._nav_goal_seq
        future = self.nav_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, s=seq: self._on_nav2_goal_response(f, s)
        )
        self.get_logger().info(
            f"Sent {len(goal.poses)} campus waypoints to Nav2 "
            f"({self.get_parameter('nav2_action_name').value})."
        )
        return True

    def _on_nav2_goal_response(self, future, seq):
        goal_handle = future.result()
        if seq != self._nav_goal_seq:
            # The request was still in flight when a new route/preempt arrived.
            # It must be explicitly cancelled after acceptance; merely ignoring
            # its callback leaves Nav2 executing the stale route.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._nav_send_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected campus waypoint goal.")
            self._nav_goal_active = False
            self._nav_sent_key = None  # allow the timer to retry this batch
            self._nav_force_resend = True
            return
        self.get_logger().info("Nav2 accepted campus waypoint goal.")
        self._nav_goal_handle = goal_handle
        self._nav_goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=seq: self._on_nav2_result(f, s)
        )
        if self._final_phase:
            # The final approach started while this batch was still in flight;
            # it must be cancelled or bt_navigator will reject the final goal.
            self._final_cancel_pending = True
            self._cancel_nav2_goal()

    def _on_nav2_result(self, future, seq):
        if seq != self._nav_goal_seq:
            return  # preempted / superseded; ignore
        wrapped = future.result()
        result = wrapped.result
        self._nav_goal_active = False
        self._nav_goal_handle = None
        # The status matters as much as error_code: bt_navigator aborts leave
        # error_code at 0, so error_code alone reads an abort as success.
        status = getattr(wrapped, "status", GoalStatus.STATUS_SUCCEEDED)
        error_code = getattr(result, "error_code", 0)
        error_msg = getattr(result, "error_msg", "")
        if status == GoalStatus.STATUS_CANCELED:
            pass  # our own preempt (batch refresh or final approach handover)
        elif status != GoalStatus.STATUS_SUCCEEDED or error_code:
            self.get_logger().warn(
                f"Nav2 campus waypoint goal did not succeed (status {status}, "
                f"error {error_code}: {error_msg})"
            )
            # Let the timer re-issue the (possibly updated) batch from the cursor.
            self._nav_sent_key = None
            self._nav_force_resend = True
        else:
            self._nav_last_succeeded_key = self._nav_sent_key
            self._nav_last_succeeded_route_s = self._lateral_s
            self.get_logger().info("Nav2 campus waypoint batch finished.")
        # bt_navigator runs one navigator at a time, so the final NavigateToPose
        # can only go out once the through-poses goal has actually terminated.
        if self._final_phase and self._final_cancel_pending:
            self._final_cancel_pending = False
            self._send_final_goal(self._nav_route_poses[-1])

    def _plan(self, start_lonlat, goal_lonlat):
        """Compatibility helper for older manual calls."""
        self._plan_and_send_to_nav(start_lonlat, goal_lonlat)

    def destroy_node(self):
        """Stop the semantic worker before tearing down ROS entities."""
        self._cancel_route_spin()
        if self._lateral_worker is not None:
            self._lateral_worker.shutdown(wait=False, cancel_futures=True)
            self._lateral_worker = None
        return super().destroy_node()


def main():
    """Spin multi-threaded so TF delivery is not blocked by our own callbacks.

    tf2_ros fills a node's TF buffer from that node's own /tf and /tf_static
    subscription callbacks, and `Buffer.lookup_transform(..., timeout=T)` waits by
    busy-looping inside the calling callback:

        while now < start + timeout and not can_transform_core(...):
            sleep(0.02)          # tf2_ros/buffer.py, see geometry2#327

    On a single-threaded executor that loop holds the only thread, so /tf_static
    can never be delivered while it waits — if the buffer was not already warm the
    wait is doomed, and the caller then treats a live transform as missing. That is
    how a campus goal arriving early after an outdoor-stack restart got dropped:
    _on_campus_goal -> _current_robot_lonlat -> lookup_transform(campus_map,
    base_link, timeout=0.5) failed with "base_link does not exist" even though the
    static transform had been published continuously for 13 s.

    Two threads are enough, and the split is safe without touching any callback:
    TransformListener puts /tf and /tf_static in its own ReentrantCallbackGroup,
    while every callback in this node uses the default MutuallyExclusive group. So
    TF delivery runs concurrently with our logic, but _on_campus_goal,
    _nav_batch_tick, the parameter callback and the timers stay mutually exclusive
    exactly as before — no new races on the _nav_* state.
    """
    rclpy.init()
    node = RoutePlanner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
