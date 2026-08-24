import json
import math
import os
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
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
        out = prefix + self._densify(road_part, step_m) + suffix
        goal_entry_index = len(out) - 2 if suffix else None
        return out, goal_entry_index

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
        if i < len(poses) - 1:
            tangent = self._segment_tangent(poses[i], poses[i + 1])
        elif i > 0:
            tangent = self._segment_tangent(poses[i - 1], poses[i])
        else:
            return False
        if tangent is None:
            return False
        return (dx * tangent[0] + dy * tangent[1]) > passed_margin

    def _advance_nav_cursor(self, rx, ry, poses, cursor, reach_tol, stop_index=None):
        """Drop reached/overshot waypoints so Nav2 is not sent goals behind the robot.

        stop_index bounds the cursor: the precise final approach keeps the last
        pose out of reach of the fly-through tolerance so it is never dropped.
        """
        limit = len(poses) if stop_index is None else stop_index
        while cursor < limit and self._waypoint_passed(rx, ry, poses, cursor, reach_tol):
            cursor += 1
        return cursor

    def _robot_xy(self):
        robot_frame = self.get_parameter("robot_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, robot_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _start_batched_route(self, poses, entry_index=None, mode="road"):
        """Store the full route and let the timer feed it to Nav2 in batches."""
        self._cancel_nav2_goal()
        self._cancel_final_goal()
        self._final_phase = False
        self._final_retry = 0
        self._final_cancel_pending = False
        self._final_wait_ticks = 0
        self._nav_mode = mode
        self._nav_entry_index = entry_index
        self._nav_route_poses = list(poses)
        self._nav_cursor = 0
        self._nav_goal_active = False
        self._nav_send_pending = False
        self._nav_batch_last_xy = None
        self._nav_sent_key = None
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
        robot = self._robot_xy()
        if robot is None:
            return
        rx, ry = robot
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
        # With the precise approach the last pose belongs to the final phase, so
        # the fly-through tolerance must not consume it.
        stop_index = n - 1 if precise_final else n
        self._nav_cursor = self._advance_nav_cursor(
            rx, ry, poses, self._nav_cursor, reach_tol, stop_index=stop_index
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

        # Active Nav2 goal still starts at a waypoint we already dropped -> must
        # preempt immediately. Waiting for nav2_batch_advance_m here is what made
        # the robot turn around onto a stale /plan.
        stale_active = (
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
        if key == self._nav_sent_key and self._nav_goal_active:
            return  # identical batch still executing

        batch = poses[self._nav_cursor:end]
        self._nav_sent_key = key
        self._nav_batch_last_xy = self._pose_xy(poses[end - 1])
        self.get_logger().info(
            f"Feeding Nav2 waypoints [{self._nav_cursor}:{end}] of {n} "
            f"(batch {len(batch)}{', preempt stale' if stale_active else ''})."
        )
        self._send_nav2_waypoints(batch, preempt=True)

    def _cancel_nav2_goal(self):
        handle = self._nav_goal_handle
        if handle is None:
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
        if seq != self._final_goal_seq:
            return  # superseded by a newer send
        goal_handle = future.result()
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
        self._nav_sent_key = None

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
        if not poses:
            self.get_logger().warn("No nav waypoints to send.")
            return
        if self.nav_action_client is None:
            return
        if not self._goals_allowed():
            # Route is still computed/published for RViz; we just hold it back
            # from Nav2 until the map frame is north-aligned. The batch timer
            # keeps retrying, so it flushes automatically once locked.
            self.get_logger().warn(
                "Holding campus waypoints: GPS heading not locked yet "
                "(/heading_align/status). Robot would drive the wrong way.",
                throttle_duration_sec=5.0,
            )
            return
        self._warn_if_driving_unlocked("Sending campus waypoints")
        if not self.nav_action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f"Nav2 action server '{self.get_parameter('nav2_action_name').value}' "
                "is not available; published /campus_nav_waypoints only."
            )
            return
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

    def _on_nav2_goal_response(self, future, seq):
        if seq != self._nav_goal_seq:
            return  # superseded by a newer send
        goal_handle = future.result()
        self._nav_send_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected campus waypoint goal.")
            self._nav_goal_active = False
            self._nav_sent_key = None  # allow the timer to retry this batch
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
        else:
            self.get_logger().info("Nav2 campus waypoint batch finished.")
        # bt_navigator runs one navigator at a time, so the final NavigateToPose
        # can only go out once the through-poses goal has actually terminated.
        if self._final_phase and self._final_cancel_pending:
            self._final_cancel_pending = False
            self._send_final_goal(self._nav_route_poses[-1])

    def _plan(self, start_lonlat, goal_lonlat):
        """Compatibility helper for older manual calls."""
        self._plan_and_send_to_nav(start_lonlat, goal_lonlat)


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
