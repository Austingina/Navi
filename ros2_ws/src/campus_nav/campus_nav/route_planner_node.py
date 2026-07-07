#!/usr/bin/env python3
"""ROS 2 (Humble) campus route planner.

Builds a routing graph from the OSM GeoPackage and plans an A* path in a local
ENU `campus_map` frame. Designed to run under the ROS system Python (rclpy +
numpy), with no extra pip dependencies.

This package does **not** publish any TF.  Campus geometry and paths use the
`campus_map` frame (fixed datum_lat/datum_lon).  Robot localization
(`map` -> `odom` -> `base_link`) is left to your GPS + FAST-LIO fusion stack.
To overlay the robot on the campus map in RViz, publish `map` -> `campus_map`
from that stack (identity if both frames share the same datum).

Interface
---------
Subscribes:
    /campus_goal_pose (geometry_msgs/PoseStamped) -> campus road-network goal
Publishes:
    /campus_path          (nav_msgs/Path) -> full OSM route preview
    /campus_nav_waypoints (nav_msgs/Path) -> sampled poses sent to Nav2
    /campus_nav_waypoint_markers (MarkerArray) -> numbered waypoint markers

All planar coordinates are ENU meters relative to the `datum_lat`/`datum_lon`
origin.

Run:
    source /opt/ros/jazzy/setup.bash   # or humble
    source /path/to/nav_test/env.sh    # sets NAV_TEST_ROOT + overlays ros_ws/install/
    cd ros_ws && colcon build --packages-select campus_nav
    ros2 launch campus_nav route_planner.launch.py

    # or directly:
    ros2 run campus_nav route_planner_node --ros-args -p pbf_path:=...
"""

import json
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateThroughPoses
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
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
    rasterize_occupancy,
    triangulate_fan,
)

# visualization_msgs is optional: only needed for the vector MarkerArray map.
# Install with: sudo apt-get install -y ros-humble-visualization-msgs
try:
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_MARKERS = True
except ImportError:
    HAVE_MARKERS = False


def _nav_test_root() -> str:
    root = os.environ.get("NAV_TEST_ROOT", "")
    if root:
        return root
    # Source tree: .../nav_test/ros_ws/src/campus_nav/campus_nav/route_planner_node.py
    return "/home/ai-sz-a26317-u1/nav_test"


def _default_pbf_path() -> str:
    return os.environ.get(
        "CAMPUS_PBF_PATH",
        os.path.join(_nav_test_root(), "xili_5km_merged.osm.pbf"),
    )


def _default_gpkg_overlay() -> str:
    return os.environ.get(
        "CAMPUS_GPKG_OVERLAY",
        os.path.join(_nav_test_root(), "campus_nav", "new_road.gpkg"),
    )


def _default_gpkg_path() -> str:
    return os.path.join(_nav_test_root(), "campus_nav.gpkg")


def _default_patrol_out_dir() -> str:
    d = os.path.join(_nav_test_root(), "campus_nav", "patrol_routes")
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

    def _patrol_waypoint_markers(enu_pts, frame_id, stamp):
        """Numbered spheres + labels for the ordered patrol waypoints.

        enu_pts: list of (east, north, label) in map meters.
        """
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
            sph.pose.position.z = 1.0
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 4.0
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
            txt.pose.position.z = 5.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 5.0
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
        self.declare_parameter("datum_lat", 22.5779177)
        self.declare_parameter("datum_lon", 113.9471564)
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
        self.declare_parameter("network_z", -1.0)          # lower road markers below robot layers
        self.declare_parameter("network_alpha", 0.92)
        # One-click campus routing: goal from RViz, start from current robot TF.
        self.declare_parameter("goal_navigation_enable", True)
        self.declare_parameter("goal_topic", "campus_goal_pose")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("nav_waypoints_topic", "campus_nav_waypoints")
        self.declare_parameter(
            "nav_waypoint_markers_topic", "campus_nav_waypoint_markers"
        )
        self.declare_parameter("nav_waypoint_spacing_m", 10.0)
        self.declare_parameter("send_to_nav2", True)
        self.declare_parameter("nav2_action_name", "navigate_through_poses")
        self.declare_parameter("nav2_behavior_tree", "")
        # Feed the route to Nav2 in rolling batches so every goal stays inside
        # the (rolling-window) global costmap. Disable to send the whole route.
        self.declare_parameter("nav2_batch_enable", True)
        # Max straight-line distance (m) from the robot for a waypoint to be
        # included in the current batch. Keep it < global_costmap half-size.
        self.declare_parameter("nav2_batch_radius_m", 25.0)
        # When the robot is within this distance (m) of the batch's last
        # waypoint, extend/send the next batch for seamless driving.
        self.declare_parameter("nav2_batch_advance_m", 8.0)
        # A waypoint within this distance (m) counts as reached (cursor moves on).
        self.declare_parameter("nav2_reach_tol_m", 3.0)
        # How often (s) to check progress and refresh the batch.
        self.declare_parameter("nav2_batch_period_s", 1.0)
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
            self.nav_wp_marker_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter("nav_waypoint_markers_topic").value,
                rviz_qos(),
            )
        self.nav_action_client = None
        # -- rolling waypoint batching state --
        self._nav_route_poses = []      # full sampled route (PoseStamped, frame_id)
        self._nav_cursor = 0            # index of next waypoint to still pursue
        self._nav_running = False       # a route is currently being executed
        self._nav_goal_active = False   # a Nav2 goal is accepted & executing
        self._nav_send_pending = False  # a goal was sent, awaiting accept/reject
        self._nav_goal_handle = None
        self._nav_batch_last_xy = None  # xy of last waypoint in the sent batch
        self._nav_sent_key = None       # (cursor, batch_end) of last sent batch
        self._nav_batch_timer = None
        if self.get_parameter("send_to_nav2").value:
            self.nav_action_client = ActionClient(
                self,
                NavigateThroughPoses,
                self.get_parameter("nav2_action_name").value,
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
                    "install ros-humble-visualization-msgs to see the vector map."
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
        self._publish_path_msg(self.patrol_path_pub, full, total, label="patrol")
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

    def _publish_path_msg(self, pub, lonlat_path, length, label="path"):
        msg = self._path_msg_from_lonlat(lonlat_path)
        pub.publish(msg)
        self.get_logger().info(
            f"Published {label}: {len(msg.poses)} poses, {length:.1f} m."
        )
        return msg

    def _path_msg_from_lonlat(self, lonlat_path):
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
        self.nav_wp_marker_pub.publish(
            MarkerArray(
                markers=_patrol_waypoint_markers(
                    enu_pts, self.frame_id, path_msg.header.stamp
                )
            )
        )

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

        d_lat = d.get("datum", {}).get("lat")
        d_lon = d.get("datum", {}).get("lon")
        if d_lat is not None and (
            abs(d_lat - self.enu.lat0) > 1e-9 or abs(d_lon - self.enu.lon0) > 1e-9
        ):
            self.get_logger().warn(
                f"Loaded patrol datum ({d_lat},{d_lon}) != node datum "
                f"({self.enu.lat0},{self.enu.lon0}); markers use the file's "
                "stored map_x/map_y, so set the node datum to match for alignment."
            )

        stamp = _latest_tf_stamp()
        # restore clicked waypoints so undo/save keep working after a load
        self.patrol_pts = [(w["lon"], w["lat"]) for w in d.get("waypoints", [])]

        # path (control points) -> /patrol_path, straight from stored map coords
        path_recs = d.get("path", [])
        if path_recs:
            msg = Path()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            for r in path_recs:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x = float(r["map_x"])
                ps.pose.position.y = float(r["map_y"])
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            self.patrol_path_pub.publish(msg)

        if HAVE_MARKERS:
            wp_enu = []
            for i, w in enumerate(d.get("waypoints", [])):
                label = "CHG" if i == 0 else str(i)
                wp_enu.append((w["map_x"], w["map_y"], label))
            if hasattr(self, "patrol_wp_pub"):
                self.patrol_wp_pub.publish(MarkerArray(
                    markers=_patrol_waypoint_markers(wp_enu, self.frame_id, stamp)
                ))
            corner_enu = [(c["map_x"], c["map_y"]) for c in d.get("corners", [])]
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
        result = self.graph.plan_points(
            start_lonlat[0], start_lonlat[1],
            goal_lonlat[0], goal_lonlat[1],
            respect_turns=self.respect_turns,
        )
        if result is None:
            self.get_logger().error("No path: start/goal not connected.")
            return
        lonlat_path, length, _sp, _gp = result
        self._publish_path_msg(self.path_pub, lonlat_path, length, label="campus path")

        step = self.get_parameter("nav_waypoint_spacing_m").value
        nav_lonlat = self._densify(lonlat_path, step)
        # The first point is the current robot position; Nav2 should drive to
        # upcoming waypoints, not re-goal itself at its current pose.
        if len(nav_lonlat) > 1:
            nav_lonlat = nav_lonlat[1:]
        nav_msg = self._publish_path_msg(
            self.nav_waypoints_pub, nav_lonlat, length, label="nav waypoints"
        )
        self._publish_nav_waypoint_markers(nav_msg)
        if self.get_parameter("send_to_nav2").value:
            if self.get_parameter("nav2_batch_enable").value:
                self._start_batched_route(nav_msg.poses)
            else:
                self._send_nav2_waypoints(nav_msg.poses)

    # -- rolling waypoint batching ------------------------------------------
    @staticmethod
    def _pose_xy(pose):
        return pose.pose.position.x, pose.pose.position.y

    def _robot_xy(self):
        robot_frame = self.get_parameter("robot_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, robot_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _start_batched_route(self, poses):
        """Store the full route and let the timer feed it to Nav2 in batches."""
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
        if self._nav_send_pending:
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

        # Advance the cursor to just past the nearest waypoint (monotonic).
        reach_tol = float(self.get_parameter("nav2_reach_tol_m").value)
        best_i, best_d = self._nav_cursor, float("inf")
        for i in range(self._nav_cursor, n):
            d = dist(i)
            if d < best_d:
                best_d, best_i = d, i
        new_cursor = best_i + 1 if best_d <= reach_tol else best_i
        self._nav_cursor = max(self._nav_cursor, new_cursor)

        if self._nav_cursor >= n:
            self._nav_running = False
            self._nav_goal_active = False
            self.get_logger().info("Batched route complete (all waypoints reached).")
            return

        # Keep driving the current batch until we near its end.
        advance = float(self.get_parameter("nav2_batch_advance_m").value)
        if self._nav_goal_active and self._nav_batch_last_xy is not None:
            lx, ly = self._nav_batch_last_xy
            if math.hypot(lx - rx, ly - ry) > advance:
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
            return  # identical batch still executing (batch already reaches the end)

        batch = poses[self._nav_cursor:end]
        self._nav_sent_key = key
        self._nav_batch_last_xy = self._pose_xy(poses[end - 1])
        self.get_logger().info(
            f"Feeding Nav2 waypoints [{self._nav_cursor}:{end}] of {n} "
            f"(batch {len(batch)})."
        )
        self._send_nav2_waypoints(batch)

    def _send_nav2_waypoints(self, poses):
        if not poses:
            self.get_logger().warn("No nav waypoints to send.")
            return
        if self.nav_action_client is None:
            return
        if not self.nav_action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f"Nav2 action server '{self.get_parameter('nav2_action_name').value}' "
                "is not available; published /campus_nav_waypoints only."
            )
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = self.get_parameter("nav2_behavior_tree").value
        self._nav_send_pending = True
        future = self.nav_action_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav2_goal_response)
        self.get_logger().info(
            f"Sent {len(poses)} campus waypoints to Nav2 "
            f"({self.get_parameter('nav2_action_name').value})."
        )

    def _on_nav2_goal_response(self, future):
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
        result_future.add_done_callback(self._on_nav2_result)

    def _on_nav2_result(self, future):
        result = future.result().result
        self._nav_goal_active = False
        error_code = getattr(result, "error_code", 0)
        error_msg = getattr(result, "error_msg", "")
        if error_code:
            self.get_logger().warn(
                f"Nav2 campus waypoint goal finished with error "
                f"{error_code}: {error_msg}"
            )
            # Let the timer re-issue the (possibly updated) batch from the cursor.
            self._nav_sent_key = None
        else:
            self.get_logger().info("Nav2 campus waypoint batch finished.")

    def _plan(self, start_lonlat, goal_lonlat):
        """Compatibility helper for older manual calls."""
        self._plan_and_send_to_nav(start_lonlat, goal_lonlat)


def main():
    rclpy.init()
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
