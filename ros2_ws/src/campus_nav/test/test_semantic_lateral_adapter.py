import math
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2, PointField

from campus_nav.route_planner_node import RoutePlanner
from campus_nav.route_graph import RouteGraph
from campus_nav.semantic_lateral_adapter import (
    LateralTarget,
    RoadBoundaryEstimator,
    RouteCorridor,
)


def _road_points(x_values, y_values):
    return np.array([(x, y) for x in x_values for y in y_values], dtype=float)


def test_route_right_is_independent_of_vehicle_heading():
    route = RouteCorridor([(0.0, 2.0), (20.0, 2.0)], [True])
    east = route.project(5.0, -3.0)
    assert math.isclose(east.d, 5.0, abs_tol=1e-6)

    reverse = RouteCorridor([(20.0, 2.0), (0.0, 2.0)], [True])
    west = reverse.project(5.0, 3.0)
    assert math.isclose(west.d, 1.0, abs_tol=1e-6)


def test_osm_topology_marks_only_vertices_with_three_road_arms_as_junctions():
    graph = RouteGraph()
    center = graph._node(120.0, 30.0)
    arms = [
        graph._node(120.001, 30.0),
        graph._node(119.999, 30.0),
        graph._node(120.0, 30.001),
    ]
    for arm in arms:
        graph._add_edge(center, arm, 10.0)

    assert graph.is_junction(120.0, 30.0)
    assert not graph.is_junction(120.001, 30.0)
    assert not graph.is_junction(120.0005, 30.0)


def test_smoothed_osm_tangent_keeps_fixed_offset_continuous_through_turn():
    route = RouteCorridor(
        [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)],
        [True, True], junction_s=[10.0], tangent_window=1.5,
    )

    offset_points = []
    for s in (9.9, 10.1):
        sample = route.sample(s)
        offset_points.append(np.array([
            sample.x + 2.0 * math.sin(sample.yaw),
            sample.y - 2.0 * math.cos(sample.yaw),
        ]))

    assert np.linalg.norm(offset_points[1] - offset_points[0]) < 0.6


def test_boundary_target_uses_measured_edge_not_osm_offset():
    # Physical road is y=[-3, 3], while the OSM reference is left-biased at y=2.
    route = RouteCorridor([(0.0, 2.0), (20.0, 2.0)], [True])
    points = _road_points(np.linspace(1.0, 8.0, 57), np.linspace(-3.0, 3.0, 81))
    estimator = RoadBoundaryEstimator(min_points=10, lookahead=4.0, inside_margin=0.75)

    target, reason = estimator.estimate(route, points, s_robot=0.0)

    assert reason == "ok"
    assert target is not None
    # The robust 95th-percentile edge adds 0.30 m conservatism to the configured
    # 0.75 m inset for this uniformly sampled synthetic road.
    assert math.isclose(target.y, -1.95, abs_tol=0.15)
    assert target.right >= 4.7
    assert target.left < -0.7


def test_non_motor_route_is_not_adapted():
    route = RouteCorridor([(0.0, 0.0), (20.0, 0.0)], [False])
    points = _road_points(np.linspace(1.0, 8.0, 40), np.linspace(-2.0, 2.0, 40))
    estimator = RoadBoundaryEstimator(min_points=10)

    target, reason = estimator.estimate(route, points, s_robot=0.0)

    assert target is None
    assert reason == "non_motor_segment"


def test_reverse_route_swaps_physical_right_side():
    route = RouteCorridor([(20.0, 0.0), (0.0, 0.0)], [True])
    points = _road_points(np.linspace(12.0, 19.0, 57), np.linspace(-3.0, 3.0, 81))
    estimator = RoadBoundaryEstimator(min_points=10, lookahead=4.0, inside_margin=0.75)

    target, reason = estimator.estimate(route, points, s_robot=0.0)

    assert reason == "ok"
    assert target is not None
    assert math.isclose(target.y, 1.95, abs_tol=0.15)


def test_osm_relative_fit_rejects_one_shifted_boundary_bin():
    route = RouteCorridor([(0.0, 0.0), (30.0, 0.0)], [True])
    strips = []
    for x in np.arange(1.1, 7.0, 0.25):
        # One complete longitudinal bin belongs to a spurious adjacent road.
        ys = np.linspace(1.0, 7.0, 81) if 4.0 <= x < 4.5 else np.linspace(-3.0, 3.0, 81)
        strips.extend((x, y) for y in ys)
    estimator = RoadBoundaryEstimator(min_points=10, temporal_alpha=1.0)

    target, reason = estimator.estimate(route, np.asarray(strips), s_robot=0.0)

    assert reason == "ok"
    assert target is not None
    assert abs(target.center) < 0.25
    assert 5.0 < target.width < 6.0


def test_censored_wide_road_seeks_confirmed_right_free_space():
    route = RouteCorridor([(0.0, 0.0), (30.0, 0.0)], [True])
    xs = np.linspace(1.0, 8.0, 57)
    ds = np.linspace(-1.0, 2.0, 61)
    points = np.array([(x, -d) for x in xs for d in ds], dtype=float)
    # In an eastbound route d=-y. The largest-d road samples touch the image
    # border, so 2 m is confirmed free space but not a measured physical edge.
    image_edge = np.array([(-y) > 1.8 for _x, y in points], dtype=bool)
    estimator = RoadBoundaryEstimator(
        min_points=10, temporal_alpha=1.0, inside_margin=0.75
    )

    target, reason = estimator.estimate(
        route, points, s_robot=0.0, image_edge=image_edge, robot_d=0.0
    )

    assert reason == "seek_right_edge"
    assert target is not None
    assert target.state_reason == "seek_right_edge"
    assert 0.0 < target.d < 1.5
    # A censored frontier must never become the remembered physical road model.
    assert estimator._center is None
    assert estimator._half_width is None


def test_visible_right_edge_does_not_require_visible_left_edge():
    route = RouteCorridor([(0.0, 0.0), (30.0, 0.0)], [True])
    xs = np.linspace(1.0, 8.0, 57)
    ds = np.linspace(-2.0, 3.0, 81)
    points = np.array([(x, -d) for x in xs for d in ds], dtype=float)
    # Only the physical left side is outside the camera FOV. Right remains a
    # measured road/non-road edge and is sufficient for the right-side target.
    image_edge = np.array([(-y) < -1.8 for _x, y in points], dtype=bool)
    estimator = RoadBoundaryEstimator(
        min_points=10, temporal_alpha=1.0, inside_margin=0.75
    )

    target, reason = estimator.estimate(
        route, points, s_robot=0.0, image_edge=image_edge, robot_d=0.0
    )

    assert reason == "right_edge_only"
    assert target is not None
    assert target.right > 2.6
    assert target.d > 1.8


def test_rearward_route_requests_spin_before_semantic_target():
    class PendingFuture:
        def __init__(self):
            self.callback = None

        def add_done_callback(self, callback):
            self.callback = callback

    class SpinClient:
        def __init__(self):
            self.goal = None

        @staticmethod
        def wait_for_server(timeout_sec):
            return timeout_sec > 0.0

        def send_goal_async(self, goal):
            self.goal = goal
            return PendingFuture()

    planner = RoutePlanner.__new__(RoutePlanner)
    planner._lateral_route = RouteCorridor(
        [(0.0, 0.0), (20.0, 0.0)], [True]
    )
    planner._lateral_s = 0.0
    planner._nav_mode = "road"
    planner._heading_status_seen = False
    planner._heading_locked = False
    planner._route_align_checked = False
    planner._route_align_required = False
    planner._route_align_complete = False
    planner._route_align_pending = False
    planner._route_align_active = False
    planner._route_align_goal_handle = None
    planner._route_align_seq = 0
    planner._route_align_retries = 0
    planner._route_align_error_rad = None
    planner._lateral_target = None
    planner._lateral_target_stamp_ns = None
    planner.route_spin_action_client = SpinClient()
    planner.get_logger = lambda: SimpleNamespace(
        info=lambda *_args, **_kwargs: None,
        warn=lambda *_args, **_kwargs: None,
    )
    values = {
        "semantic_lateral_align_heading_enable": True,
        "semantic_lateral_enabled": True,
        "semantic_lateral_shadow_mode": False,
        "semantic_lateral_align_enter_deg": 60.0,
        "semantic_lateral_align_exit_deg": 20.0,
        "semantic_lateral_align_max_retries": 1,
        "semantic_lateral_align_action_name": "spin",
        "semantic_lateral_align_timeout_s": 45.0,
    }
    planner.get_parameter = lambda name: SimpleNamespace(value=values[name])

    holding = planner._route_heading_alignment_tick(math.pi)

    assert holding
    assert planner._lateral_state == "ALIGN_ROUTE_HEADING"
    assert planner._route_align_pending
    assert math.isclose(
        abs(planner.route_spin_action_client.goal.target_yaw),
        math.pi,
        abs_tol=1e-6,
    )


def test_new_route_resets_completed_heading_alignment():
    planner = RoutePlanner.__new__(RoutePlanner)
    planner._route_align_checked = True
    planner._route_align_required = False
    planner._route_align_complete = True
    planner._route_align_pending = False
    planner._route_align_active = False
    planner._route_align_goal_handle = None
    planner._route_align_seq = 4
    planner._route_align_retries = 1
    planner._route_align_error_rad = math.pi
    planner._lateral_route_generation = 2
    planner._lateral_pending = {123: {"segmentation": object()}}
    planner._lateral_estimator = SimpleNamespace(reset=lambda: None)

    # The short route returns before graph/ENU access, but it must still reset
    # every piece of route-local alignment and semantic-frame state.
    planner._configure_lateral_route([], None, None, "road")

    assert not planner._route_align_checked
    assert not planner._route_align_required
    assert not planner._route_align_complete
    assert planner._route_align_retries == 0
    assert planner._route_align_error_rad is None
    assert planner._route_align_seq == 5
    assert planner._lateral_pending == {}


def test_lethal_right_target_shifts_inward_but_not_past_road_center():
    planner = RoutePlanner.__new__(RoutePlanner)
    planner._lateral_route = RouteCorridor(
        [(0.0, 0.0), (10.0, 0.0)], [True]
    )
    planner.get_parameter = lambda _name: SimpleNamespace(value=0.2)
    # For this eastbound route, right-positive d maps to negative world y.
    def target_is_safe(_x, y):
        safe = y >= -1.01
        return safe, "ok" if safe else "target_footprint_lethal"

    planner._lateral_target_is_safe = target_is_safe
    target = LateralTarget(
        x=4.0, y=-2.0, yaw=0.0, s=4.0, d=2.0,
        left=-2.0, right=2.75, confidence=1.0, bins=(),
        center=0.0, width=4.75,
    )

    adjusted, shifted = planner._shift_target_inward_for_safety(target)

    assert shifted
    assert math.isclose(adjusted.d, 1.0, abs_tol=1e-6)
    assert math.isclose(adjusted.y, -1.0, abs_tol=1e-6)
    assert adjusted.d >= adjusted.center
    assert adjusted.state_reason == "costmap_inward_shift"


def test_slew_limited_target_left_of_center_can_shift_toward_center():
    planner = RoutePlanner.__new__(RoutePlanner)
    planner._lateral_route = RouteCorridor(
        [(0.0, 0.0), (10.0, 0.0)], [True]
    )
    planner.get_parameter = lambda _name: SimpleNamespace(value=0.2)

    # The slew-limited point starts left of the fitted center.  Its nearest
    # safe footprint lies 0.4 m toward the center (positive route d).
    def target_is_safe(_x, y):
        safe = y <= -1.39
        return safe, "ok" if safe else "target_footprint_lethal"

    planner._lateral_target_is_safe = target_is_safe
    target = LateralTarget(
        x=4.0, y=-1.0, yaw=0.0, s=4.0, d=1.0,
        left=0.0, right=5.0, confidence=1.0, bins=(),
        center=2.5, width=5.0,
    )

    adjusted, shifted = planner._shift_target_inward_for_safety(target)

    assert shifted
    assert math.isclose(adjusted.d, 1.4, abs_tol=1e-6)
    assert math.isclose(adjusted.y, -1.4, abs_tol=1e-6)
    assert adjusted.d <= adjusted.center
    assert adjusted.state_reason == "costmap_inward_shift"


def test_superseded_nav_goal_response_is_explicitly_cancelled():
    class Handle:
        accepted = True

        def __init__(self):
            self.cancelled = False

        def cancel_goal_async(self):
            self.cancelled = True

    handle = Handle()
    future = SimpleNamespace(result=lambda: handle)
    planner = RoutePlanner.__new__(RoutePlanner)
    planner._nav_goal_seq = 8

    planner._on_nav2_goal_response(future, seq=7)

    assert handle.cancelled


def test_junction_freezes_prejunction_center_and_width_until_stable_release():
    route = RouteCorridor(
        [(0.0, 0.0), (30.0, 0.0)], [True], junction_s=[10.0]
    )
    estimator = RoadBoundaryEstimator(
        min_points=10, junction_guard_before=4.0,
        junction_guard_after=3.0, junction_release_frames=3,
    )
    wide = _road_points(np.linspace(3.0, 9.0, 49), np.linspace(-3.0, 3.0, 81))
    before, reason = estimator.estimate(route, wide, s_robot=2.0)
    assert reason == "ok"

    # The intersection mask may disappear or spread across every branch.  It is
    # ignored: the target follows a fixed offset around the selected OSM branch.
    guarded, reason = estimator.estimate(
        route, np.empty((0, 2), dtype=float), s_robot=6.5
    )
    assert reason == "junction_guard"
    assert guarded.junction_guard
    assert math.isclose(guarded.center, before.center, abs_tol=0.05)
    assert math.isclose(guarded.width, before.width, abs_tol=0.05)

    # Three consistent frames beyond the after-guard are required before the
    # estimator starts blending toward the new road width.
    narrow = _road_points(
        np.linspace(15.0, 21.0, 49), np.linspace(-2.0, 2.0, 81)
    )
    first, reason = estimator.estimate(route, narrow, s_robot=14.0)
    assert reason == "junction_release:1/3"
    assert first.junction_guard
    second, reason = estimator.estimate(route, narrow, s_robot=14.0)
    assert reason == "junction_release:2/3"
    assert second.junction_guard
    released, reason = estimator.estimate(route, narrow, s_robot=14.0)
    assert reason == "ok"
    assert not released.junction_guard
    assert released.width < before.width


def test_closed_loop_continues_driving_a_guarded_junction_target():
    planner = RoutePlanner.__new__(RoutePlanner)
    planner.frame_id = "campus_map"
    planner._lateral_state = "JUNCTION_GUARD"
    planner._lateral_s = 1.0
    planner._lateral_target_stamp_ns = 0
    planner._lateral_target = LateralTarget(
        4.0, -2.0, 0.0, 4.0, 2.0, -3.0, 3.0, 0.9, (),
        center=0.0, width=6.0, junction_guard=True,
        state_reason="junction_guard",
    )
    now = SimpleNamespace(nanoseconds=0, to_msg=lambda: Time())
    planner.get_clock = lambda: SimpleNamespace(now=lambda: now)
    planner.get_parameter = lambda _name: SimpleNamespace(value=1.0)
    planner._lateral_target_is_safe = lambda _x, _y: (True, "ok")

    pose = planner._lateral_closed_loop_pose()

    assert pose is not None
    assert pose.pose.position.x == 4.0
    assert pose.pose.position.y == -2.0


def _safety_planner(grid):
    planner = RoutePlanner.__new__(RoutePlanner)
    planner.frame_id = "campus_map"
    planner._lateral_costmap = grid
    values = {
        "semantic_lateral_costmap_max_age_s": 1.0,
        "semantic_lateral_safety_radius_m": 0.75,
        "semantic_lateral_lethal_occupancy": 99,
    }
    planner.get_parameter = lambda name: SimpleNamespace(value=values[name])
    return planner


def test_fused_costmap_footprint_gate_rejects_lethal_cell():
    grid = OccupancyGrid()
    grid.header.frame_id = "campus_map"
    grid.info.resolution = 0.5
    grid.info.width = grid.info.height = 20
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    planner = _safety_planner(grid)

    safe, reason = planner._lateral_target_is_safe(0.0, 0.0)
    assert safe
    assert reason == "ok"

    grid.data[10 * 20 + 10] = 100
    safe, reason = planner._lateral_target_is_safe(0.0, 0.0)
    assert not safe
    assert reason == "target_footprint_lethal"


def test_exact_organized_cloud_and_mask_decode_to_route_boundary():
    height, width = 48, 64
    xs = np.broadcast_to(np.linspace(1.0, 7.0, width), (height, width))
    ys = np.broadcast_to(
        np.linspace(-3.0, 3.0, height)[:, None], (height, width)
    )
    xyz = np.stack([xs, ys, np.zeros_like(xs)], axis=-1).astype(np.float32)
    segmentation = Image(
        height=height, width=width, encoding="mono8", step=width,
        data=np.full((height, width), 2, dtype=np.uint8).tobytes(),
    )
    confidence = Image(
        height=height, width=width, encoding="mono8", step=width,
        data=np.full((height, width), 255, dtype=np.uint8).tobytes(),
    )
    cloud = PointCloud2(
        height=height,
        width=width,
        fields=[
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ],
        point_step=12,
        row_step=12 * width,
        data=xyz.tobytes(),
    )
    planner = RoutePlanner.__new__(RoutePlanner)
    planner._lateral_estimator = RoadBoundaryEstimator(
        min_points=10, lookahead=4.0, inside_margin=0.75
    )
    route = RouteCorridor([(0.0, 2.0), (20.0, 2.0)], [True])

    generation, target, reason, road_xy, stamp_ns = (
        planner._process_lateral_frame(
            segmentation, confidence, cloud,
            np.eye(3), np.zeros(3), route, 0.0, 2, 7, 123, 1, 128,
        )
    )

    assert (generation, reason, stamp_ns) == (7, "ok", 123)
    assert road_xy.shape == (height * width, 2)
    assert target is not None
    assert target.y < -1.7


def test_debug_markers_include_directed_osm_reference():
    published = []
    planner = RoutePlanner.__new__(RoutePlanner)
    planner.frame_id = "campus_map"
    planner._lateral_route = RouteCorridor(
        [(1.0, 2.0), (4.0, 2.5), (8.0, 3.0)], [True, True]
    )
    planner._lateral_debug_pub = SimpleNamespace(publish=published.append)

    planner._publish_lateral_debug(None, np.empty((0, 2)), 123)

    assert len(published) == 1
    osm = next(
        marker for marker in published[0].markers
        if marker.ns == "osm_route_reference"
    )
    assert [(point.x, point.y) for point in osm.points] == [
        (1.0, 2.0), (4.0, 2.5), (8.0, 3.0)
    ]
