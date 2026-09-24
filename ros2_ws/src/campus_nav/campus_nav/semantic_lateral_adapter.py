"""Route-frame road-boundary estimation for semantic lateral navigation.

The ordered OSM route defines longitudinal progress and road right. Camera
pose is used only to transform observations into that route frame; camera or
robot yaw never defines road right.
"""

import math
import threading
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RouteSample:
    x: float
    y: float
    yaw: float
    s: float
    segment: int
    motor_road: bool


@dataclass(frozen=True)
class RouteProjection:
    s: float
    d: float
    distance: float
    segment: int


@dataclass(frozen=True)
class BoundaryBin:
    s: float
    left: float
    right: float
    count: int
    left_observed: bool = True
    right_observed: bool = True


@dataclass(frozen=True)
class LateralTarget:
    x: float
    y: float
    yaw: float
    s: float
    d: float
    left: float
    right: float
    confidence: float
    bins: tuple
    center: float = 0.0
    width: float = 0.0
    junction_guard: bool = False
    state_reason: str = "ok"


class RouteCorridor:
    """Directed polyline with arc-length and right-positive lateral coordinates."""

    def __init__(
        self, xy, motor_segments=None, junction_s=None, tangent_window=1.5
    ):
        points = np.asarray(xy, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            raise ValueError("route requires at least two XY points")
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        keep = np.r_[True, lengths > 1e-6]
        points = points[keep]
        if len(points) < 2:
            raise ValueError("route has no non-zero segment")
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        self.xy = points
        self.delta = delta
        self.lengths = lengths
        self.unit = delta / lengths[:, None]
        self.s0 = np.r_[0.0, np.cumsum(lengths)[:-1]]
        self.total_length = float(lengths.sum())
        self.vertex_s = np.r_[0.0, np.cumsum(lengths)]
        self.junction_s = tuple(sorted({
            float(np.clip(value, 0.0, self.total_length))
            for value in (junction_s or ())
        }))
        self.tangent_window = max(0.0, float(tangent_window))
        if motor_segments is None:
            self.motor = np.ones(len(lengths), dtype=bool)
        else:
            flags = np.asarray(motor_segments, dtype=bool)
            # Filtering duplicate vertices changes segment indexing. Route
            # construction should normally be duplicate-free; reject ambiguous
            # metadata instead of silently assigning the wrong road type.
            if len(flags) != len(lengths):
                raise ValueError("motor_segments must match non-zero route segments")
            self.motor = flags

    def junction_in_guard(self, s, before, after):
        """Return a stable guard id, merging overlapping junction zones.

        Two close junctions must behave as one intersection.  Returning the
        nearest junction makes the id switch halfway through an overlapping
        pair, which re-latches the corridor from an already contaminated
        intersection observation.  The first junction in each connected guard
        interval is therefore its stable identifier.
        """
        position = float(s)
        for start, end, guard_id in self.junction_guard_intervals(before, after):
            if start <= position <= end:
                return guard_id
        return None

    def junction_guard_intervals(self, before, after):
        """Return merged ``(start_s, end_s, guard_id)`` guard intervals."""
        if not self.junction_s:
            return ()
        before = max(0.0, float(before))
        after = max(0.0, float(after))
        intervals = []
        group_first = self.junction_s[0]
        group_last = group_first
        for junction in self.junction_s[1:]:
            if junction - before <= group_last + after:
                group_last = junction
                continue
            intervals.append((
                max(0.0, group_first - before),
                min(self.total_length, group_last + after),
                group_first,
            ))
            group_first = junction
            group_last = junction
        intervals.append((
            max(0.0, group_first - before),
            min(self.total_length, group_last + after),
            group_first,
        ))
        return tuple(intervals)

    def _position(self, s):
        s = float(np.clip(s, 0.0, self.total_length))
        i = int(np.searchsorted(self.s0 + self.lengths, s, side="right"))
        i = min(i, len(self.lengths) - 1)
        f = (s - self.s0[i]) / self.lengths[i]
        p = self.xy[i] + np.clip(f, 0.0, 1.0) * self.delta[i]
        return p, i

    def sample(self, s):
        s = float(np.clip(s, 0.0, self.total_length))
        p, i = self._position(s)
        tx, ty = self.unit[i]
        if self.tangent_window > 0.0:
            before, _ = self._position(s - self.tangent_window)
            after, _ = self._position(s + self.tangent_window)
            tangent = after - before
            norm = float(np.linalg.norm(tangent))
            if norm > 1e-6:
                tx, ty = tangent / norm
        return RouteSample(
            float(p[0]), float(p[1]), math.atan2(ty, tx), s, i,
            bool(self.motor[i]),
        )

    def project(self, x, y, s_hint=None, search_back=8.0, search_forward=20.0):
        p = np.array([x, y], dtype=np.float64)
        indices = np.arange(len(self.lengths))
        if s_hint is not None:
            lo = max(0.0, float(s_hint) - search_back)
            hi = min(self.total_length, float(s_hint) + search_forward)
            mask = (self.s0 + self.lengths >= lo) & (self.s0 <= hi)
            if np.any(mask):
                indices = indices[mask]
        a = self.xy[indices]
        delta = self.delta[indices]
        lengths = self.lengths[indices]
        frac = np.sum((p - a) * delta, axis=1) / (lengths * lengths)
        frac = np.clip(frac, 0.0, 1.0)
        closest = a + frac[:, None] * delta
        residual = p - closest
        dist2 = np.sum(residual * residual, axis=1)
        j = int(np.argmin(dist2))
        i = int(indices[j])
        tx, ty = self.unit[i]
        # Right normal of directed route tangent is (ty, -tx).
        d = residual[j, 0] * ty - residual[j, 1] * tx
        s = self.s0[i] + frac[j] * self.lengths[i]
        return RouteProjection(float(s), float(d), math.sqrt(float(dist2[j])), i)

    def project_points(
        self, points_xy, s_min, s_max, lateral_limit, return_indices=False
    ):
        """Project points onto route segments overlapping a local arc window."""
        points = np.asarray(points_xy, dtype=np.float64)
        segment_mask = (
            (self.s0 + self.lengths >= max(0.0, s_min - 2.0))
            & (self.s0 <= min(self.total_length, s_max + 2.0))
        )
        indices = np.flatnonzero(segment_mask)
        if not len(indices) or not len(points):
            empty = (
                np.empty(0), np.empty(0), np.empty(0, dtype=np.int32)
            )
            if return_indices:
                return (*empty, np.empty(0, dtype=np.int32))
            return empty

        best_dist2 = np.full(len(points), np.inf)
        best_s = np.zeros(len(points))
        best_d = np.zeros(len(points))
        best_i = np.zeros(len(points), dtype=np.int32)
        for i in indices:
            a = self.xy[i]
            delta = self.delta[i]
            length = self.lengths[i]
            frac = ((points - a) @ delta) / (length * length)
            frac = np.clip(frac, 0.0, 1.0)
            closest = a + frac[:, None] * delta
            residual = points - closest
            dist2 = np.sum(residual * residual, axis=1)
            update = dist2 < best_dist2
            tx, ty = self.unit[i]
            best_dist2[update] = dist2[update]
            best_s[update] = self.s0[i] + frac[update] * length
            best_d[update] = residual[update, 0] * ty - residual[update, 1] * tx
            best_i[update] = i
        valid = (
            (best_s >= s_min) & (best_s <= s_max)
            & (np.abs(best_d) <= lateral_limit)
        )
        result = best_s[valid], best_d[valid], best_i[valid]
        if return_indices:
            return (*result, np.flatnonzero(valid).astype(np.int32))
        return result


class RoadBoundaryEstimator:
    """Fit and track road bounds as offsets from the directed OSM route."""

    def __init__(
        self, near=1.0, far=7.0, bin_size=0.5, lateral_limit=12.0,
        min_points=20, width_min=2.5, width_max=15.0,
        lookahead=4.0, inside_margin=0.75, temporal_alpha=0.25,
        fit_max_slope=0.35, center_innovation=0.6, width_innovation=0.6,
        junction_guard_before=4.0, junction_guard_after=3.0,
        junction_release_frames=3,
    ):
        self.near = float(near)
        self.far = float(far)
        self.bin_size = float(bin_size)
        self.lateral_limit = float(lateral_limit)
        self.min_points = int(min_points)
        self.width_min = float(width_min)
        self.width_max = float(width_max)
        self.lookahead = float(lookahead)
        self.inside_margin = float(inside_margin)
        self.temporal_alpha = float(temporal_alpha)
        self.fit_max_slope = float(fit_max_slope)
        self.center_innovation = float(center_innovation)
        self.width_innovation = float(width_innovation)
        self.junction_guard_before = float(junction_guard_before)
        self.junction_guard_after = float(junction_guard_after)
        self.junction_release_frames = max(1, int(junction_release_frames))
        if not 0.0 < self.temporal_alpha <= 1.0:
            raise ValueError("temporal_alpha must be in (0, 1]")
        if min(
            self.fit_max_slope, self.center_innovation,
            self.width_innovation, self.junction_guard_before,
            self.junction_guard_after,
        ) < 0.0:
            raise ValueError(
                "fit, innovation and junction guard values must be nonnegative"
            )
        self._lock = threading.Lock()
        self._reset_locked()

    def _reset_locked(self):
        self._model_s = None
        self._center = None
        self._half_width = None
        self._center_slope = 0.0
        self._half_width_slope = 0.0
        self._model_confidence = 0.0
        self._guard_s = None
        self._guard_center = None
        self._guard_half_width = None
        self._guard_d = None
        self._guard_confidence = 0.0
        self._release_candidate = None
        self._release_good = 0

    def reset(self):
        """Forget the tracked corridor when the directed OSM route changes."""
        with self._lock:
            self._reset_locked()

    @staticmethod
    def _robust_line(s, values, counts, s_ref, max_slope):
        """Huber-IRLS line fit evaluated at s_ref, weighted by bin density."""
        x = np.asarray(s, dtype=np.float64) - float(s_ref)
        y = np.asarray(values, dtype=np.float64)
        base = np.sqrt(np.maximum(1.0, np.asarray(counts, dtype=np.float64)))
        if len(y) == 1 or np.ptp(x) < 1e-6:
            return float(np.median(y)), 0.0
        design = np.column_stack((np.ones_like(x), x))
        weights = base.copy()
        beta = np.array([np.median(y), 0.0], dtype=np.float64)
        for _ in range(5):
            root = np.sqrt(weights)[:, None]
            beta = np.linalg.lstsq(
                design * root, y * root[:, 0], rcond=None
            )[0]
            residual = y - design @ beta
            scale = 1.4826 * np.median(
                np.abs(residual - np.median(residual))
            )
            if scale <= 1e-5:
                break
            limit = 1.5 * scale
            huber = np.minimum(
                1.0, limit / np.maximum(np.abs(residual), 1e-9)
            )
            weights = base * huber
        slope = float(np.clip(beta[1], -max_slope, max_slope))
        intercept = float(np.average(y - slope * x, weights=weights))
        return intercept, slope

    def _target_from_model(
        self, route, target_s, center, half_width, confidence, bin_s,
        center_slope=0.0, half_width_slope=0.0, junction_guard=False,
        state_reason="ok", target_d_override=None,
    ):
        left = center - half_width
        right = center + half_width
        if right - left < 2.0 * self.inside_margin:
            return None
        target_d = (
            right - self.inside_margin
            if target_d_override is None else float(target_d_override)
        )
        route_target = route.sample(target_s)
        right_x = math.sin(route_target.yaw)
        right_y = -math.cos(route_target.yaw)
        fitted_bins = []
        for value in bin_s:
            ds = float(value) - target_s
            bin_center = center + center_slope * ds
            bin_half = max(
                self.inside_margin, half_width + half_width_slope * ds
            )
            fitted_bins.append(BoundaryBin(
                float(value), float(bin_center - bin_half),
                float(bin_center + bin_half), 0,
            ))
        return LateralTarget(
            route_target.x + target_d * right_x,
            route_target.y + target_d * right_y,
            route_target.yaw, target_s, target_d, left, right,
            float(confidence), tuple(fitted_bins), float(center),
            float(2.0 * half_width), bool(junction_guard), state_reason,
        )

    def _locked_target(self, route, target_s, s_min, s_max, reason):
        if self._guard_center is None or self._guard_half_width is None:
            return None
        bin_s = np.arange(
            s_min + 0.5 * self.bin_size, s_max, self.bin_size,
            dtype=np.float64,
        )
        return self._target_from_model(
            route, target_s, self._guard_center, self._guard_half_width,
            self._guard_confidence, bin_s, junction_guard=True,
            state_reason=reason, target_d_override=self._guard_d,
        )

    def estimate(
        self, route, road_points_xy, s_robot, image_edge=None, robot_d=None
    ):
        with self._lock:
            return self._estimate_locked(
                route, road_points_xy, s_robot, image_edge, robot_d
            )

    def _estimate_locked(
        self, route, road_points_xy, s_robot, image_edge=None, robot_d=None
    ):
        sample = route.sample(s_robot)
        if not sample.motor_road:
            return None, "non_motor_segment"
        s_min = min(route.total_length, s_robot + self.near)
        s_max = min(route.total_length, s_robot + self.far)
        if s_max - s_min < self.bin_size:
            return None, "route_window_too_short"
        target_s = min(s_robot + self.lookahead, route.total_length)

        # Freeze the last fitted corridor before entering a topological junction.
        # A constant offset follows the selected OSM branch around the turn while
        # the intersection's large connected road mask is deliberately ignored.
        junction = route.junction_in_guard(
            s_robot, self.junction_guard_before, self.junction_guard_after
        )
        if junction is not None:
            if self._guard_s != junction:
                if self._center is None:
                    return None, "junction_without_prior_model"
                ds = target_s - self._model_s
                self._guard_s = junction
                self._guard_center = self._center + self._center_slope * ds
                self._guard_half_width = max(
                    self.inside_margin,
                    self._half_width + self._half_width_slope * ds,
                )
                # Do not perform a lane change inside an intersection.  OSM
                # chooses the branch and heading, while the achieved vehicle
                # offset at guard entry is held through the complete junction.
                desired_d = (
                    self._guard_center + self._guard_half_width
                    - self.inside_margin
                )
                if robot_d is None:
                    self._guard_d = desired_d
                else:
                    left_limit = (
                        self._guard_center - self._guard_half_width
                        + self.inside_margin
                    )
                    right_limit = desired_d
                    self._guard_d = float(np.clip(
                        robot_d, left_limit, right_limit
                    ))
                self._guard_confidence = self._model_confidence
                self._release_candidate = None
                self._release_good = 0
            target = self._locked_target(
                route, target_s, s_min, s_max, "junction_guard"
            )
            return target, "junction_guard"

        road_points_xy = np.asarray(road_points_xy, dtype=np.float64)
        if image_edge is None:
            image_edge = np.zeros(len(road_points_xy), dtype=bool)
        else:
            image_edge = np.asarray(image_edge, dtype=bool)
            if len(image_edge) != len(road_points_xy):
                raise ValueError("image_edge must match road_points_xy")
        s, d, segments, point_indices = route.project_points(
            road_points_xy, s_min, s_max, self.lateral_limit,
            return_indices=True,
        )
        projected_edge = image_edge[point_indices]
        if len(s) < self.min_points:
            return self._missing_or_locked(
                route, target_s, s_min, s_max, "insufficient_road_points"
            )
        motor = route.motor[segments]
        s, d, projected_edge = s[motor], d[motor], projected_edge[motor]
        if len(s) < self.min_points:
            return self._missing_or_locked(
                route, target_s, s_min, s_max,
                "insufficient_motor_road_points",
            )

        raw_bins = []
        edges = np.arange(s_min, s_max + self.bin_size, self.bin_size)
        for lo, hi in zip(edges[:-1], edges[1:]):
            in_bin = (s >= lo) & (s < hi)
            values = d[in_bin]
            edge_flags = projected_edge[in_bin]
            if len(values) < self.min_points:
                continue
            median = np.median(values)
            mad = np.median(np.abs(values - median))
            if mad > 1e-6:
                keep = np.abs(values - median) <= 4.5 * mad
                values = values[keep]
                edge_flags = edge_flags[keep]
            if len(values) < self.min_points:
                continue
            left, right = np.quantile(values, [0.05, 0.95])
            width = right - left
            # A road point on the horizontal image border means that side is
            # only a visible free-space frontier, not a physical road edge.
            # Associate border points with the matching d extreme; this stays
            # valid even when camera yaw is not perfectly aligned.
            left_observed = not bool(np.any(
                edge_flags & (values <= np.quantile(values, 0.15))
            ))
            right_observed = not bool(np.any(
                edge_flags & (values >= np.quantile(values, 0.85))
            ))
            # A narrow *complete* span is not a road.  A narrow censored span is
            # still useful confirmed free space while seeking an unseen edge.
            width_valid = (
                width <= self.width_max
                and (
                    width >= self.width_min
                    or not (left_observed and right_observed)
                )
            )
            if width_valid:
                raw_bins.append(BoundaryBin(
                    float((lo + hi) * 0.5), float(left), float(right),
                    len(values), left_observed, right_observed,
                ))
        if not raw_bins:
            return self._missing_or_locked(
                route, target_s, s_min, s_max, "no_valid_boundary_bins"
            )

        bin_s = np.array([item.s for item in raw_bins], dtype=np.float64)

        # If the physical right edge is outside the camera FOV, never promote
        # the visible envelope to a measured boundary.  It is still confirmed
        # road, so use it as a conservative receding-horizon frontier.  The
        # caller's lateral-step limiter lets the robot reveal more of a wide
        # road progressively without corrupting the tracked width model.
        right_bins = [item for item in raw_bins if item.right_observed]
        if not right_bins:
            if self._guard_s is not None:
                target = self._locked_target(
                    route, target_s, s_min, s_max,
                    "junction_release_wait",
                )
                return target, "junction_release_wait"
            frontier_s = np.array([item.s for item in raw_bins], dtype=np.float64)
            frontier_values = np.array(
                [item.right for item in raw_bins], dtype=np.float64
            )
            frontier_counts = np.array(
                [item.count for item in raw_bins], dtype=np.float64
            )
            frontier, frontier_slope = self._robust_line(
                frontier_s, frontier_values, frontier_counts, target_s,
                self.fit_max_slope,
            )
            seek_d = frontier - self.inside_margin
            if self._center is None:
                half_width = max(self.width_min * 0.5, self.inside_margin)
                center = frontier - half_width
                center_slope = frontier_slope
                half_width_slope = 0.0
            else:
                ds = target_s - self._model_s
                center = self._center + self._center_slope * ds
                half_width = max(
                    self.inside_margin,
                    self._half_width + self._half_width_slope * ds,
                )
                center_slope = self._center_slope
                half_width_slope = self._half_width_slope
                # This is an ephemeral frontier corridor for visualization and
                # targeting only.  Do not write it into the tracked physical
                # center/width model below.
                center = frontier - half_width
                center_slope = frontier_slope - half_width_slope
            coverage = min(
                1.0,
                len(raw_bins) * self.bin_size / max(1.0, self.far - self.near),
            )
            density = min(
                1.0,
                sum(item.count for item in raw_bins) / (self.min_points * 8.0),
            )
            confidence = 0.5 * coverage + 0.5 * density
            target = self._target_from_model(
                route, target_s, center, half_width, confidence, bin_s,
                center_slope, half_width_slope,
                state_reason="seek_right_edge", target_d_override=seek_d,
            )
            return target, "seek_right_edge"

        # The right edge directly controls the driving target.  When the left
        # side is censored, keep the last width (or a conservative minimum-width
        # visualization) instead of pretending that the left FOV limit is a curb.
        right_s = np.array([item.s for item in right_bins], dtype=np.float64)
        right_values = np.array(
            [item.right for item in right_bins], dtype=np.float64
        )
        right_counts = np.array(
            [item.count for item in right_bins], dtype=np.float64
        )
        fitted_right, fitted_right_slope = self._robust_line(
            right_s, right_values, right_counts, target_s, self.fit_max_slope
        )
        full_bins = [
            item for item in right_bins if item.left_observed
        ]
        if not full_bins:
            if self._half_width is None:
                lower_bound = float(np.median([
                    item.right - item.left for item in right_bins
                ]))
                half_width = 0.5 * float(np.clip(
                    max(self.width_min, lower_bound),
                    self.width_min, self.width_max,
                ))
                half_width_slope = 0.0
            else:
                ds = target_s - self._model_s
                half_width = max(
                    self.inside_margin,
                    self._half_width + self._half_width_slope * ds,
                )
                half_width_slope = self._half_width_slope
            center = fitted_right - half_width
            center_slope = fitted_right_slope - half_width_slope
            fit_reason = "right_edge_only"
        else:
            full_s = np.array([item.s for item in full_bins], dtype=np.float64)
            full_counts = np.array(
                [item.count for item in full_bins], dtype=np.float64
            )
            fitted_left, fitted_left_slope = self._robust_line(
                full_s,
                np.array([item.left for item in full_bins], dtype=np.float64),
                full_counts, target_s, self.fit_max_slope,
            )
            center = 0.5 * (fitted_left + fitted_right)
            half_width = 0.5 * (fitted_right - fitted_left)
            center_slope = 0.5 * (
                fitted_left_slope + fitted_right_slope
            )
            half_width_slope = 0.5 * (
                fitted_right_slope - fitted_left_slope
            )
            fit_reason = "ok"

        half_width_slope = float(np.clip(
            half_width_slope,
            -self.fit_max_slope * 0.5,
            self.fit_max_slope * 0.5,
        ))
        half_width = float(np.clip(
            half_width, self.width_min * 0.5, self.width_max * 0.5
        ))
        coverage = min(
            1.0, len(raw_bins) * self.bin_size / max(1.0, self.far - self.near)
        )
        density = min(
            1.0, sum(item.count for item in raw_bins) / (self.min_points * 8.0)
        )
        confidence = 0.5 * coverage + 0.5 * density

        # Past the junction, do not unlock on the first possibly contaminated
        # frame.  Require a short run of self-consistent new-width observations.
        if self._guard_s is not None:
            candidate = (center, half_width)
            if self._release_candidate is None:
                self._release_good = 1
            else:
                delta = max(
                    abs(candidate[0] - self._release_candidate[0]),
                    abs(candidate[1] - self._release_candidate[1]),
                )
                self._release_good = self._release_good + 1 if delta <= 0.35 else 1
            self._release_candidate = candidate
            if self._release_good < self.junction_release_frames:
                reason = (
                    f"junction_release:{self._release_good}/"
                    f"{self.junction_release_frames}"
                )
                target = self._locked_target(
                    route, target_s, s_min, s_max, reason
                )
                return target, reason
            self._guard_s = None
            self._guard_center = None
            self._guard_half_width = None
            self._guard_d = None
            self._release_candidate = None
            self._release_good = 0

        # Temporal filtering happens in route-relative d, not map x/y.  Advancing
        # along a bend therefore does not look like lateral noise.
        if self._center is not None:
            ds = target_s - self._model_s
            predicted_center = self._center + self._center_slope * ds
            predicted_half = self._half_width + self._half_width_slope * ds
            center_delta = float(np.clip(
                center - predicted_center,
                -self.center_innovation, self.center_innovation,
            ))
            half_delta = float(np.clip(
                half_width - predicted_half,
                -0.5 * self.width_innovation, 0.5 * self.width_innovation,
            ))
            center = predicted_center + self.temporal_alpha * center_delta
            half_width = predicted_half + self.temporal_alpha * half_delta
            center_slope = (
                (1.0 - self.temporal_alpha) * self._center_slope
                + self.temporal_alpha * center_slope
            )
            half_width_slope = (
                (1.0 - self.temporal_alpha) * self._half_width_slope
                + self.temporal_alpha * half_width_slope
            )

        self._model_s = target_s
        self._center = float(center)
        self._half_width = float(half_width)
        self._center_slope = float(center_slope)
        self._half_width_slope = float(half_width_slope)
        self._model_confidence = float(confidence)
        target = self._target_from_model(
            route, target_s, self._center, self._half_width, confidence,
            bin_s, self._center_slope, self._half_width_slope,
            state_reason=fit_reason,
        )
        if target is None:
            return None, "road_too_narrow"
        return target, fit_reason

    def _missing_or_locked(self, route, target_s, s_min, s_max, reason):
        if self._guard_s is None:
            return None, reason
        target = self._locked_target(
            route, target_s, s_min, s_max, "junction_release_wait"
        )
        return target, "junction_release_wait"
