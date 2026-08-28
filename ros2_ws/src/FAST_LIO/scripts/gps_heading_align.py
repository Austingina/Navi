#!/usr/bin/env python3
"""GPS/LIO heading alignment for navsat_transform (north-aligned yaw).

Estimates the constant rotation yaw_offset between the LIO odom frame (whose
yaw=0 is the arbitrary boot heading) and ENU/north, then feeds navsat an
absolute-yaw odometry and map->odom publishers the rotation.

Method (rolling-baseline trajectory alignment, NOT course-vs-heading):
  The SAME physical motion appears as a displacement vector in both frames,
  related by the constant frame rotation R(theta):
        d_gps_ENU = R(theta) * d_lio_odom
  so theta = angle(d_gps) - angle(d_lio). Because we compare MOTION to MOTION
  (not GPS course to robot heading), this holds for ANY path shape -- curved,
  turning, even reversing -- and needs no straight driving.

  We keep ONE anchor and let the baseline (anchor->current chord) grow: a
  longer chord shrinks the angular error (~ gps_noise / baseline), so a short
  total path can still lock accurately. The chord relation is exact for any
  path, so growth is valid even while turning. We re-anchor only when GPS
  quality drops, when the path loops back (chord collapses vs its peak), or
  when |d_lio|/|d_gps| shows a GPS/LIO glitch. Lock once the baseline exceeds
  min_total_distance and the estimate stops moving.

Only GPS-quality-gated segments feed the estimate (urban-canyon safe). Once
locked, offset is frozen by default; absolute yaw continues as
lio_yaw + frozen_offset so relative heading still tracks LIO through bad GPS.

Publishes:
  /odometry/absolute_yaw  -- LIO pose/twist with absolute ENU yaw.
      NOTE: only published AFTER first lock. Withholding it before lock is the
      "gate" that stops navsat_transform from committing a wrong/zero datum
      before the heading is north-aligned (this build's wait_for_datum:true
      does NOT actually wait; it seeds (0,0,0)).
  /heading_align/status      -- unlocked | coarse | locked | frozen_bad_gps
  /heading_align/yaw_offset  -- absolute-heading correction [rad]. During a
      SLAM->outdoor handoff it is first seeded from the final SLAM pose (coarse),
      then replaced by the GPS/LIO displacement estimate on lock. map->odom
      publishers rotate odom(LIO startup frame) by this into ENU/north.

On first lock, calls navsat_transform SetDatum once with a FIXED site origin
(datum_lat/datum_lon, shared with campus_nav) so the map frame is anchored to
the same point the OSM overlay uses. This is required, not optional: if navsat
auto-datums instead, the map origin lands at the (arbitrary) lock position and
the campus network / goal lon-lat math are all offset.

Optional auto_drive: publish /cmd_vel forward until min_distance for first lock.
Optional exit_after_lock: shut down after first lock (heading_calib mode).
"""

from __future__ import annotations

import copy
import json
import math
import os
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, Twist
from geographic_msgs.msg import GeoPoint, GeoPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

try:
    from robot_localization.srv import SetDatum
except ImportError:  # pragma: no cover
    SetDatum = None  # type: ignore


_R_EARTH = 6378137.0  # WGS84 equatorial radius [m]


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    # ZYX yaw from quaternion (roll/pitch ~ 0 for ground robot odom).
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ll_to_local_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Equirectangular ENU meters relative to (lat0, lon0): (east, north)."""
    lat0r = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = dlon * math.cos(lat0r) * _R_EARTH
    north = dlat * _R_EARTH
    return east, north


class GpsHeadingAlign(Node):
    def __init__(self):
        super().__init__('gps_heading_align')

        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('absolute_yaw_topic', '/odometry/absolute_yaw')
        self.declare_parameter('status_topic', '/heading_align/status')
        # Absolute-heading correction (rad) so map->odom publishers can rotate
        # odom(LIO startup frame) into ENU/north. A fresh SLAM handoff may seed
        # it before GPS lock; GPS/LIO later replaces that coarse value.
        self.declare_parameter('yaw_offset_topic', '/heading_align/yaw_offset')
        self.declare_parameter(
            'slam_heading_handoff_file', '/tmp/navi_slam_heading_handoff.json')
        self.declare_parameter('slam_heading_handoff_max_age', 30.0)
        # navsat_transform advertises the relative service name "datum"; with the
        # node in the "/" namespace that resolves to "/datum" (NOT the private
        # "/navsat_transform/datum"). SetDatum pins the map origin to the fixed
        # site datum below (required for campus_nav alignment; auto-datum would
        # anchor at the arbitrary lock position instead).
        self.declare_parameter('datum_service', '/datum')

        # Fixed site origin for the map frame. MUST match campus_nav's
        # datum_lat/datum_lon, because route_planner draws the OSM graph and
        # converts goals in an ENU frame anchored at this exact point. If we let
        # navsat auto-datum at the (arbitrary) lock position instead, the map
        # origin drifts to wherever we happened to lock and the OSM overlay +
        # goal lon/lat math are all offset. SetDatum only moves the origin here;
        # orientation still comes from abs_yaw (map stays ENU/north).
        # Default MUST equal campus_nav/config/route_planner.yaml's datum.
        self.declare_parameter('datum_lat', 22.574357630323675)
        self.declare_parameter('datum_lon', 113.93507078035577)

        # min_distance is the minimum baseline (anchor->current chord) before the
        # chord has a trustworthy direction; below it we just keep growing.
        self.declare_parameter('min_distance', 1.5)
        # Baseline the rolling chord must exceed before a lock is allowed; the
        # heading error scales ~ gps_noise / baseline, so longer = more accurate.
        self.declare_parameter('min_total_distance', 3.0)
        # Lock once the rolling estimate moves less than this between updates.
        self.declare_parameter('lock_stable_tol_deg', 3.0)
        # Re-anchor if the path loops back and the chord collapses below this
        # fraction of the largest baseline reached (chord direction unreliable).
        self.declare_parameter('shrink_ratio', 0.6)
        # Loop-back re-anchors before we call out the circling case explicitly.
        # A couple are normal (a turn early on); a run of them is not.
        self.declare_parameter('loop_reanchor_warn_count', 3)
        # Drop+re-anchor if |d_lio|/|d_gps| leaves this band (GPS jump or LIO
        # slip -- the two must measure the same physical motion).
        self.declare_parameter('mag_ratio_tol', 2.0)
        self.declare_parameter('max_h_var', 25.0)          # m^2 (~5 m sigma)
        self.declare_parameter('min_fix_status', int(NavSatStatus.STATUS_FIX))
        self.declare_parameter('innovation_gate_deg', 25.0)
        self.declare_parameter('refine_after_lock', False)
        self.declare_parameter('refine_alpha', 0.2)
        # Refine runs on a much tighter budget than the lock, because it is
        # allowed to move a yaw_offset that everything downstream already
        # depends on. All three defaults come from replaying the 2026-08-26
        # Daxuecheng run (395 m); see _try_update_offset's refine branch.
        #
        # Chord length where the estimate is best. Below it GPS noise dominates
        # (sigma/L), above it LIO yaw drift does -- measured sd bottoms out at
        # 0.46-0.58 deg over 30-50 m and climbs back to 1.11 deg at 280 m.
        self.declare_parameter('refine_min_distance', 30.0)
        # Force a re-anchor past this, so no chord ever grows into the range
        # where LIO drift rules. Each anchor then yields one near-independent
        # estimate and refine_alpha averages them.
        self.declare_parameter('refine_max_distance', 50.0)
        # Solution quality for refine. Deliberately far stricter than
        # max_h_var: the lock has to be loose enough to happen at all, but a
        # PSRDIFF chord (sigma 2.5 m) over 30 m carries 6.7 deg of noise, so
        # feeding those in is worse than not refining. 0 => reuse max_h_var.
        self.declare_parameter('refine_max_h_var', 0.05)
        self.declare_parameter('call_set_datum', True)

        self.declare_parameter('auto_drive', False)
        self.declare_parameter('drive_speed', 0.3)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('exit_after_lock', False)
        self.declare_parameter('publish_rate', 20.0)

        self.fix_topic = self.get_parameter('fix_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.absolute_yaw_topic = self.get_parameter('absolute_yaw_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.yaw_offset_topic = self.get_parameter('yaw_offset_topic').value
        self.slam_heading_handoff_file = str(
            self.get_parameter('slam_heading_handoff_file').value)
        self.slam_heading_handoff_max_age = float(
            self.get_parameter('slam_heading_handoff_max_age').value)
        self.datum_service = self.get_parameter('datum_service').value
        self.datum_lat = float(self.get_parameter('datum_lat').value)
        self.datum_lon = float(self.get_parameter('datum_lon').value)

        self.min_distance = float(self.get_parameter('min_distance').value)
        self.min_total_distance = float(
            self.get_parameter('min_total_distance').value)
        self.lock_stable_tol = math.radians(
            float(self.get_parameter('lock_stable_tol_deg').value))
        self.shrink_ratio = float(self.get_parameter('shrink_ratio').value)
        self.loop_reanchor_warn_count = int(
            self.get_parameter('loop_reanchor_warn_count').value)
        self.mag_ratio_tol = float(self.get_parameter('mag_ratio_tol').value)
        self.max_h_var = float(self.get_parameter('max_h_var').value)
        self.min_fix_status = int(self.get_parameter('min_fix_status').value)
        self.innovation_gate = math.radians(
            float(self.get_parameter('innovation_gate_deg').value))
        self.refine_after_lock = bool(self.get_parameter('refine_after_lock').value)
        self.refine_alpha = float(self.get_parameter('refine_alpha').value)
        self.refine_min_dist = float(
            self.get_parameter('refine_min_distance').value)
        self.refine_max_dist = float(
            self.get_parameter('refine_max_distance').value)
        self.refine_max_h_var = float(
            self.get_parameter('refine_max_h_var').value) or self.max_h_var
        self.call_set_datum = bool(self.get_parameter('call_set_datum').value)

        self.auto_drive = bool(self.get_parameter('auto_drive').value)
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.exit_after_lock = bool(self.get_parameter('exit_after_lock').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State
        self.locked = False
        self.yaw_offset = 0.0
        self.datum_sent = False
        self._datum_attempts = 0
        self.last_status = 'unlocked'
        self.coarse_absolute_yaw: Optional[float] = self._consume_slam_heading_handoff()
        self.coarse_offset_active = False

        # anchor: (lat, lon, t, lio_x, lio_y) fixed while the baseline grows.
        self.anchor_fix: Optional[Tuple[float, float, float, float, float]] = None
        # Horizontal variance of the fix this anchor was set from. A chord is
        # only as good as its worse end, so refine gates on both.
        self._anchor_h_var = float('nan')
        self._max_baseline = 0.0            # largest chord reached this anchor [m]
        self._loop_reanchors = 0            # consecutive loop-back re-anchors
        self._last_theta: Optional[float] = None
        self.latest_fix: Optional[NavSatFix] = None
        self.latest_odom: Optional[Odometry] = None
        self.gps_quality_ok = False
        self._diag = 'waiting for GPS + odom'  # why still unlocked (for logs)
        # Horizontal variance of the last fix that passed the gate, so the lock
        # message can report the accuracy the offset was actually derived from.
        self._last_h_var = float('nan')

        self.abs_pub = self.create_publisher(Odometry, self.absolute_yaw_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.offset_pub = self.create_publisher(Float64, self.yaw_offset_topic, 10)
        self.cmd_pub = (
            self.create_publisher(Twist, self.cmd_vel_topic, 10)
            if self.auto_drive else None)

        self.create_subscription(
            NavSatFix, self.fix_topic, self.on_fix, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, 10)

        self.datum_client = None
        if SetDatum is not None and self.call_set_datum:
            self.datum_client = self.create_client(SetDatum, self.datum_service)

        self.create_timer(1.0 / max(publish_rate, 1.0), self.on_timer)

        self.get_logger().info(
            f'gps_heading_align: fix={self.fix_topic} odom={self.odom_topic} '
            f'min_dist={self.min_distance}m '
            f'max_h_var={self.max_h_var} refine={self.refine_after_lock} '
            f'auto_drive={self.auto_drive} exit_after_lock={self.exit_after_lock}')

    # ------------------------------------------------------------------ I/O
    def on_fix(self, msg: NavSatFix):
        self.latest_fix = msg
        reason = self._fix_quality_gate(msg)
        self.gps_quality_ok = reason is None
        if not self.gps_quality_ok:
            # Drop the anchor so no segment straddles the bad-GPS gap; the
            # accumulated rotation stays valid (theta is a constant), so we keep
            # it and simply resume gathering on the next good fix.
            self._diag = f'GPS quality gate failed ({reason}); anchor reset'
            self.anchor_fix = None
            return
        if self.anchor_fix is None:
            self._set_anchor(msg.latitude, msg.longitude, self._stamp_sec(msg))
            return
        self._try_update_offset(msg)

    def _set_anchor(self, lat: float, lon: float, t: float):
        # Every re-anchor starts a fresh rolling baseline.
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            lx, ly = float(p.x), float(p.y)
        else:
            lx = ly = float('nan')
        self.anchor_fix = (lat, lon, t, lx, ly)
        # Every caller gets here right after the fix cleared the quality gate,
        # so _last_h_var describes this very anchor.
        self._anchor_h_var = self._last_h_var
        self._max_baseline = 0.0
        self._last_theta = None

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg
        if self.coarse_absolute_yaw is not None and not self.coarse_offset_active:
            lio_yaw = quat_to_yaw(msg.pose.pose.orientation)
            self.yaw_offset = wrap_pi(self.coarse_absolute_yaw - lio_yaw)
            self.coarse_offset_active = True
            self._diag = 'SLAM yaw prior active; GPS/LIO is refining it'
            self.get_logger().info(
                f'COARSE yaw from SLAM handoff: absolute_enu_yaw='
                f'{math.degrees(self.coarse_absolute_yaw):.1f} deg, '
                f'new_lio_yaw={math.degrees(lio_yaw):.1f} deg, '
                f'yaw_offset={math.degrees(self.yaw_offset):.1f} deg')

    def on_timer(self):
        self._publish_absolute_yaw()
        self._publish_status()
        # Feed the frozen/refined offset to map->odom publishers so they can
        # rotate odom(LIO) into ENU. Continuous @publish_rate => a late joiner
        # (map_odom_from_gps) still gets it within one tick.
        if self.locked or self.coarse_offset_active:
            self.offset_pub.publish(Float64(data=float(self.yaw_offset)))
        if self.auto_drive and not self.locked and self.cmd_pub is not None:
            tw = Twist()
            tw.linear.x = self.drive_speed
            self.cmd_pub.publish(tw)

    def _consume_slam_heading_handoff(self) -> Optional[float]:
        """读取并删除一次性 SLAM→ENU yaw；超时/非法交接绝不使用。"""
        path = self.slam_heading_handoff_file
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            return None
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'ignore invalid SLAM heading handoff: {exc}')
            return None
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

        try:
            age = time.time() - float(data['created_at'])
            yaw = float(data['absolute_enu_yaw_rad'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('ignore malformed SLAM heading handoff')
            return None
        if age < -2.0 or age > self.slam_heading_handoff_max_age or not math.isfinite(yaw):
            self.get_logger().warn(
                f'ignore stale/invalid SLAM heading handoff (age={age:.1f}s)')
            return None
        self.get_logger().info(
            f'accepted one-shot SLAM heading prior (age={age:.1f}s, '
            f'ENU yaw={math.degrees(yaw):.1f} deg)')
        return wrap_pi(yaw)

    # --------------------------------------------------------------- quality
    def _fix_quality_gate(self, msg: NavSatFix) -> Optional[str]:
        """Say why this fix is unusable, or None when it is usable.

        The caller puts the reason straight into the status log. With max_h_var
        tightened to fixed-solution levels, "gate failed" on its own is not
        actionable -- the operator needs to see how far off the fix is to decide
        whether to wait or move the robot. h_var doubles as a solution-type
        indicator, which NavSatFix itself cannot carry (the driver merges float
        and fixed RTK into status=2): measured at Daxuecheng on 2026-08-26,
        NARROW_INT sits at <=0.01, NARROW_FLOAT near 0.33, PSRDIFF near 1.3.
        """
        if msg.status.status < self.min_fix_status:
            return f'status={msg.status.status} < {self.min_fix_status}'
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return 'lat/lon not finite'
        # Covariance type UNKNOWN (0) still often carries usable diagonals.
        cov = msg.position_covariance
        h_var = 0.5 * (float(cov[0]) + float(cov[4]))
        if h_var <= 0.0:
            # No usable variance. This used to substitute max_h_var as a
            # "conservative stand-in so max_h_var still applies if set low",
            # but the test below is ">", so the stand-in landed exactly on the
            # limit and never rejected anything. Harmless while max_h_var was 25;
            # not harmless now that it sits at fixed-solution level, because such
            # a fix would pass and then be logged as if it were centimetre-
            # accurate, which is precisely the judgement the lock message exists
            # to support. Refuse instead -- unicore_gnss fills the diagonals from
            # the reported lat/lon std for every solution type seen so far, so
            # this only fires on genuinely broken output.
            return 'no usable position_covariance reported'
        if h_var > self.max_h_var:
            return f'h_var={h_var:.4f} > {self.max_h_var} (sigma {h_var ** 0.5:.2f} m)'
        self._last_h_var = h_var
        return None

    def _stamp_sec(self, msg) -> float:
        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    # --------------------------------------------------------------- update
    def _try_update_offset(self, msg: NavSatFix):
        if self.latest_odom is None or self.anchor_fix is None:
            return
        if self.locked and not self.refine_after_lock:
            return

        lat0, lon0, t0, lx0, ly0 = self.anchor_fix
        t1 = self._stamp_sec(msg)
        dt = t1 - t0
        if dt < 0.5:
            return
        if not (math.isfinite(lx0) and math.isfinite(ly0)):
            # Anchor had no odom yet; re-anchor now that odom is available.
            self._set_anchor(msg.latitude, msg.longitude, t1)
            return

        # d_gps: ENU chord from the (fixed) anchor to now -- baseline grows.
        east, north = ll_to_local_m(lat0, lon0, msg.latitude, msg.longitude)
        dgps = math.hypot(east, north)
        if dgps > self._max_baseline:
            self._max_baseline = dgps
        if dgps < self.min_distance:
            self._diag = (
                f'baseline {dgps:.1f}/{self.min_distance:.1f} m; keep moving')
            return
        # Path looped back toward the anchor: the chord collapsed vs its peak, so
        # its direction is no longer reliable -> restart with a fresh baseline.
        if dgps < self.shrink_ratio * self._max_baseline:
            self._diag = (
                f'path looped (baseline {dgps:.1f} < '
                f'{self.shrink_ratio:.1f}x peak {self._max_baseline:.1f} m); '
                f're-anchor')
            self._loop_reanchors += 1
            # Repeated collapses are the signature of driving in circles, and
            # circles are what a Nav2 goal produces while the heading is still
            # unlocked: the reported heading is off by yaw_offset but GPS keeps
            # correcting the position, so the controller steers by a constant
            # error and closes an arc. That resets the baseline every lap, and
            # there is no timeout here — it can go on forever. Say so, with the
            # way out, instead of only logging one more 're-anchor'.
            if self._loop_reanchors >= self.loop_reanchor_warn_count:
                self.get_logger().warn(
                    f'heading NOT locking: the baseline has collapsed '
                    f'{self._loop_reanchors} times. The robot is most likely '
                    f'driving in circles on a wrong heading, which resets the '
                    f'baseline every lap. Drive it straight by hand for ~'
                    f'{self.min_total_distance + 2.0:.0f} m to break out of it.',
                    throttle_duration_sec=10.0,
                )
            self._set_anchor(msg.latitude, msg.longitude, t1)
            return
        # NOTE: no speed gate. Speed is irrelevant to a displacement-alignment
        # (a slow but real 3 m drive is a perfectly good baseline). Fake motion
        # from GPS jitter while stationary is instead caught by the mag-ratio
        # check below: |d_lio|~0 while |d_gps| large => rejected.

        # d_lio: LIO odom chord over the SAME anchor->now interval. Same physical
        # motion, so |d_lio| ~ |d_gps| and d_gps = R(theta) d_lio for ANY path.
        p = self.latest_odom.pose.pose.position
        dlx, dly = float(p.x) - lx0, float(p.y) - ly0
        dlio = math.hypot(dlx, dly)
        ratio = dlio / dgps if dgps > 1e-6 else 0.0
        if ratio < 1.0 / self.mag_ratio_tol or ratio > self.mag_ratio_tol:
            self._diag = (
                f'gps/lio move mismatch (|lio|={dlio:.1f} |gps|={dgps:.1f} m); '
                f're-anchor')
            self._set_anchor(msg.latitude, msg.longitude, t1)
            return

        # Rolling rotation estimate from the current chord pair.
        theta = wrap_pi(math.atan2(north, east) - math.atan2(dly, dlx))

        if self.locked:
            d = wrap_pi(theta - self.yaw_offset)
            if abs(d) > self.innovation_gate:
                return
            # A 5 m lock is geometrically limited to a few degrees no matter how
            # good the fix is: RTK-fixed positions carry ~0.2 m of real error,
            # and over a 5 m chord that is atan(0.2*sqrt2/5) = 3.2 deg. The
            # 2026-08-26 run locked at 72.46 deg against a measured truth of
            # 69.25 +- 0.62 deg -- off by 3.2 deg, exactly as predicted, which
            # then went straight into every downstream frame for 15 minutes.
            # Refine buys that back without making the lock wait for a long
            # straight drive, but only under conditions the lock cannot demand.
            if dgps < self.refine_min_dist:
                self._diag = (
                    f'refine: baseline {dgps:.1f}/{self.refine_min_dist:.0f} m')
                return
            # The chord is only as good as its worse end.
            worst = max(self._anchor_h_var, self._last_h_var)
            if not math.isfinite(worst) or worst > self.refine_max_h_var:
                self._diag = (
                    f'refine: h_var {worst:.3f} > {self.refine_max_h_var} '
                    f'(need a fixed solution at both chord ends)')
                return
            # First-order low-pass, which is what refine_alpha was always meant
            # to be -- it was declared but never used, and the old line below
            # assigned theta outright. Two reasons the smoothing matters:
            # updating yaw_offset rotates the whole map frame about the robot,
            # so a raw 0.5 deg step throws a point 30 m out by 26 cm and shakes
            # the costmaps; and LIO yaw drifts one way (~4.2 deg/km here), so a
            # plain running average would lag behind the truth while a low-pass
            # tracks it. alpha trades those off: too small and the lag through
            # the drift dominates, too large and per-chord noise (sd ~0.5 deg)
            # comes through. ~0.3 puts both near 0.3-0.7 deg.
            self.yaw_offset = wrap_pi(self.yaw_offset + self.refine_alpha * d)
            self.get_logger().info(
                f'refine yaw_offset={math.degrees(self.yaw_offset):.2f} deg '
                f'(chord {dgps:.1f} m, h_var {worst:.4f}, '
                f'moved {math.degrees(self.refine_alpha * d):+.2f} deg)',
                throttle_duration_sec=5.0)
            # Cap the chord so it never reaches the length where LIO yaw drift
            # beats GPS noise (sd measured 0.46 deg at 50 m, 1.11 deg at 280 m).
            # Re-anchoring here is what makes "the longer it drives, the better
            # it gets" true: each anchor contributes one near-independent
            # estimate, and the low-pass above averages them. Letting a single
            # chord grow instead would just trade GPS noise for LIO drift.
            if dgps > self.refine_max_dist:
                self._set_anchor(msg.latitude, msg.longitude, t1)
            return

        # Lock when the baseline is long enough AND the estimate has converged.
        move = (abs(wrap_pi(theta - self._last_theta))
                if self._last_theta is not None else math.pi)
        self._last_theta = theta
        if dgps < self.min_total_distance or move > self.lock_stable_tol:
            self._diag = (
                f'aligning: baseline {dgps:.1f}/{self.min_total_distance:.0f} m, '
                f'est={math.degrees(theta):.1f} deg (Δ{math.degrees(move):.1f} '
                f'deg); keep moving (any path)')
            return

        # First lock
        self.yaw_offset = theta
        self.locked = True
        self._loop_reanchors = 0
        # Report the fix accuracy the offset came from, and what it implies for
        # the offset itself: the chord angle error is roughly atan(sigma /
        # baseline), so a metre-level fix over a 5 m baseline is worth tens of
        # degrees. Without this the log cannot distinguish a 2 cm fixed-solution
        # lock from a float-solution one, and a wrong-but-stable offset is the
        # quietest failure this node has -- every downstream frame is rotated by
        # it and nothing else complains.
        #
        # Read the angle as a lower bound: sigma is what the receiver claims, and
        # it claims too much. Against RTK-fixed truth on 2026-08-26 the reported
        # sigma ran optimistic by ~3x (0.07 m claimed vs 0.22 m median residual),
        # and ~2x for the degraded modes, so the real error is a small multiple
        # of what is printed here.
        sigma = self._last_h_var ** 0.5
        self.get_logger().info(
            f'LOCKED yaw_offset={self.yaw_offset:.3f} rad '
            f'({math.degrees(self.yaw_offset):.1f} deg) '
            f'from {dgps:.1f} m baseline (any-shape path); '
            f'fix h_var={self._last_h_var:.4f} (sigma {sigma:.2f} m) '
            f'=> heading err ~{math.degrees(math.atan2(sigma, dgps)):.1f} deg')
        self._maybe_set_datum(msg)
        if self.auto_drive and self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())  # stop
        if self.exit_after_lock:
            self.get_logger().info('exit_after_lock: shutting down in 0.5s')
            self.create_timer(0.5, self._shutdown_once)

    def _shutdown_once(self):
        if getattr(self, '_shutting_down', False):
            return
        self._shutting_down = True
        if self.auto_drive and self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())
        rclpy.shutdown()

    # --------------------------------------------------------------- datum
    def _maybe_set_datum(self, fix: NavSatFix):
        if self.datum_sent or not self.call_set_datum:
            return
        if self.datum_client is None:
            self.get_logger().warn('SetDatum unavailable; skip datum call')
            self.datum_sent = True
            return
        if not self.datum_client.wait_for_service(timeout_sec=0.5):
            self._datum_attempts += 1
            if self._datum_attempts >= 5:
                # Give up retrying, but this is a real problem: without our
                # SetDatum, navsat auto-datums at the lock position and the OSM
                # overlay + goal lon/lat will be offset from the fixed site datum.
                self.get_logger().error(
                    f'datum service {self.datum_service} unavailable after '
                    f'{self._datum_attempts} tries; map origin will fall back to '
                    f'the lock position -> campus overlay/goals will be OFFSET')
                self.datum_sent = True
                return
            self.get_logger().warn(
                f'datum service {self.datum_service} not ready; will retry '
                f'({self._datum_attempts}/5)')
            return

        # Origin = FIXED site datum (not the current fix), so the map frame is
        # anchored to the same point campus_nav uses. navsat still projects the
        # robot's live GPS to its true ENU offset from this datum, so the robot
        # ends up at the right place; only the frame origin is pinned.
        #
        # datum YAW MUST BE 0 (map frame = ENU/north). It is the map frame's
        # orientation, NOT the robot's heading. We already feed navsat the
        # absolute ENU heading via /odometry/absolute_yaw (use_odometry_yaw:true),
        # so passing the robot heading here too double-counts it and rotates every
        # GPS position by ~abs_yaw about the datum (measured: /odometry/gps came
        # out = R(-abs_yaw)*true, i.e. ~130 m off). yaw=0 keeps GPS in true ENU.
        req = SetDatum.Request()
        req.geo_pose = GeoPose()
        req.geo_pose.position = GeoPoint(
            latitude=self.datum_lat, longitude=self.datum_lon, altitude=0.0)
        req.geo_pose.orientation = yaw_to_quat(0.0)

        future = self.datum_client.call_async(req)

        def _done(fut):
            try:
                fut.result()
                self.datum_sent = True
                self.get_logger().info(
                    f'SetDatum ok (fixed site origin, ENU yaw=0) '
                    f'lat={self.datum_lat:.7f} lon={self.datum_lon:.7f} '
                    f'(offset now={math.degrees(self.yaw_offset):.1f} deg)')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'SetDatum failed: {exc}')

        future.add_done_callback(_done)

    # --------------------------------------------------------------- publish
    def _publish_absolute_yaw(self):
        # Gate: do NOT feed navsat_transform any odometry/filtered until we are
        # locked. navsat needs both odom + gps to commit a datum; withholding
        # odom keeps it from auto-datuming (at a wrong/zero origin) before the
        # heading is north-aligned. Once locked we publish the corrected yaw and
        # explicitly SetDatum the fixed site origin (see _maybe_set_datum).
        if self.latest_odom is None or not self.locked:
            return
        odom = self.latest_odom
        out = Odometry()
        out.header = odom.header
        out.child_frame_id = odom.child_frame_id
        # Deep-copy pose so writing the absolute yaw below does NOT mutate the
        # cached self.latest_odom in place. Otherwise (timer @publish_rate faster
        # than LIO) the same latest_odom is re-published across ticks and each
        # tick re-reads its already-corrected orientation and adds yaw_offset
        # AGAIN, producing a yaw_offset-spaced staircase (lio+k*offset).
        out.pose = copy.deepcopy(odom.pose)
        out.twist = odom.twist
        abs_yaw = wrap_pi(quat_to_yaw(odom.pose.pose.orientation) + self.yaw_offset)
        out.pose.pose.orientation = yaw_to_quat(abs_yaw)
        self.abs_pub.publish(out)

        # Retry SetDatum if lock happened before service was up.
        if not self.datum_sent and self.latest_fix is not None:
            self._maybe_set_datum(self.latest_fix)

    def _publish_status(self):
        if not self.locked:
            status = 'coarse' if self.coarse_offset_active else 'unlocked'
        elif not self.gps_quality_ok:
            status = 'frozen_bad_gps'
        else:
            status = 'locked'

        prev = self.last_status
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

        # Console feedback so the operator understands why nothing north-aligned
        # yet (and downstream, why campus goals may be held). Log every state
        # transition immediately, plus a throttled heartbeat.
        transition = status != prev
        if status in ('unlocked', 'coarse'):
            gps = 'ok' if self.gps_quality_ok else 'bad/low-quality'
            self.get_logger().warn(
                f'heading {status.upper()} (gps={gps}): {self._diag}',
                throttle_duration_sec=5.0,
            )
        elif status == 'frozen_bad_gps':
            self.get_logger().warn(
                f'heading FROZEN (bad GPS): keeping locked yaw_offset='
                f'{math.degrees(self.yaw_offset):.1f} deg, tracking LIO.',
                throttle_duration_sec=5.0,
            )
        elif transition:  # locked
            self.get_logger().info(
                f'heading LOCKED and north-aligned '
                f'(yaw_offset={math.degrees(self.yaw_offset):.1f} deg).')


def main():
    rclpy.init()
    node = GpsHeadingAlign()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node.auto_drive and node.cmd_pub is not None:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
