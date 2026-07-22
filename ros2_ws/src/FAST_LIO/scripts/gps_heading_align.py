#!/usr/bin/env python3
"""GPS course heading alignment for navsat_transform (north-aligned yaw).

Estimates yaw_offset = wrap(gps_course_enu - lio_yaw) only when GPS quality
gates pass (urban-canyon safe). Once locked, offset is frozen by default;
absolute yaw continues as lio_yaw + frozen_offset so relative heading still
tracks LIO through bad-GPS segments.

Publishes:
  /odometry/absolute_yaw  -- LIO pose/twist with absolute ENU yaw
  /heading_align/status   -- unlocked | locking | locked | frozen_bad_gps

On first lock, calls navsat_transform SetDatum once (wait_for_datum: true).

Optional auto_drive: publish /cmd_vel forward until min_distance for first lock.
Optional exit_after_lock: shut down after first lock (heading_calib mode).
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, Twist
from geographic_msgs.msg import GeoPoint, GeoPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

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
        self.declare_parameter('datum_service', '/navsat_transform/datum')

        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('min_distance', 5.0)
        self.declare_parameter('max_h_var', 25.0)          # m^2 (~5 m sigma)
        self.declare_parameter('min_fix_status', int(NavSatStatus.STATUS_FIX))
        self.declare_parameter('max_yaw_rate', 0.25)       # rad/s; reject turns
        self.declare_parameter('innovation_gate_deg', 25.0)
        self.declare_parameter('refine_after_lock', False)
        self.declare_parameter('refine_alpha', 0.2)
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
        self.datum_service = self.get_parameter('datum_service').value

        self.min_speed = float(self.get_parameter('min_speed').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.max_h_var = float(self.get_parameter('max_h_var').value)
        self.min_fix_status = int(self.get_parameter('min_fix_status').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.innovation_gate = math.radians(
            float(self.get_parameter('innovation_gate_deg').value))
        self.refine_after_lock = bool(self.get_parameter('refine_after_lock').value)
        self.refine_alpha = float(self.get_parameter('refine_alpha').value)
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
        self.last_status = 'unlocked'

        self.anchor_fix: Optional[Tuple[float, float, float]] = None  # lat, lon, t
        self.latest_fix: Optional[NavSatFix] = None
        self.latest_odom: Optional[Odometry] = None
        self.gps_quality_ok = False

        self.abs_pub = self.create_publisher(Odometry, self.absolute_yaw_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
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
            f'min_dist={self.min_distance}m min_speed={self.min_speed} '
            f'max_h_var={self.max_h_var} refine={self.refine_after_lock} '
            f'auto_drive={self.auto_drive} exit_after_lock={self.exit_after_lock}')

    # ------------------------------------------------------------------ I/O
    def on_fix(self, msg: NavSatFix):
        self.latest_fix = msg
        self.gps_quality_ok = self._fix_quality_ok(msg)
        if not self.gps_quality_ok:
            self.anchor_fix = None
            return
        if self.anchor_fix is None:
            self.anchor_fix = (msg.latitude, msg.longitude, self._stamp_sec(msg))
            return
        self._try_update_offset(msg)

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def on_timer(self):
        self._publish_absolute_yaw()
        self._publish_status()
        if self.auto_drive and not self.locked and self.cmd_pub is not None:
            tw = Twist()
            tw.linear.x = self.drive_speed
            self.cmd_pub.publish(tw)

    # --------------------------------------------------------------- quality
    def _fix_quality_ok(self, msg: NavSatFix) -> bool:
        if msg.status.status < self.min_fix_status:
            return False
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return False
        # Covariance type UNKNOWN (0) still often carries usable diagonals.
        cov = msg.position_covariance
        h_var = 0.5 * (float(cov[0]) + float(cov[4]))
        if h_var <= 0.0:
            # Unknown variance: allow but do not treat as high quality; use
            # a conservative stand-in so max_h_var still applies if set low.
            h_var = self.max_h_var
        if h_var > self.max_h_var:
            return False
        return True

    def _stamp_sec(self, msg) -> float:
        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    # --------------------------------------------------------------- update
    def _try_update_offset(self, msg: NavSatFix):
        if self.latest_odom is None or self.anchor_fix is None:
            return
        if self.locked and not self.refine_after_lock:
            return

        lat0, lon0, t0 = self.anchor_fix
        t1 = self._stamp_sec(msg)
        dt = t1 - t0
        if dt < 0.5:
            return

        east, north = ll_to_local_m(lat0, lon0, msg.latitude, msg.longitude)
        dist = math.hypot(east, north)
        if dist < self.min_distance:
            return
        speed = dist / dt
        if speed < self.min_speed:
            # Reset anchor so a long slow crawl does not eventually pass.
            self.anchor_fix = (msg.latitude, msg.longitude, t1)
            return

        odom = self.latest_odom
        yaw_rate = abs(float(odom.twist.twist.angular.z))
        if yaw_rate > self.max_yaw_rate:
            self.anchor_fix = (msg.latitude, msg.longitude, t1)
            return

        course_yaw = math.atan2(north, east)  # ENU yaw: 0=east, +CCW (REP-103)

        lio_yaw = quat_to_yaw(odom.pose.pose.orientation)
        meas_offset = wrap_pi(course_yaw - lio_yaw)

        if self.locked:
            innov = abs(wrap_pi(meas_offset - self.yaw_offset))
            if innov > self.innovation_gate:
                self.anchor_fix = (msg.latitude, msg.longitude, t1)
                return
            a = self.refine_alpha
            self.yaw_offset = wrap_pi(
                (1.0 - a) * self.yaw_offset + a * meas_offset)
            self.anchor_fix = (msg.latitude, msg.longitude, t1)
            self.get_logger().info(
                f'refine yaw_offset={self.yaw_offset:.3f} rad '
                f'({math.degrees(self.yaw_offset):.1f} deg)')
            return

        # First lock
        self.yaw_offset = meas_offset
        self.locked = True
        self.anchor_fix = (msg.latitude, msg.longitude, t1)
        self.get_logger().info(
            f'LOCKED yaw_offset={self.yaw_offset:.3f} rad '
            f'({math.degrees(self.yaw_offset):.1f} deg) '
            f'dist={dist:.1f}m speed={speed:.2f}m/s')
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
        if not self.datum_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'datum service {self.datum_service} not ready; will retry')
            return

        abs_yaw = wrap_pi(
            quat_to_yaw(self.latest_odom.pose.pose.orientation) + self.yaw_offset)
        req = SetDatum.Request()
        req.geo_pose = GeoPose()
        req.geo_pose.position = GeoPoint(
            latitude=fix.latitude, longitude=fix.longitude, altitude=0.0)
        req.geo_pose.orientation = yaw_to_quat(abs_yaw)

        future = self.datum_client.call_async(req)

        def _done(fut):
            try:
                fut.result()
                self.datum_sent = True
                self.get_logger().info(
                    f'SetDatum ok lat={fix.latitude:.7f} lon={fix.longitude:.7f} '
                    f'yaw={abs_yaw:.3f}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'SetDatum failed: {exc}')

        future.add_done_callback(_done)

    # --------------------------------------------------------------- publish
    def _publish_absolute_yaw(self):
        if self.latest_odom is None:
            return
        # Before lock still publish LIO yaw so navsat can receive odom, but
        # wait_for_datum keeps it from committing a wrong origin.
        odom = self.latest_odom
        out = Odometry()
        out.header = odom.header
        out.child_frame_id = odom.child_frame_id
        out.pose = odom.pose
        out.twist = odom.twist
        if self.locked:
            abs_yaw = wrap_pi(quat_to_yaw(odom.pose.pose.orientation) + self.yaw_offset)
            out.pose.pose.orientation = yaw_to_quat(abs_yaw)
        self.abs_pub.publish(out)

        # Retry SetDatum if lock happened before service was up.
        if self.locked and not self.datum_sent and self.latest_fix is not None:
            self._maybe_set_datum(self.latest_fix)

    def _publish_status(self):
        if not self.locked:
            status = 'unlocked'
        elif not self.gps_quality_ok:
            status = 'frozen_bad_gps'
        else:
            status = 'locked'
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


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
