#!/usr/bin/env python3
"""Publish map->odom by fusing GPS + LIO position, with an absolute-yaw rotation.

Rationale
---------
LIO gives accurate short-term motion but its yaw is relative to an *arbitrary
startup orientation*, not true north. GPS gives absolute position (and, via
navsat, ENU/north). We fuse position in a small KF, and rotate the whole
map->odom by the absolute-heading correction (yaw_offset) so the map frame is
ENU/north — otherwise the LIO cloud stays rotated by yaw_offset and never lines
up with the geo-referenced base map.

yaw_offset source
-----------------
gps_heading_align estimates yaw_offset = wrap(gps_course_enu - lio_yaw) and,
once locked, publishes it on /heading_align/yaw_offset. Before any offset is
received (or in setups without heading align, e.g. the sim EKF path where this
node is not even started) yaw_offset stays 0 and map->odom is identity-rotation,
i.e. the previous behavior — so nothing else changes.

Filter
------
State: robot position in the map(ENU) frame  p = [px, py]  (two scalar KFs)
    Predict: p += R(yaw_offset) * (LIO position delta)   # rotate odom delta -> map/ENU
             P  += q * dt                                 # process noise (LIO drift)
    Update : z = GPS position (ENU, from navsat), R = GPS covariance
             standard KF update, with a Mahalanobis gate to reject GPS spikes

Output: map->odom rotation = R(yaw_offset);
        map->odom translation = p_filtered - R(yaw_offset) * LIO_position.

Inputs:
    /odometry/gps (nav_msgs/Odometry)      -- robot position in map (from navsat)
    /heading_align/yaw_offset (std_msgs/Float64) -- ENU heading correction [rad]
    TF odom->base_link                     -- from FAST-LIO chain
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import tf2_ros


class MapOdomFromGps(Node):
    def __init__(self):
        super().__init__('map_odom_from_gps')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('gps_topic', '/odometry/gps')
        # Absolute-heading correction (rad) from gps_heading_align. map->odom is
        # rotated by this so odom(LIO) is expressed in ENU/north. 0 => identity
        # rotation (previous behavior / no heading-align running).
        self.declare_parameter('yaw_offset_topic', '/heading_align/yaw_offset')
        self.declare_parameter('publish_rate', 20.0)
        # Process noise: how fast LIO position drifts (m^2 per second, per axis).
        self.declare_parameter('process_noise', 0.05)
        # Fallback GPS variance (m^2) when the message covariance is 0/unknown.
        self.declare_parameter('gps_variance_default', 1.0)
        # Use the covariance carried in the GPS Odometry message for R.
        self.declare_parameter('use_msg_covariance', True)
        # Reject a GPS update if innovation exceeds this many sigmas (0 disables).
        self.declare_parameter('gps_gate_sigma', 5.0)
        # Initial state covariance (large => first GPS snaps the estimate in).
        self.declare_parameter('initial_variance', 10.0)
        self.declare_parameter('publish_identity_until_gps', True)
        # Fixed campus datum: same values used by gps_heading_align and campus_nav.
        # map_odom_from_gps subscribes to /gps/fix and converts lat/lon -> ENU
        # directly so GPS position is available immediately, before navsat can
        # produce /odometry/gps (which requires heading lock first).
        self.declare_parameter('datum_lat', 22.574357630323675)
        self.declare_parameter('datum_lon', 113.93507078035577)
        self.declare_parameter('raw_gps_topic', '/gps/fix')

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        gps_topic = self.get_parameter('gps_topic').value
        yaw_offset_topic = self.get_parameter('yaw_offset_topic').value
        rate = float(self.get_parameter('publish_rate').value)
        self.q = float(self.get_parameter('process_noise').value)
        self.r_default = float(self.get_parameter('gps_variance_default').value)
        self.use_msg_cov = bool(self.get_parameter('use_msg_covariance').value)
        self.gate = float(self.get_parameter('gps_gate_sigma').value)
        self.p0 = float(self.get_parameter('initial_variance').value)
        self.publish_identity_until_gps = bool(
            self.get_parameter('publish_identity_until_gps').value)

        # ENU conversion constants for the raw /gps/fix path
        _datum_lat_deg = self.get_parameter('datum_lat').value
        _datum_lon_deg = self.get_parameter('datum_lon').value
        self._datum_lat_deg = _datum_lat_deg
        self._datum_lon_deg = _datum_lon_deg
        self._cos_datum_lat = math.cos(math.radians(_datum_lat_deg))
        self._R_earth = 6_371_000.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Absolute-heading correction: map->odom yaw. 0 until gps_heading_align
        # locks and starts publishing => identity rotation (previous behavior).
        self.yaw_offset = 0.0
        self._c = 1.0   # cos(yaw_offset)
        self._s = 0.0   # sin(yaw_offset)
        self.have_offset = False

        # KF state (map-frame position) + per-axis variance
        self.px = self.py = 0.0
        self.var_x = self.var_y = self.p0
        self.initialized = False

        # Latest GPS measurement (consumed once)
        self.gps = None          # (x, y, rx, ry)
        self.gps_new = False

        self.lio_prev = None     # last LIO position used for prediction
        self.last_time = None

        self.create_subscription(Odometry, gps_topic, self.gps_cb, 10)
        self.create_subscription(Float64, yaw_offset_topic, self.offset_cb, 10)
        raw_gps_topic = self.get_parameter('raw_gps_topic').value
        self.create_subscription(NavSatFix, raw_gps_topic, self._raw_gps_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'map_odom_from_gps: 2D position KF (x,y) + ENU yaw from '
            f'{yaw_offset_topic} (identity until first offset). '
            f'q={self.q}, gate={self.gate} sigma. '
            f'Pre-lock GPS position from {raw_gps_topic} (datum '
            f'{_datum_lat_deg:.6f}, {_datum_lon_deg:.6f})')

    def offset_cb(self, msg: Float64):
        self.yaw_offset = float(msg.data)
        self._c = math.cos(self.yaw_offset)
        self._s = math.sin(self.yaw_offset)
        if not self.have_offset:
            self.have_offset = True
            self.get_logger().info(
                f'yaw_offset locked: {math.degrees(self.yaw_offset):.1f} deg; '
                f'map->odom now rotates odom(LIO) into ENU/north')

    def _latlon_to_enu(self, lat, lon):
        """Flat-earth ENU relative to the campus datum (identical formula to
        gps_heading_align and campus_nav.ENU — kept inline to avoid cross-package
        imports)."""
        east = self._R_earth * self._cos_datum_lat * math.radians(lon - self._datum_lon_deg)
        north = self._R_earth * math.radians(lat - self._datum_lat_deg)
        return east, north

    def _raw_gps_cb(self, msg: NavSatFix):
        """Pre-lock GPS position path: convert raw fix -> ENU using the fixed datum.

        Active only before heading lock (have_offset=False), i.e. while navsat
        has not yet established its datum and /odometry/gps is silent. Once
        heading is locked, offset_cb sets have_offset=True and this callback
        becomes a no-op; gps_cb (from /odometry/gps) takes over from then on.
        """
        if self.have_offset:
            return
        if msg.status.status < 0:  # STATUS_NO_FIX
            return
        east, north = self._latlon_to_enu(msg.latitude, msg.longitude)
        rx = ry = self.r_default
        # NavSatFix position_covariance is [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        # (east variance = [0], north variance = [4]).
        if (self.use_msg_cov
                and msg.position_covariance_type >= 2  # DIAGONAL_KNOWN or KNOWN
                and msg.position_covariance[0] > 0.0):
            rx = msg.position_covariance[0]
            ry = msg.position_covariance[4]
        self.gps = (east, north, rx, ry)
        self.gps_new = True

    def gps_cb(self, msg: Odometry):
        rx = ry = self.r_default
        if self.use_msg_cov:
            cxx = msg.pose.covariance[0]
            cyy = msg.pose.covariance[7]
            if cxx > 0.0:
                rx = cxx
            if cyy > 0.0:
                ry = cyy
        self.gps = (msg.pose.pose.position.x, msg.pose.pose.position.y, rx, ry)
        self.gps_new = True

    def _lio_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_link_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def on_timer(self):
        now = self.get_clock().now()
        lio = self._lio_xy()

        # Not enough to run the filter yet
        if lio is None or (not self.initialized and self.gps is None):
            if self.publish_identity_until_gps and not self.initialized:
                self.broadcast(0.0, 0.0, 0.0)
            return

        # Initialize the estimate from the first GPS fix
        if not self.initialized:
            self.px, self.py = self.gps[0], self.gps[1]
            self.var_x = self.var_y = self.p0
            self.lio_prev = lio
            self.last_time = now
            self.initialized = True
            self.gps_new = False
            self._broadcast_from_state(lio)
            return

        # --- Predict: advance by LIO delta (rotated odom->map), grow covariance ---
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt < 0.0:
            dt = 0.0
        dx = lio[0] - self.lio_prev[0]
        dy = lio[1] - self.lio_prev[1]
        self.px += self._c * dx - self._s * dy
        self.py += self._s * dx + self._c * dy
        self.lio_prev = lio
        self.var_x += self.q * dt
        self.var_y += self.q * dt

        # --- Update: fuse a fresh GPS fix (with outlier gate) ---
        if self.gps_new:
            self.gps_new = False
            zx, zy, rx, ry = self.gps
            self._kf_update_axis('x', zx, rx)
            self._kf_update_axis('y', zy, ry)

        self._broadcast_from_state(lio)

    def _kf_update_axis(self, axis, z, r):
        if axis == 'x':
            p, var = self.px, self.var_x
        else:
            p, var = self.py, self.var_y
        innov = z - p
        s = var + r
        # Mahalanobis outlier gate
        if self.gate > 0.0 and s > 0.0 and (innov * innov) > (self.gate * self.gate) * s:
            self.get_logger().warn(
                f'GPS {axis} rejected (innov={innov:.2f}, sqrt(S)={math.sqrt(s):.2f})')
            return
        k = var / s
        p = p + k * innov
        var = (1.0 - k) * var
        if axis == 'x':
            self.px, self.var_x = p, var
        else:
            self.py, self.var_y = p, var

    def _broadcast_from_state(self, lio):
        # map<-odom rotates by yaw_offset, so p_map = R * p_odom + t.
        # For base_link: p_gps(map,ENU) = R * lio(odom) + t  =>  t = p - R*lio.
        rx = self._c * lio[0] - self._s * lio[1]
        ry = self._s * lio[0] + self._c * lio[1]
        self.broadcast(self.px - rx, self.py - ry, self.yaw_offset)

    def broadcast(self, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        # Yaw-only correction (odom(LIO) -> ENU); roll/pitch stay with LIO.
        tf.transform.rotation.z = math.sin(yaw * 0.5)
        tf.transform.rotation.w = math.cos(yaw * 0.5)
        self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = MapOdomFromGps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
