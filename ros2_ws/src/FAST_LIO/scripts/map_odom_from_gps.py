#!/usr/bin/env python3
"""Publish map->odom by fusing GPS + LIO on translation only (no rotation EKF).

Rationale
---------
This robot has no absolute-heading source (no magnetometer / dual GPS), so a
full pose EKF ends up trying to estimate an unobservable yaw and drifts. But we
*do* want to fuse position, because both GPS (noisy / offset) and LIO (slow
drift) are imperfect.

So this node runs a small **2D position Kalman filter (x, y only)** and keeps
the rotation of map->odom as identity — heading is trusted entirely to LIO.

Filter
------
State: robot position in the map frame  p = [px, py]  (two independent scalar KFs)
    Predict: p += (LIO position delta)         # LIO gives accurate short-term motion
             P  += q * dt                       # process noise (LIO drift)
    Update : z = GPS position, R = GPS covariance (from msg, or default)
             standard KF update, with a Mahalanobis gate to reject GPS spikes

Because map->odom rotation is identity, LIO position deltas (odom frame) equal
map-frame deltas, so adding them to the map-frame estimate is consistent.

Output: map->odom translation = p_filtered - LIO_position ; rotation = identity.

Inputs:
    /odometry/gps (nav_msgs/Odometry)  -- robot position in map (from navsat)
    TF odom->base_link                 -- from FAST-LIO chain
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class MapOdomFromGps(Node):
    def __init__(self):
        super().__init__('map_odom_from_gps')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('gps_topic', '/odometry/gps')
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

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        gps_topic = self.get_parameter('gps_topic').value
        rate = float(self.get_parameter('publish_rate').value)
        self.q = float(self.get_parameter('process_noise').value)
        self.r_default = float(self.get_parameter('gps_variance_default').value)
        self.use_msg_cov = bool(self.get_parameter('use_msg_covariance').value)
        self.gate = float(self.get_parameter('gps_gate_sigma').value)
        self.p0 = float(self.get_parameter('initial_variance').value)
        self.publish_identity_until_gps = bool(
            self.get_parameter('publish_identity_until_gps').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

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
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'map_odom_from_gps: 2D position KF (x,y), identity rotation. '
            f'q={self.q}, gate={self.gate} sigma')

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
                self.broadcast(0.0, 0.0)
            return

        # Initialize the estimate from the first GPS fix
        if not self.initialized:
            self.px, self.py = self.gps[0], self.gps[1]
            self.var_x = self.var_y = self.p0
            self.lio_prev = lio
            self.last_time = now
            self.initialized = True
            self.gps_new = False
            self.broadcast(self.px - lio[0], self.py - lio[1])
            return

        # --- Predict: advance by LIO delta, grow covariance ---
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt < 0.0:
            dt = 0.0
        self.px += lio[0] - self.lio_prev[0]
        self.py += lio[1] - self.lio_prev[1]
        self.lio_prev = lio
        self.var_x += self.q * dt
        self.var_y += self.q * dt

        # --- Update: fuse a fresh GPS fix (with outlier gate) ---
        if self.gps_new:
            self.gps_new = False
            zx, zy, rx, ry = self.gps
            self._kf_update_axis('x', zx, rx)
            self._kf_update_axis('y', zy, ry)

        self.broadcast(self.px - lio[0], self.py - lio[1])

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

    def broadcast(self, x, y):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0   # identity: heading trusted to LIO
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
