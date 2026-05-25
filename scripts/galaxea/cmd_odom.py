#!/usr/bin/env python3
"""Bridge node for the Galaxea R1Pro chassis.

Two independent jobs live in this node so it can run from the existing
``start_r1pro.bash`` without spawning an extra process:

1. ``/cmd_vel`` (geometry_msgs/Twist)
       --> ``/motion_target/target_speed_chassis`` (geometry_msgs/TwistStamped)
   Plain repackage: copy the Twist into a TwistStamped, stamp it with the
   ROS clock and the ``base_link`` frame, and forward it to the chassis
   controller.

2. ``/motion_control/chassis_speed`` (TwistStamped, measured body velocity)
   + ``/livox/imu`` (sensor_msgs/Imu, yaw rate)
   + optional ``/Odometry`` (FAST_LIO, nav_msgs/Odometry)
       --> ``/odom`` (nav_msgs/Odometry)
   A 4-state EKF integrates the measured chassis velocity and the IMU yaw
   rate to estimate ``(px, py, theta, gyro_bias_z)`` in the odom frame.
   The node starts producing /odom immediately; it does **not** depend on
   /Odometry being available. When /Odometry **is** available, it is used
   as a measurement update that simultaneously corrects pose drift and
   makes the IMU yaw-rate bias observable. The first /Odometry message
   also snaps the EKF origin so /odom shares the same frame definition as
   the laser odometry from that point on.

State vector::

    x = [px, py, theta, b_g]
          |   |    |     |
          |   |    |     +-- IMU yaw-rate bias (rad/s); only observable
          |   |    |          while /Odometry is supplying a theta meas.
          |   |    +-------- Heading in the odom frame (rad).
          |   +------------- Position y in the odom frame (m).
          +----------------- Position x in the odom frame (m).

Prediction is driven by the chassis_speed callback (typical chassis
feedback rate ~50 Hz), using the latest bias-corrected IMU yaw rate; the
IMU callback itself only caches the latest gyro reading. /Odometry is
optional and arrives sporadically (10 Hz from FAST_LIO).
"""

import json
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Small math helpers (no scipy dep)
# ---------------------------------------------------------------------------

def stamp_to_sec(stamp) -> float:
    """ROS2 builtin_interfaces/Time -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Z-Y-X yaw extraction. Stable when pitch/roll are small (chassis case)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    """Return (qx, qy, qz, qw) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# Plain-Python 4-state EKF
# ---------------------------------------------------------------------------

@dataclass
class EKFConfig:
    # Per-sqrt-second noise on the velocity / gyro inputs used in the predict
    # step. These drive how fast the position/heading covariance grows when
    # /Odometry is not available, so set them to reflect measurement noise on
    # chassis_speed (typically 2-5 cm/s for differential drive feedback) and
    # short-term gyro noise + un-modelled bias drift.
    sigma_vx: float = 0.05         # m/s
    sigma_vy: float = 0.05         # m/s
    sigma_omega: float = 0.01      # rad/s
    # Bias random-walk noise (per sqrt-second).
    sigma_bias: float = 1e-4       # rad / s / sqrt(s)

    # /Odometry measurement noise (diagonal). FAST_LIO doesn't always populate
    # the Odometry covariance, so we override here.
    sigma_lio_xy: float = 0.05     # m
    sigma_lio_yaw: float = 0.02    # rad

    # Initial covariance: position/heading start at zero (since we have no
    # global reference until /Odometry arrives) with large uncertainty so the
    # first LIO update snaps cleanly. Bias starts at zero.
    init_sigma_pos: float = 1.0
    init_sigma_yaw: float = 0.5
    init_sigma_bias: float = 0.05


class ChassisEKF:
    """Minimal 4-state EKF: [px, py, theta, bias_omega_z]."""

    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.diag([
            cfg.init_sigma_pos ** 2,
            cfg.init_sigma_pos ** 2,
            cfg.init_sigma_yaw ** 2,
            cfg.init_sigma_bias ** 2,
        ])
        # True after the *first* prediction or measurement (i.e. the EKF has
        # something other than the prior to publish).
        self.has_state = False
        # True after the first /Odometry has been folded in -- before then,
        # /odom is in a "free-running" frame and the first /Odometry will be
        # snapped (not Kalman-updated) so /odom matches LIO's origin.
        self.lio_anchored = False

    # -- accessors ----------------------------------------------------
    @property
    def px(self) -> float: return float(self.x[0])
    @property
    def py(self) -> float: return float(self.x[1])
    @property
    def theta(self) -> float: return float(self.x[2])
    @property
    def bias(self) -> float: return float(self.x[3])

    # -- prediction ---------------------------------------------------
    def predict(self, vx: float, vy: float, omega_imu: float, dt: float):
        """Integrate one step using measured body velocity + IMU gyro.

        Uses a midpoint yaw for the position integration to reduce truncation
        error during sharp turns and long /Odometry outages.
        """
        if dt <= 0.0:
            return
        # Cap dt so a long callback stall doesn't blow up the covariance in
        # a single update.
        dt = min(dt, 0.5)

        omega_corr = omega_imu - self.x[3]
        theta = self.x[2]
        theta_mid = theta + 0.5 * omega_corr * dt
        cos_m = math.cos(theta_mid)
        sin_m = math.sin(theta_mid)

        # State update
        self.x[0] += (vx * cos_m - vy * sin_m) * dt
        self.x[1] += (vx * sin_m + vy * cos_m) * dt
        self.x[2] = wrap_angle(theta + omega_corr * dt)
        # x[3] (bias) evolves as a random walk in the noise model only.

        # Jacobian wrt state (evaluated at the old theta for simplicity).
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        F = np.array([
            [1.0, 0.0, -(vx * sin_t + vy * cos_t) * dt, 0.0],
            [0.0, 1.0,  (vx * cos_t - vy * sin_t) * dt, 0.0],
            [0.0, 0.0,  1.0,                            -dt],
            [0.0, 0.0,  0.0,                             1.0],
        ])

        cfg = self.cfg
        # Velocity inputs rotate from body to world via theta; omega -> theta;
        # bias drift is a pure random walk.
        sx2 = (cfg.sigma_vx * dt) ** 2 * cos_t * cos_t \
            + (cfg.sigma_vy * dt) ** 2 * sin_t * sin_t
        sy2 = (cfg.sigma_vx * dt) ** 2 * sin_t * sin_t \
            + (cfg.sigma_vy * dt) ** 2 * cos_t * cos_t
        sxy = ((cfg.sigma_vx ** 2 - cfg.sigma_vy ** 2) * dt * dt) * sin_t * cos_t
        sth2 = (cfg.sigma_omega * dt) ** 2
        sb2 = (cfg.sigma_bias ** 2) * dt
        Q = np.array([
            [sx2, sxy, 0.0,  0.0],
            [sxy, sy2, 0.0,  0.0],
            [0.0, 0.0, sth2, 0.0],
            [0.0, 0.0, 0.0,  sb2],
        ])

        self.P = F @ self.P @ F.T + Q
        self.has_state = True

    # -- measurement update -------------------------------------------
    def update_lio(self, px: float, py: float, theta: float,
                   sigma_xy: Optional[float] = None,
                   sigma_yaw: Optional[float] = None):
        """Standard linear-H Kalman update with z = [px, py, theta]."""
        cfg = self.cfg
        sxy = cfg.sigma_lio_xy if sigma_xy is None else sigma_xy
        sth = cfg.sigma_lio_yaw if sigma_yaw is None else sigma_yaw
        R = np.diag([sxy ** 2, sxy ** 2, sth ** 2])

        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        z = np.array([px, py, theta], dtype=np.float64)
        y = z - H @ self.x
        y[2] = wrap_angle(y[2])

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # singular -- skip rather than crash
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.lio_anchored = True
        self.has_state = True

    def snap_to(self, px: float, py: float, theta: float,
                sigma_pos: float = 0.02, sigma_yaw: float = 0.01):
        """Bootstrap pose to the first /Odometry without Kalman fusion.

        Avoids a multi-meter jump in /odom when LIO is the first absolute
        reference the EKF ever sees (until then /odom has been free-running
        from the origin we picked on startup).
        """
        self.x[0] = px
        self.x[1] = py
        self.x[2] = wrap_angle(theta)
        # Keep the existing bias estimate untouched.
        self.P = np.diag([
            sigma_pos ** 2, sigma_pos ** 2, sigma_yaw ** 2, self.P[3, 3]
        ])
        self.lio_anchored = True
        self.has_state = True


# ---------------------------------------------------------------------------
# Bridge / EKF node
# ---------------------------------------------------------------------------

# Modes purely for logging / /odom/health visibility.
MODE_FREE_RUNNING = "EKF_FREE_RUNNING"     # no /Odometry ever seen yet
MODE_LIO_TRACKING = "EKF_LIO_TRACKING"     # /Odometry healthy, EKF + updates
MODE_LIO_LOST = "EKF_DEAD_RECKONING"       # had LIO, now stale


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # --- parameters --------------------------------------------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('chassis_target_topic',
                               '/motion_target/target_speed_chassis')
        self.declare_parameter('chassis_speed_topic',
                               '/motion_control/chassis_speed')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('lio_topic', '/Odometry')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('health_topic', '/odom/health')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # /Odometry is considered stale after this many seconds. Used only
        # for the health/status flag; the EKF keeps integrating regardless.
        self.declare_parameter('lio_timeout_sec', 0.5)
        # If chassis_speed stops arriving for this long, the EKF stops
        # publishing (we have nothing to integrate).
        self.declare_parameter('chassis_speed_timeout_sec', 1.0)
        # Health publish period (seconds).
        self.declare_parameter('health_publish_period_sec', 1.0)

        # EKF noise overrides (all optional).
        for k, v in (
            ('sigma_vx', 0.05), ('sigma_vy', 0.05), ('sigma_omega', 0.01),
            ('sigma_bias', 1e-4),
            ('sigma_lio_xy', 0.05), ('sigma_lio_yaw', 0.02),
            ('init_sigma_pos', 1.0), ('init_sigma_yaw', 0.5),
            ('init_sigma_bias', 0.05),
        ):
            self.declare_parameter(k, v)

        p = self.get_parameter
        cfg = EKFConfig(
            sigma_vx=p('sigma_vx').value,
            sigma_vy=p('sigma_vy').value,
            sigma_omega=p('sigma_omega').value,
            sigma_bias=p('sigma_bias').value,
            sigma_lio_xy=p('sigma_lio_xy').value,
            sigma_lio_yaw=p('sigma_lio_yaw').value,
            init_sigma_pos=p('init_sigma_pos').value,
            init_sigma_yaw=p('init_sigma_yaw').value,
            init_sigma_bias=p('init_sigma_bias').value,
        )
        self.ekf = ChassisEKF(cfg)

        self.lio_timeout = float(p('lio_timeout_sec').value)
        self.chassis_timeout = float(p('chassis_speed_timeout_sec').value)
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value

        # --- runtime state ----------------------------------------------
        self.last_predict_ts: float = -1.0
        self.last_imu_ts: float = -1.0
        self.last_chassis_ts: float = -1.0
        self.last_lio_ts: float = -1.0
        self.last_omega_imu: float = 0.0
        # Latest measured body velocity (kept around so we can publish it in
        # the /odom twist field even when chassis_speed didn't tick).
        self.last_vx: float = 0.0
        self.last_vy: float = 0.0
        self.last_omega_chassis: float = 0.0
        self.transitions: int = 0
        self.mode: str = MODE_FREE_RUNNING

        # --- QoS ---------------------------------------------------------
        reliable_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(  # IMU might be best-effort under load
            depth=200,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # --- /cmd_vel -> /motion_target/target_speed_chassis (unchanged) -
        self.sub_cmd = self.create_subscription(
            Twist, p('cmd_vel_topic').value, self.cmd_callback, reliable_qos
        )
        self.pub_target = self.create_publisher(
            TwistStamped, p('chassis_target_topic').value, reliable_qos
        )

        # --- EKF inputs / output -----------------------------------------
        self.sub_chassis = self.create_subscription(
            TwistStamped, p('chassis_speed_topic').value,
            self.chassis_callback, reliable_qos,
        )
        self.sub_imu = self.create_subscription(
            Imu, p('imu_topic').value, self.imu_callback, sensor_qos,
        )
        self.sub_lio = self.create_subscription(
            Odometry, p('lio_topic').value, self.lio_callback, reliable_qos,
        )
        self.pub_odom = self.create_publisher(
            Odometry, p('odom_topic').value, reliable_qos
        )
        self.pub_health = self.create_publisher(
            String, p('health_topic').value, 10
        )

        self.create_timer(
            float(p('health_publish_period_sec').value), self._publish_health
        )

        self.get_logger().info(
            f"bridge_node up. cmd_vel={p('cmd_vel_topic').value} -> "
            f"{p('chassis_target_topic').value}; EKF inputs: "
            f"chassis_speed={p('chassis_speed_topic').value}, "
            f"imu={p('imu_topic').value}, optional lio={p('lio_topic').value} "
            f"-> {p('odom_topic').value}"
        )

    # ---------- /cmd_vel -> /motion_target/target_speed_chassis ----------
    def cmd_callback(self, msg: Twist):
        """Pass-through repackage of /cmd_vel into a TwistStamped."""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.base_frame
        stamped.twist = msg
        self.pub_target.publish(stamped)

    # ---------- IMU -------------------------------------------------------
    def imu_callback(self, msg: Imu):
        ts = stamp_to_sec(msg.header.stamp)
        if ts <= 0.0:
            ts = self._now_sec()
        self.last_omega_imu = float(msg.angular_velocity.z)
        self.last_imu_ts = ts

    # ---------- /motion_control/chassis_speed (EKF predict + publish) ----
    def chassis_callback(self, msg: TwistStamped):
        ts = stamp_to_sec(msg.header.stamp)
        if ts <= 0.0:
            ts = self._now_sec()

        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        omega_chassis = float(msg.twist.angular.z)
        self.last_vx = vx
        self.last_vy = vy
        self.last_omega_chassis = omega_chassis
        self.last_chassis_ts = ts

        # Prefer IMU yaw rate if we have a recent reading (it's typically
        # an order of magnitude lower-noise than chassis feedback yaw); fall
        # back to chassis_speed.angular.z if no IMU has been seen yet.
        if self.last_imu_ts > 0.0 and \
                (self._now_sec() - self.last_imu_ts) < 1.0:
            omega_used = self.last_omega_imu
            use_chassis_omega = False
        else:
            omega_used = omega_chassis
            use_chassis_omega = True

        if self.last_predict_ts <= 0.0:
            # First message: just stamp the predict clock; the EKF prior
            # already represents "we are at the origin with large covariance".
            self.last_predict_ts = ts
            self.ekf.has_state = True
        else:
            dt = ts - self.last_predict_ts
            if dt > 0.0:
                if use_chassis_omega:
                    # We're feeding omega *without* bias correction; freeze
                    # the bias state for this step by treating the input as
                    # already corrected (i.e. add the current bias back).
                    omega_input = omega_used + self.ekf.bias
                else:
                    omega_input = omega_used
                self.ekf.predict(vx, vy, omega_input, dt)
                self.last_predict_ts = ts

        # Re-evaluate mode for visibility (no behavioural side-effect).
        self._update_mode()

        # Publish /odom at the chassis_speed rate.
        self._publish_odom(ts)

    # ---------- /Odometry (optional measurement update) ------------------
    def lio_callback(self, msg: Odometry):
        ts = stamp_to_sec(msg.header.stamp)
        if ts <= 0.0:
            ts = self._now_sec()

        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        if not self.ekf.lio_anchored:
            # First LIO we have ever seen -> snap so /odom shares the same
            # frame definition as LIO from now on, instead of doing a giant
            # Kalman-correction jump that depends on whatever drift had
            # accumulated during free-running.
            self.ekf.snap_to(px, py, yaw)
            self.get_logger().info(
                f"EKF anchored to first /Odometry: "
                f"({px:.3f}, {py:.3f}, yaw={math.degrees(yaw):.1f} deg)"
            )
        else:
            # Predict up to the LIO timestamp using the latest known inputs
            # so the update is applied at the right instant in time.
            if self.last_predict_ts > 0.0 and ts > self.last_predict_ts:
                dt = ts - self.last_predict_ts
                # Use the last known velocity (zero if chassis_speed went
                # stale) and the latest IMU gyro.
                now = self._now_sec()
                stale = (self.last_chassis_ts <= 0.0) or \
                        ((now - self.last_chassis_ts) > self.chassis_timeout)
                vx = 0.0 if stale else self.last_vx
                vy = 0.0 if stale else self.last_vy
                self.ekf.predict(vx, vy, self.last_omega_imu, dt)
                self.last_predict_ts = ts
            self.ekf.update_lio(px, py, yaw)

        self.last_lio_ts = ts
        self._update_mode()

    # ---------- publish helpers ------------------------------------------
    def _publish_odom(self, ts: float):
        out = Odometry()
        sec = int(ts)
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = int(round((ts - sec) * 1e9))
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame

        out.pose.pose.position.x = self.ekf.px
        out.pose.pose.position.y = self.ekf.py
        out.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self.ekf.theta)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Pose covariance (6x6 row-major: x, y, z, roll, pitch, yaw).
        P = self.ekf.P
        cov = [0.0] * 36
        cov[0]  = float(P[0, 0])
        cov[1]  = float(P[0, 1])
        cov[6]  = float(P[1, 0])
        cov[7]  = float(P[1, 1])
        cov[14] = 1e-3                  # z (un-estimated, small const)
        cov[21] = 1e-3                  # roll
        cov[28] = 1e-3                  # pitch
        cov[35] = float(P[2, 2])        # yaw
        out.pose.covariance = cov

        # Twist: latest measured chassis velocity + bias-corrected IMU yaw rate
        # (if available; else chassis_speed yaw).
        out.twist.twist.linear.x = self.last_vx
        out.twist.twist.linear.y = self.last_vy
        out.twist.twist.linear.z = 0.0
        if self.last_imu_ts > 0.0 and \
                (self._now_sec() - self.last_imu_ts) < 1.0:
            out.twist.twist.angular.z = self.last_omega_imu - self.ekf.bias
        else:
            out.twist.twist.angular.z = self.last_omega_chassis

        tw_cov = [0.0] * 36
        cfg = self.ekf.cfg
        tw_cov[0]  = cfg.sigma_vx ** 2
        tw_cov[7]  = cfg.sigma_vy ** 2
        tw_cov[14] = 1e-3
        tw_cov[21] = 1e-3
        tw_cov[28] = 1e-3
        tw_cov[35] = cfg.sigma_omega ** 2
        out.twist.covariance = tw_cov

        self.pub_odom.publish(out)

    def _update_mode(self):
        new_mode = self.mode
        if not self.ekf.lio_anchored:
            new_mode = MODE_FREE_RUNNING
        else:
            lio_age = self._now_sec() - self.last_lio_ts
            new_mode = MODE_LIO_LOST if lio_age > self.lio_timeout \
                else MODE_LIO_TRACKING
        if new_mode != self.mode:
            self.transitions += 1
            self.get_logger().warn(
                f"/odom mode: {self.mode} -> {new_mode}"
            )
            self.mode = new_mode

    def _publish_health(self):
        # Make sure stale-LIO transitions become visible even if no callback
        # fired since.
        self._update_mode()
        now = self._now_sec()
        status = {
            "mode":              self.mode,
            "imu_age_sec":       round(now - self.last_imu_ts, 3)
                                  if self.last_imu_ts > 0 else None,
            "chassis_age_sec":   round(now - self.last_chassis_ts, 3)
                                  if self.last_chassis_ts > 0 else None,
            "lio_age_sec":       round(now - self.last_lio_ts, 3)
                                  if self.last_lio_ts > 0 else None,
            "transitions":       self.transitions,
            "lio_anchored":      self.ekf.lio_anchored,
            "ekf_px":            self.ekf.px,
            "ekf_py":            self.ekf.py,
            "ekf_theta_deg":     math.degrees(self.ekf.theta),
            "ekf_gyro_bias_rps": self.ekf.bias,
            "ekf_pos_sigma_m":   float(math.sqrt(max(0.0, self.ekf.P[0, 0]))),
            "ekf_yaw_sigma_deg": float(math.degrees(
                math.sqrt(max(0.0, self.ekf.P[2, 2])))),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.pub_health.publish(msg)

    # ---------- misc ------------------------------------------------------
    def _now_sec(self) -> float:
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
