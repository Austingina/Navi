#!/usr/bin/env python3
"""Publish zero velocity when the navigation stack shuts down.

Why this exists
---------------
The velocity chain ends at collision_monitor, which publishes /cmd_vel to the
(external) Galaxea chassis:

    controller_server -> /cmd_vel_nav -> velocity_smoother
        -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel -> chassis

When the bringup launch is Ctrl-C'd (or the tmux session killed), every node in
that chain dies at once. Nothing sends a final zero, so the last /cmd_vel the
chassis received is whatever collision_monitor happened to publish last. If the
robot was driving, it keeps the last commanded velocity until the chassis's own
watchdog (if any) times out.

This node is silent during normal operation — it must never fight
collision_monitor for /cmd_vel — and publishes a short burst of zeros on the way
out, so the last thing on the wire is a stop.

Signal handling
---------------
rclpy is initialized with SignalHandlerOptions.NO so it does not install its own
handlers. SIGINT / SIGTERM / SIGHUP are caught here and only set a flag; the
context therefore stays valid through the burst, which is what lets the zeros
actually reach the chassis. (rclpy's default handler would race us to shutting
down the context, after which publishing is a no-op.)

The burst is driven by time.sleep, not a ROS timer, so it works identically
under use_sim_time when /clock has already stopped.

Ordering note: the other stack nodes get the same signal we do, so for the first
few hundred ms collision_monitor may still publish. The burst outlives that by
design — stop_duration_s (default 1.5 s) is far longer than their teardown, so a
zero is always the final message. It also fits inside ros2 launch's 5 s
SIGINT->SIGTERM escalation window.
"""

import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # /cmd_vel is what the chassis subscribes to in the outdoor stack (see
        # param_outdoor_elevation.yaml's collision_monitor comment).
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # The indoor path goes through cmd_vel_bridge.py, which republishes as
        # TwistStamped on this topic. Zeroing both costs nothing and covers the
        # case where the bridge is not running. Set to '' to disable.
        self.declare_parameter('chassis_topic',
                               '/motion_target/target_speed_chassis')
        self.declare_parameter('base_frame', 'base_link')
        # How long to keep sending zeros after the shutdown signal, and how
        # fast. Long enough to outlive the rest of the stack's teardown.
        self.declare_parameter('stop_duration_s', 1.5)
        self.declare_parameter('stop_rate_hz', 20.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        chassis_topic = self.get_parameter('chassis_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.stop_duration_s = float(
            self.get_parameter('stop_duration_s').value)
        self.stop_rate_hz = max(
            1.0, float(self.get_parameter('stop_rate_hz').value))

        # Match cmd_vel_bridge.py's QoS so the chassis subscriber matches us.
        reliable_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Created up front, not on shutdown: the publisher needs to already be
        # DDS-matched with the chassis subscriber when the burst goes out. A
        # publisher created during teardown would spend the whole burst still
        # discovering, and every message would be dropped.
        self.cmd_vel_pub = self.create_publisher(
            Twist, cmd_vel_topic, reliable_qos)
        self.chassis_pub = None
        if chassis_topic:
            self.chassis_pub = self.create_publisher(
                TwistStamped, chassis_topic, reliable_qos)

        self.get_logger().info(
            f'safety_stop armed: will publish zero velocity on '
            f'{cmd_vel_topic}'
            + (f' + {chassis_topic}' if self.chassis_pub else '')
            + f' for {self.stop_duration_s:.1f} s on SIGINT/SIGTERM/SIGHUP. '
            f'Silent until then.')

    def publish_stop_burst(self):
        """Spam zero velocity so the chassis's last command is a stop."""
        n_cmd = self.cmd_vel_pub.get_subscription_count()
        n_chassis = (self.chassis_pub.get_subscription_count()
                     if self.chassis_pub else 0)
        self.get_logger().warn(
            f'shutting down -> zero velocity for {self.stop_duration_s:.1f} s '
            f'({n_cmd} cmd_vel subscriber(s), {n_chassis} chassis '
            f'subscriber(s))')
        if n_cmd == 0 and n_chassis == 0:
            # Not fatal — worth flagging, since it means nothing is listening
            # and the robot will not be stopped by us.
            self.get_logger().error(
                'no subscribers on either stop topic; the chassis will NOT be '
                'stopped by safety_stop. Check the topic names.')

        zero = Twist()
        period = 1.0 / self.stop_rate_hz
        deadline = time.monotonic() + self.stop_duration_s
        sent = 0
        while time.monotonic() < deadline:
            self.cmd_vel_pub.publish(zero)
            if self.chassis_pub is not None:
                stamped = TwistStamped()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.header.frame_id = self.base_frame
                # stamped.twist is already all-zero
                self.chassis_pub.publish(stamped)
            sent += 1
            time.sleep(period)
        self.get_logger().warn(f'zero velocity sent {sent} time(s); exiting.')


def main():
    # SignalHandlerOptions.NO: we own the signals, so the context survives long
    # enough to actually publish the stop. See the module docstring.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SafetyStop()

    shutdown = threading.Event()

    def _on_signal(signum, _frame):
        # Keep this minimal and re-entrant-safe: no logging, no publishing.
        # The burst runs on the main thread once the spin loop unwinds.
        node.get_logger().debug(f'signal {signum} received')
        shutdown.set()

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _on_signal)

    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.publish_stop_burst()
        except Exception as exc:  # noqa: BLE001 — never block teardown
            print(f'[safety_stop] stop burst failed: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
