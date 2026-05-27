#!/usr/bin/env python3
"""Minimal /cmd_vel bridge for the Galaxea R1Pro chassis.

This node has exactly one job:

    /cmd_vel (geometry_msgs/Twist)
        --> /motion_target/target_speed_chassis (geometry_msgs/TwistStamped)

It repackages the incoming Twist into a TwistStamped, stamps it with the ROS
clock and the ``base_link`` frame, and forwards it to the chassis controller.

The chassis EKF / /odom / /odom/health logic that used to live here was
migrated into the C++ FAST-LIO node (``fastlio_odometry``) so that the laser
localization and chassis dead-reckoning share a single process. That cut the
Python CPU footprint from ~100% of one core down to noise.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('chassis_target_topic',
                               '/motion_target/target_speed_chassis')
        self.declare_parameter('base_frame', 'base_link')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        target_topic = self.get_parameter('chassis_target_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        reliable_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.pub_target = self.create_publisher(
            TwistStamped, target_topic, reliable_qos
        )
        self.sub_cmd = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_callback, reliable_qos
        )

        self.get_logger().info(
            f"cmd_vel_bridge up: {cmd_vel_topic} -> {target_topic}"
        )

    def cmd_callback(self, msg: Twist):
        """Pass-through repackage of /cmd_vel into a TwistStamped."""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.base_frame
        stamped.twist = msg
        self.pub_target.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
