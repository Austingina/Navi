#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # 订阅 /cmd_vel (Twist)，转发为 /motion_target/target_speed_chassis (TwistStamped)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_target = self.create_publisher(TwistStamped, '/motion_target/target_speed_chassis', 10)

        # 订阅 /motion_control/chassis_speed (TwistStamped)，转发为 /odom (Odometry)
        self.sub_chassis = self.create_subscription(TwistStamped, '/motion_control/chassis_speed', self.chassis_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        print("Bridge node started...")

    # /cmd_vel → /motion_target/target_speed_chassis
    # def cmd_callback(self, msg: Twist):
    #     stamped = TwistStamped()
    #     stamped.header.stamp = self.get_clock().now().to_msg()
    #     stamped.header.frame_id = 'base_link'
    #     stamped.twist = msg
    #     print(f"[cmd_vel] v=({msg.linear.x:.2f},{msg.linear.y:.2f}), w={msg.angular.z:.2f}")
    #     self.pub_target.publish(stamped)
    def cmd_callback(self, msg: Twist):
        # 互斥约束：线速度与角速度不能同时存在

        # if abs(msg.linear.x) > 1e-6 or abs(msg.linear.y) > 1e-6:  
        #     # 有线速度 → 角速度清零
        #     msg.angular.z = 0.0

        # elif abs(msg.angular.z) > 1e-6:
        #     # 有角速度 → 线速度清零
        #     msg.linear.x = 0.0
        #     msg.linear.y = 0.0

        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg

        print(f"[cmd_vel→target] v=({msg.linear.x:.2f},{msg.linear.y:.2f}), w={msg.angular.z:.2f}")
        self.pub_target.publish(stamped)

    # /motion_control/chassis_speed → /odom
    def chassis_callback(self, msg: TwistStamped):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 填充速度
        odom.twist.twist = msg.twist

        # pose 暂时置零（如果需要真实位姿，可以做积分）
        odom.pose.pose.orientation.w = 1.0

        print(f"[chassis_speed] v=({msg.twist.linear.x:.4f},{msg.twist.linear.y:.4f}), w={msg.twist.angular.z:.4f}")
        self.pub_odom.publish(odom)

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
