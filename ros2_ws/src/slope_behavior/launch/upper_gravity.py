import rclpy
from rclpy.node import Node
import sys
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
# from hdas_msg.msg import MotorControl
import time
from std_msgs.msg import Bool
from rclpy.duration import Duration
import pickle
import copy

qos_profile = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,  
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
)

class JointStateSender(Node):
    def __init__(self):
        super().__init__('joint_state_sender')
        self.left_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_joint_state_arm_left', qos_profile)
        self.right_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_joint_state_arm_right', qos_profile)
        self.torso_joint_state_pub = self.create_publisher(JointState, '/torso_joint_target_position_', qos_profile)
        self.torso_joint_state_pub_real = self.create_publisher(JointState, '/motion_target/target_joint_state_torso', qos_profile)

        self.fake_pub_left = self.create_publisher(JointState, '/hdas/feedback_arm_left', qos_profile)
        self.fake_pub_right = self.create_publisher(JointState, '/hdas/feedback_arm_right', qos_profile)
        self.left_gripper_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_position_gripper_left', qos_profile)
        self.right_gripper_joint_state_pub = self.create_publisher(JointState, '/motion_target/target_position_gripper_right', qos_profile)

        self.chassis_command_pub = self.create_publisher(TwistStamped, '/motion_target/target_speed_chassis', qos_profile)
        self.acc_limit_pub = self.create_publisher(TwistStamped, '/motion_target/chassis_acc_limit', qos_profile)
        self.breaking_mode_pub = self.create_publisher(Bool, '/motion_target/brake_mode', qos_profile)
        self.command = None
        self.feedback_running = False

    def send_joint_state_real(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_vel_limit(self, vel_limit_left, vel_limit_right, vel_limit_torso):
        left_joint_state = JointState()
        left_joint_state.velocity = vel_limit_left
        right_joint_state = JointState()
        right_joint_state.velocity = vel_limit_right
        torso_joint_state = JointState()
        torso_joint_state.velocity = vel_limit_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_gripper_joint_state(self, position_left, position_right):
        gripper_left_joint_state = JointState()
        gripper_left_joint_state.position = position_left
        gripper_right_joint_state = JointState()
        gripper_right_joint_state.position = position_right

        self.left_gripper_joint_state_pub.publish(gripper_left_joint_state)
        self.right_gripper_joint_state_pub.publish(gripper_right_joint_state)    

    def send_breaking_mode(self,breaking_mode_signal):
        breaking_mode_msg = Bool()
        breaking_mode_msg.data = bool(breaking_mode_signal[0])
        self.breaking_mode_pub.publish(breaking_mode_msg)

    def run(self):
        while True:
            try:
                # 速度限制（躯干速度放慢）
                self.send_vel_limit([1.6,1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,1.6,4.0,4.0,4.0], [0.8,0.8,0.7,0.7])

                # 速度限制（出厂默认速度）
                # self.send_vel_limit([1.6,1.6,1.6,1.6,4.0,4.0,4.0],[1.6,1.6,1.6,1.6,4.0,4.0,4.0], [1.0,1.6,1.4,1.4])

                # 关节状态
                #双手下垂，躯干直立
                self.send_joint_state_real([0.0,0.0,0.0,0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0,0.0,0.0,0.0], [0.0, 0.0, 0.0, 0.0])

                #双手抬起90度，躯干直立
                # self.send_joint_state_real([0.0,0.0,0.0,-1.57,0.0,0.0,0.0], [0.0,0.0,0.0,-1.57,0.0,0.0,0.0], [0.0, 0.0, 0.0, 0.0])

                #双手抬起90度，躯干弯曲
                # self.send_joint_state_real([0.0,0.0,0.0,-1.57,0.0,0.0,0.0], [0.0,0.0,0.0,-1.57,0.0,0.0,0.0], [1.74, -2.70, -0.96, 0.0])
                # self.send_breaking_mode([1])

                # 夹爪状态
                self.send_gripper_joint_state([100.0], [100.0])
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    sender = JointStateSender()
    try:
        sender.run()
    except KeyboardInterrupt:
        print("程序退出")
    finally:
        sender.destroy_node()   
        rclpy.shutdown()

if __name__ == '__main__':
    main()