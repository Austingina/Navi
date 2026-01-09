import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # 发布 TwistStamped 消息
        self.publisher_ = self.create_publisher(TwistStamped, '/motion_target/target_speed_chassis', 10)
        
        # 设置发布频率
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 设置初始化的 TwistStamped 消息
        self.twist = TwistStamped()

    def timer_callback(self):
        key = self.get_key()
        if key:
            # 根据按键输入调整速度
            if key == 'w':  # 前进
                self.twist.twist.linear.x = 0.5
                self.twist.twist.angular.z = 0.0
            elif key == 's':  # 后退
                self.twist.twist.linear.x = -0.5
                self.twist.twist.angular.z = 0.0
            elif key == 'a':  # 向左转
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = 0.5
            elif key == 'd':  # 向右转
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = -0.5
            elif key == 'q':  # 停止
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = 0.0
            else:
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = 0.0

            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.twist)

    def get_key(self):
        # 获取键盘输入
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(1)
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    keyboard_control_node = KeyboardControl()

    rclpy.spin(keyboard_control_node)

    keyboard_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
