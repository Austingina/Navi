import rclpy
from rclpy.node import Node

class PrintTimeNode(Node):
    def __init__(self):
        super().__init__('print_time_node')
        timer_period = 0.1  # 每0.1秒打印一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        self.get_logger().info(f"Current ROS time: {now.nanoseconds / 1e9:.9f} s")

def main(args=None):
    rclpy.init(args=args)
    node = PrintTimeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()