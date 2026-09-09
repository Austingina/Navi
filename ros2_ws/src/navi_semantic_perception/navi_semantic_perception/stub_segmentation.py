#!/usr/bin/env python3
"""SAM3 之前的占位掩码节点：把图像下半部固定区域标成某个类，用来打通全链。

关键：它订阅【点云】而不是彩色图，并把 stamp 逐位照抄过来 —— 这样掩码与点云
是严格 1:1、同 stamp 的，那一层的 ApproximateTime（100 ms 硬编码容差、队列 10）
必然配对成功。SAM3 节点上线后应沿用同一做法。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


class StubSegmentation(Node):

    def __init__(self):
        super().__init__('stub_segmentation')
        p = self.declare_parameter
        self.in_topic = p('cloud_topic', '/semantic/camera1/points').value
        self.out_topic = p('segmentation_topic', '/semantic/camera1/segmentation').value
        # 默认 2 = grass，与 config/ontology.yaml 一致。用 1 会标成 sidewalk，
        # 而层 YAML 只认 grass -> 冒烟测试看不到任何代价。
        self.class_id = int(p('class_id', 2).value)
        # 归一化的矩形 [u0,v0,u1,v1]，默认下半部中间一块 —— 相机大俯角看地面，
        # 这块正好落在车前方的地面上，便于在 costmap 上核对位置。
        self.roi = [float(x) for x in p('roi', [0.30, 0.55, 0.70, 0.95]).value]
        self.pub = self.create_publisher(Image, self.out_topic, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.in_topic, self._on_cloud,
                                 rclpy.qos.qos_profile_sensor_data)
        self._mask = None
        self.get_logger().info(
            f'stub_segmentation: {self.in_topic} -> {self.out_topic}, '
            f'class_id={self.class_id}, roi={self.roi}')

    def _on_cloud(self, cloud):
        h, w = cloud.height, cloud.width
        if self._mask is None or self._mask.shape != (h, w):
            m = np.zeros((h, w), np.uint8)
            u0, v0, u1, v1 = self.roi
            m[int(v0 * h):int(v1 * h), int(u0 * w):int(u1 * w)] = self.class_id
            self._mask = m
            self.get_logger().info(f'掩码 {w}x{h}，标记 {int((m > 0).mean() * 100)}% 的像素')
        img = Image()
        img.header = cloud.header          # stamp 与 frame 都照抄，保证严格配对
        img.height, img.width = h, w
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = w
        img.data = self._mask.tobytes()
        self.pub.publish(img)


def main():
    rclpy.init()
    n = StubSegmentation()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
