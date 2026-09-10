#!/usr/bin/env python3
"""发 latched 的 vision_msgs/LabelInfo（class_id <-> class_name 映射）。

semantic_segmentation_layer 硬要求：LabelInfo 必须在第一帧图像/点云【之前】到，
否则每帧都被丢（semantic_segmentation_layer.cpp:511-516 "Class map is empty"）。
所以本节点要在 nav2 之前起。

两个静默失效的坑：
  1. class_id 必须 0..254 —— 消息字段是 uint16，但那一层存进 uint8_t
     (segmentation_buffer.cpp:113)，>255 静默截断。
  2. class_name 必须与 costmap yaml 里 <class_type>.classes 逐字节相同。不匹配时
     class_map 非空（不触发 exit(-1)）但 class_to_id_map 为空 -> 整层零效果、无报错。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import LabelInfo, VisionClass

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LabelInfoPublisher(Node):

    def __init__(self):
        super().__init__('label_info_publisher')
        ids = list(self.declare_parameter('class_ids', [0, 1]).value)
        names = list(self.declare_parameter('class_names', ['background', 'lawn']).value)
        thr = float(self.declare_parameter('threshold', 0.5).value)
        topic = self.declare_parameter('topic', '/semantic/camera1/label_info').value
        period = float(self.declare_parameter('repeat_period_s', 5.0).value)
        frame = self.declare_parameter('frame_id', 'camera1_color_optical_frame').value
        if len(ids) != len(names):
            raise ValueError(f'class_ids({len(ids)}) 与 class_names({len(names)}) 长度不一致')
        bad = [i for i in ids if not 0 <= int(i) <= 254]
        if bad:
            raise ValueError(f'class_id 必须在 0..254（那一层存 uint8_t，会静默截断）：{bad}')

        self.msg = LabelInfo()
        self.msg.header.frame_id = frame
        self.msg.threshold = thr
        for i, nm in zip(ids, names):
            vc = VisionClass()
            vc.class_id = int(i)
            vc.class_name = str(nm)
            self.msg.class_map.append(vc)

        self.pub = self.create_publisher(LabelInfo, topic, LATCHED)
        self._publish()
        # latched 已足够（transient_local 会在订阅者上线时补发），补发只是为了
        # 覆盖「本节点比 nav2 后起」和 costmap reset 的情形，代价近乎为零。
        if period > 0:
            self.create_timer(period, self._publish)
        self.get_logger().info(
            f'LabelInfo -> {topic} (latched): ' +
            ', '.join(f'{v.class_id}:{v.class_name}' for v in self.msg.class_map) +
            f'  threshold={thr}')

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    n = LabelInfoPublisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
