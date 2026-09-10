#!/usr/bin/env python3
"""TensorRT 语义分割节点：彩色图 -> mono8 类别掩码 + confidence。

后端是 TensorRT（见 trt_engine.py 顶部关于 sm_110 的说明），预处理/推理/后处理
全在 GPU 上，主机侧只上传 uint8 RGB、下载两张 uint8 图。

对 semantic_segmentation_layer 的契约（README:88「Must be pixel-aligned and
time-synchronized with segmentation」）由两条保证兑现：

1. **像素对齐**：掩码尺寸恰好等于 `depth_to_color_cloud` 发的组织化点云的
   width×height，且同为原始 `color/image_raw` 网格。input_scale<1 时用**最近邻**
   缩回（类别 id 不可插值）。那一层只比 `width*height` 的乘积
   （semantic_segmentation_layer.cpp:495），尺寸错位会被静默放过，所以这里显式断言。
2. **时间同步**：本节点由**点云触发**，发布时 `header` 逐位照抄点云的 header，
   所以掩码与点云的 stamp 严格 1:1，那 100 ms 硬编码容差永远不会成为约束。
"""
import os
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import LabelInfo, VisionClass

from navi_semantic_perception.trt_engine import TrtSegmenter

# LabelInfo 必须 latched：那一层只在启动时订阅一次，晚到的类别表等于整层静默失效
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamp_key(h):
    return (h.stamp.sec, h.stamp.nanosec)


def _stamp_sec(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def img_from(a, header):
    """mono8 Image，header 逐位照抄传入的 header（点云的），保证 stamp 1:1。"""
    m = Image()
    m.header = header
    m.height, m.width = a.shape
    m.encoding = 'mono8'
    m.is_bigendian = 0
    m.step = a.shape[1]
    m.data = np.ascontiguousarray(a).tobytes()
    return m


class SegmentationTrt(Node):

    def __init__(self):
        super().__init__('segmentation_trt')
        p = self.declare_parameter
        share = ''
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('navi_semantic_perception')
        except Exception:
            pass
        self.model_path = p('model_path', os.path.join(share, 'models', 'model.onnx')).value
        self.onto_path = p('ontology_path', os.path.join(share, 'config', 'ontology.yaml')).value
        self.color_topic = p('color_topic', '/camera1/camera1/color/image_raw').value
        self.cloud_topic = p('cloud_topic', '/semantic/camera1/points').value
        self.mask_topic = p('mask_topic', '/semantic/camera1/segmentation').value
        self.conf_topic = p('confidence_topic', '/semantic/camera1/confidence').value
        self.label_topic = p('label_info_topic', '/semantic/camera1/label_info').value
        # 彩色原网格尺寸。引擎是固定形状的，必须在启动时就知道；运行期再对点云断言。
        self.W = int(p('color_width', 640).value)
        self.H = int(p('color_height', 480).value)
        # 推理输入缩放。1.0 = 原生 640x480，掩码天然落在彩色网格上，无需任何缩放。
        # <1 会走最近邻缩小再缩回，只在原生跑不到目标帧率时才用。
        self.scale = float(p('input_scale', 1.0).value)
        # 引擎缓存缺失时是否现编（几分钟）。false 则要求先跑 build_trt_engine。
        self.allow_build = bool(p('allow_engine_build', True).value)
        self.engine_path = p('engine_path', '').value or None
        # 彩色帧与点云 stamp 的最大配对间隔。实测同一 frameset 的 color/depth 差
        # 中位 0.036 ms、最大 33 ms，所以 50 ms 足够宽。
        self.max_pair_dt = float(p('max_pair_dt', 0.05).value)
        self.publish_confidence = bool(p('publish_confidence', True).value)
        # 彩色可视化。mask 本身是 mono8 的类别 id(0/1/2)，在 RViz 里是全黑的 ——
        # 它是给那一层用的，不是给人看的。这个话题才是用来看的。
        # 刻意用 RELIABLE 发：RViz2 的 Image display 默认 Reliability=Reliable，
        # 而 mask/points 走 SensorDataQoS(BEST_EFFORT)，默认设置下 RViz 收不到。
        self.publish_color = bool(p('publish_color_viz', True).value)
        self.color_viz_topic = p('color_viz_topic',
                                 '/semantic/camera1/segmentation_color').value
        self.color_alpha = float(p('color_viz_alpha', 0.5).value)
        # 暗场 fail-safe。实测（analysis/lawn_layer/brightness_corr.py，240 帧）：
        # 近场条带平均亮度与"路面被误判成 grass 的比例"相关系数 r = -0.850。
        #   亮度 60-80  : 误判中位 54.9%，100% 的帧 >20%
        #   亮度 80-100 : 误判中位 21.4%，59% 的帧 >20%
        #   亮度 100-120: 误判中位  0.0%，只有 3% 的帧 >20%
        # 也就是说高架桥阴影下的湿暗石板会被整片刷成草坪；以 base_cost=100 /
        # max_cost=253 / samples_to_max_cost=3，三帧内车必须走的路面就变成 253。
        # 置信度阈值【分不开】这两者（坏帧上要 240 才压住，而真草坪 conf 的 p75 才 231），
        # 亮度可以。所以太暗时直接发全 background 掩码 —— 保持与点云 1:1 的
        # stamp 契约，但让这一层什么都不标，宁可漏检不可误标。
        self.min_brightness = float(p('min_mean_brightness', 100.0).value)
        # 亮度在哪一段行上量：底部条带对应车前约 0.4~2 m，正是这一层的作用区。
        self.brightness_rows = int(p('brightness_rows', 96).value)

        with open(self.onto_path) as f:
            onto = yaml.safe_load(f)
        # 约定：0 恒为 background，随后按 ontology 里的顺序 1..N
        self.class_names = ['background'] + [c['name'] for c in onto['ontology']['classes']]
        # ontology 里的颜色本来就是"给掩码可视化用的"（上游注释原话）。BGR -> RGB。
        pal = [(0, 0, 0)] + [tuple(reversed(c.get('color', [255, 255, 255])))
                             for c in onto['ontology']['classes']]
        self._palette = np.zeros((256, 3), np.uint8)
        for i, c in enumerate(pal[:256]):
            self._palette[i] = c

        self.iw = max(1, int(round(self.W * self.scale)))
        self.ih = max(1, int(round(self.H * self.scale)))
        self.seg = TrtSegmenter(self.model_path, self.iw, self.ih,
                                engine_path=self.engine_path,
                                allow_build=self.allow_build,
                                log=self.get_logger().info)
        if self.seg.num_classes != len(self.class_names):
            self.get_logger().error(
                f'模型输出 {self.seg.num_classes} 类，ontology 给了 '
                f'{len(self.class_names)} 类 {self.class_names} —— 类别 id 会错位')
        # 缩放索引表预计算（scale==1 时全不用）
        if (self.iw, self.ih) != (self.W, self.H):
            self._yi = (np.arange(self.ih) * (self.H / self.ih)).astype(np.int32)
            self._xi = (np.arange(self.iw) * (self.W / self.iw)).astype(np.int32)
            self._yb = (np.arange(self.H) * (self.ih / self.H)).astype(np.int32)
            self._xb = (np.arange(self.W) * (self.iw / self.W)).astype(np.int32)

        self.color = {}          # stamp_key -> (t_sec, ndarray HxWx3 uint8 RGB)
        self.n_infer = 0
        self.n_nopair = 0
        self.n_dark = 0
        self.t_sum = 0.0
        self._blank = np.zeros((self.H, self.W), np.uint8)

        self.pub_mask = self.create_publisher(Image, self.mask_topic,
                                              rclpy.qos.qos_profile_sensor_data)
        self.pub_conf = self.create_publisher(Image, self.conf_topic,
                                              rclpy.qos.qos_profile_sensor_data) \
            if self.publish_confidence else None
        self.pub_color = self.create_publisher(Image, self.color_viz_topic, 1) \
            if self.publish_color else None      # depth=1 RELIABLE，RViz 默认即可看到
        self.pub_label = self.create_publisher(LabelInfo, self.label_topic, LATCHED)
        self._publish_labels()
        self.create_timer(5.0, self._publish_labels)   # 兜底补发
        self.create_subscription(Image, self.color_topic, self._on_color,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_timer(10.0, self._stats)
        self.get_logger().info(
            f'segmentation_trt: {self.color_topic} + {self.cloud_topic} -> {self.mask_topic}\n'
            f'  推理 {self.iw}x{self.ih} (input_scale={self.scale}) -> '
            f'输出 {self.W}x{self.H}\n'
            f'  类别 {dict(enumerate(self.class_names))}')

    def _publish_labels(self):
        m = LabelInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera1_color_optical_frame'
        m.threshold = 0.5
        for i, nm in enumerate(self.class_names):
            vc = VisionClass()
            vc.class_id = i          # 必须 0..254：buffer 存进 uint8_t，>255 静默截断
            vc.class_name = nm       # 必须与层 YAML 的 classes 逐字节相同
            m.class_map.append(vc)
        self.pub_label.publish(m)

    def _on_color(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().error(f'彩色编码 {msg.encoding}，只支持 rgb8/bgr8',
                                    throttle_duration_sec=5.0)
            return
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            a = a[..., ::-1]
        self.color[_stamp_key(msg.header)] = (_stamp_sec(msg.header), a.copy())
        if len(self.color) > 60:                      # 只留最近 ~2 s
            for k in sorted(self.color, key=lambda k: self.color[k][0])[:len(self.color) - 60]:
                del self.color[k]

    def _on_cloud(self, cloud):
        # 没人要就不烧 GPU。必须把【所有】输出都算进来 —— 只看 mask 的话，
        # 在 RViz 里单看 segmentation_color 会得到"什么都不发"，很难查。
        if (self.pub_mask.get_subscription_count() == 0
                and (self.pub_color is None or self.pub_color.get_subscription_count() == 0)
                and (self.pub_conf is None or self.pub_conf.get_subscription_count() == 0)):
            return
        t = _stamp_sec(cloud.header)
        if not self.color:
            self.n_nopair += 1
            return
        k = min(self.color, key=lambda k: abs(self.color[k][0] - t))
        if abs(self.color[k][0] - t) > self.max_pair_dt:
            self.n_nopair += 1
            return
        rgb = self.color[k][1]
        if (cloud.width, cloud.height) != (self.W, self.H):
            self.get_logger().error(
                f'点云 {cloud.width}x{cloud.height} 与本节点的 {self.W}x{self.H} 不一致 —— '
                '那一层只比 width*height 的乘积，错位会被静默放过；改 color_width/height',
                throttle_duration_sec=5.0)
            return
        if rgb.shape[:2] != (self.H, self.W):
            self.get_logger().error(
                f'彩色 {rgb.shape[1]}x{rgb.shape[0]} 与 {self.W}x{self.H} 不一致',
                throttle_duration_sec=5.0)
            return

        t0 = time.perf_counter()
        if self.min_brightness > 0:
            strip = rgb[self.H - self.brightness_rows:]
            if float(strip.mean()) < self.min_brightness:
                self.n_dark += 1                     # 暗场：发全 background，不标记
                self.pub_mask.publish(img_from(self._blank, cloud.header))
                if self.pub_conf is not None:
                    self.pub_conf.publish(img_from(self._blank, cloud.header))
                if self.pub_color is not None:      # 暗场也要发，否则 RViz 画面像卡住
                    self.pub_color.publish(self._color_viz(rgb, self._blank, cloud.header))
                return
        small = rgb if self.scale == 1.0 else rgb[self._yi][:, self._xi]
        cls, conf = self.seg.infer(np.ascontiguousarray(small))
        if self.scale != 1.0:                          # 最近邻缩回：类别 id 不可插值
            cls = cls[self._yb][:, self._xb]
            conf = conf[self._yb][:, self._xb]
        self.t_sum += time.perf_counter() - t0
        self.n_infer += 1

        self.pub_mask.publish(img_from(cls, cloud.header))
        if self.pub_conf is not None:
            self.pub_conf.publish(img_from(conf, cloud.header))
        if self.pub_color is not None:
            self.pub_color.publish(self._color_viz(rgb, cls, cloud.header))

    def _color_viz(self, rgb, cls, header):
        """rgb8 叠加图：原图 + 类别颜色。只为人眼，下游不消费。"""
        col = self._palette[cls]
        a = self.color_alpha
        out = (rgb.astype(np.float32) * (1.0 - a) + col.astype(np.float32) * a)
        m = Image()
        m.header = header
        m.height, m.width = cls.shape
        m.encoding = 'rgb8'
        m.is_bigendian = 0
        m.step = cls.shape[1] * 3
        m.data = np.ascontiguousarray(out.clip(0, 255).astype(np.uint8)).tobytes()
        return m

    def _stats(self):
        if self.n_infer == 0:
            if self.n_nopair:
                self.get_logger().warning(f'{self.n_nopair} 帧点云配不到彩色帧'
                                          f'（|Δstamp| > {self.max_pair_dt}s）')
                self.n_nopair = 0
            return
        self.get_logger().info(
            f'推理 {self.n_infer} 帧，均 {self.t_sum / self.n_infer * 1000:.1f} ms '
            f'({self.n_infer / self.t_sum:.1f} Hz 上限)，配不上 {self.n_nopair} 帧'
            + (f'，暗场跳过 {self.n_dark} 帧（近场亮度 < {self.min_brightness}）'
               if self.n_dark else ''))
        self.n_infer = self.n_nopair = self.n_dark = 0
        self.t_sum = 0.0


def main():
    rclpy.init()
    n = SegmentationTrt()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
