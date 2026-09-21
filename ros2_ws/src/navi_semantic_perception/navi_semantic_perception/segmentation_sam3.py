"""Publish synchronized SAM3 masks, confidence and organized clouds for lawn_layer."""

import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2

from vision_msgs.msg import LabelInfo, VisionClass

import yaml

from .sam3_engine import Sam3Segmenter
from .sam3_paths import checkpoint_directory


def stamp_ns(header):
    """Keep exact nanosecond stamps for matching and publication."""
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def decode_rgb(msg):
    """Read RGB/BGR including row padding in a sensor_msgs/Image."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported color encoding: {msg.encoding}')
    if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * 3:
        raise ValueError('Invalid color dimensions/step')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    rgb = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        rgb = rgb[..., ::-1]
    return np.ascontiguousarray(rgb).copy()


def image_message(array, header, encoding='mono8'):
    """Preserve the source cloud header, including its acquisition timestamp."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = msg.width * (3 if encoding == 'rgb8' else 1)
    msg.data = np.ascontiguousarray(array, dtype=np.uint8).tobytes()
    return msg


class SegmentationSam3(Node):
    """Keep one inference in flight and select the latest available RGB/cloud pair."""

    def __init__(self, segmenter_factory=None, **kwargs):
        """Load SAM3 and initialize the synchronized publication pipeline."""
        super().__init__('segmentation_sam3', **kwargs)
        p = self.declare_parameter
        share = Path(get_package_share_directory('navi_semantic_perception'))
        backend = p('backend', 'pytorch').value
        if backend not in ('pytorch', 'tensorrt'):
            raise ValueError('SAM3 backend must be pytorch or tensorrt')
        if segmenter_factory is None:
            if backend == 'tensorrt':
                from .sam3_trt_engine import Sam3TRTSegmenter
                segmenter_factory = Sam3TRTSegmenter
            else:
                segmenter_factory = Sam3Segmenter
        default_model = checkpoint_directory()
        if backend == 'tensorrt':
            default_model = default_model / 'trt_engines'
        model_path = p('model_path', '').value or str(default_model)
        config_path = p('config_path', '').value or str(share / 'models/sam3')
        processor_path = p('processor_path', '').value or str(share / 'models/sam3/processor')
        ontology_path = Path(p('ontology_path', str(share / 'config/ontology.yaml')).value)
        with ontology_path.expanduser().open() as stream:
            classes = yaml.safe_load(stream)['ontology']['classes']
        self.class_names = ['background'] + [item['name'] for item in classes]
        if len(self.class_names) > 255 or len(set(self.class_names)) != len(self.class_names):
            raise ValueError('Ontology must contain unique class names and at most 254 classes')
        prompts = [(index, item.get('sam3_prompt', item['name']))
                   for index, item in enumerate(classes, 1)
                   if item.get('sam3_prompt', item['name']).strip()]
        self.palette = np.zeros((256, 3), np.uint8)
        for index, item in enumerate(classes, 1):
            self.palette[index] = list(reversed(item.get('color', [255, 255, 255])))
        color_topic = p('color_topic', '/camera1/camera1/color/image_raw').value
        cloud_topic = p('cloud_topic', '/semantic/camera1/points_raw').value
        output_cloud_topic = p('output_cloud_topic', '/semantic/camera1/points').value
        if self.resolve_topic_name(cloud_topic) == self.resolve_topic_name(output_cloud_topic):
            raise ValueError('SAM3 input/output cloud topics must differ to avoid a loop')
        self.width = int(p('color_width', 640).value)
        self.height = int(p('color_height', 480).value)
        self.max_pair_dt = float(p('max_pair_dt', 0.05).value)
        self.max_result_age = float(p('max_result_age', 2.0).value)
        self.color_alpha = float(p('color_viz_alpha', 0.5).value)
        if min(self.width, self.height, self.max_pair_dt, self.max_result_age) <= 0:
            raise ValueError('Image dimensions, pairing tolerance and result age must be positive')
        if not 0 <= self.color_alpha <= 1:
            raise ValueError('color_viz_alpha must be in 0..1')
        threshold = float(p('score_threshold', 0.5).value)
        self.get_logger().info(f'Loading local SAM3 ({backend}): {model_path}; prompts={prompts}')
        self.segmenter = segmenter_factory(
            model_path=model_path, processor_path=processor_path, prompts=prompts,
            config_path=config_path,
            device=p('device', 'cuda').value, precision=p('precision', 'bfloat16').value,
            score_threshold=threshold, mask_threshold=float(p('mask_threshold', 0.5).value),
        )
        self.pub_mask = self.create_publisher(
            Image, p('mask_topic', '/semantic/camera1/segmentation').value,
            qos_profile_sensor_data)
        self.pub_conf = self.create_publisher(
            Image, p('confidence_topic', '/semantic/camera1/confidence').value,
            qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(
            PointCloud2, output_cloud_topic, qos_profile_sensor_data)
        self.pub_color = None
        if p('publish_color_viz', True).value:
            self.pub_color = self.create_publisher(
                Image, p('color_viz_topic', '/semantic/camera1/segmentation_color').value, 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_label = self.create_publisher(
            LabelInfo, p('label_info_topic', '/semantic/camera1/label_info').value, latched)
        labels = LabelInfo()
        labels.header.stamp = self.get_clock().now().to_msg()
        labels.threshold = threshold
        labels.class_map = [VisionClass(class_id=index, class_name=name)
                            for index, name in enumerate(self.class_names)]
        self.pub_label.publish(labels)

        self.colors = {}
        self.latest_cloud = None
        self.future = None
        self.active_pair = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='sam3')
        self.n_infer = self.n_stale = self.n_nopair = 0
        self.total_time = 0.0
        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_timer(0.01, self._tick)
        self.create_timer(10.0, self._stats)
        self.get_logger().info(
            f'SAM3 ready: {color_topic} + {cloud_topic} -> {output_cloud_topic} + masks; '
            f'classes={dict(enumerate(self.class_names))}')

    def _on_color(self, msg):
        try:
            rgb = decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        if rgb.shape[:2] != (self.height, self.width):
            self.get_logger().error('Color image does not match color_width/height',
                                    throttle_duration_sec=5.0)
            return
        self.colors[stamp_ns(msg.header)] = rgb
        while len(self.colors) > 60:
            del self.colors[next(iter(self.colors))]

    def _on_cloud(self, cloud):
        if (cloud.width, cloud.height) != (self.width, self.height):
            self.get_logger().error('Cloud dimensions do not match the color image grid',
                                    throttle_duration_sec=5.0)
            return
        self.latest_cloud = (cloud, time.monotonic())

    def _has_consumers(self):
        return any(pub is not None and pub.get_subscription_count() > 0 for pub in (
            self.pub_mask, self.pub_conf, self.pub_cloud, self.pub_color))

    def _expired(self, cloud, received):
        # Check both wall time (paused bags) and sensor age (queued/delayed messages).
        sensor_age = (self.get_clock().now().nanoseconds - stamp_ns(cloud.header)) / 1e9
        return max(time.monotonic() - received, sensor_age) > self.max_result_age

    def _tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            cloud, rgb, received, started = self.active_pair
            try:
                cls, conf = self.future.result()
                if self._expired(cloud, received):
                    self.n_stale += 1
                else:
                    self._publish(cloud, rgb, cls, conf)
                    self.n_infer += 1
                    self.total_time += time.monotonic() - started
            except Exception as exc:
                self.get_logger().error(f'SAM3 frame dropped: {exc}', throttle_duration_sec=5.0)
            finally:
                self.future = self.active_pair = None
        if self.latest_cloud is None or not self._has_consumers():
            return
        cloud, received = self.latest_cloud
        if self._expired(cloud, received):
            self.latest_cloud = None
            self.n_stale += 1
            return
        if not self.colors:
            return
        stamp = stamp_ns(cloud.header)
        closest = min(self.colors, key=lambda key: abs(key - stamp))
        if abs(closest - stamp) > self.max_pair_dt * 1e9:
            # Allow color to arrive after depth, but never infer on an unmatched image.
            if time.monotonic() - received > self.max_pair_dt:
                self.latest_cloud = None
                self.n_nopair += 1
            return
        rgb = self.colors[closest]
        self.latest_cloud = None
        self.active_pair = (cloud, rgb, received, time.monotonic())
        self.future = self.worker.submit(self.segmenter.infer, rgb)

    def _publish(self, cloud, rgb, cls, conf):
        if (cls.shape != (cloud.height, cloud.width) or conf.shape != cls.shape
                or cls.dtype != np.uint8 or conf.dtype != np.uint8):
            raise ValueError('SAM3 output must be uint8 and match both cloud dimensions')
        self.pub_mask.publish(image_message(cls, cloud.header))
        self.pub_conf.publish(image_message(conf, cloud.header))
        # Publish the exact input cloud only once its masks are ready. Nav2's small
        # message_filters queue would otherwise evict it during slow inference.
        self.pub_cloud.publish(cloud)
        if self.pub_color is not None:
            overlay = rgb.copy()
            foreground = cls != 0
            overlay[foreground] = (
                rgb[foreground] * (1 - self.color_alpha)
                + self.palette[cls[foreground]] * self.color_alpha
            ).clip(0, 255).astype(np.uint8)
            self.pub_color.publish(image_message(overlay, cloud.header, 'rgb8'))

    def _stats(self):
        ms = self.total_time * 1000 / self.n_infer if self.n_infer else 0
        self.get_logger().info(
            f'SAM3: {self.n_infer} frames, mean {ms:.1f} ms; '
            f'stale={self.n_stale}, unpaired={self.n_nopair}')
        self.n_infer = self.n_stale = self.n_nopair = 0
        self.total_time = 0.0

    def destroy_node(self):
        """Finish the single worker before releasing model and ROS resources."""
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    """Run the SAM3 semantic perception node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationSam3()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
