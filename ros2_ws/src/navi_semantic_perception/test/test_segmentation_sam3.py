"""Exercise ROS frame contracts and slow inference without loading a GPU model."""

from threading import Event
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, PointCloud2

from navi_semantic_perception.segmentation_sam3 import (
    SegmentationSam3, decode_rgb, image_message, stamp_ns,
)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class ControlledSegmenter:
    def __init__(self, **kwargs):
        self.started = Event()
        self.release = Event()
        self.inputs = []

    def infer(self, rgb):
        self.inputs.append(rgb)
        self.started.set()
        assert self.release.wait(3), 'Test did not release inference'
        return np.full((2, 3), 2, np.uint8), np.full((2, 3), 204, np.uint8)


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context, domain_id=168)
    instance = SegmentationSam3(
        segmenter_factory=ControlledSegmenter, context=context,
        parameter_overrides=[Parameter('color_width', value=3),
                             Parameter('color_height', value=2)],
    )
    for name in ('pub_cloud', 'pub_mask', 'pub_conf', 'pub_color'):
        setattr(instance, name, Publisher())
    try:
        yield instance
    finally:
        instance.segmenter.release.set()
        instance.destroy_node()
        context.shutdown()


def pair(node, value=10):
    cloud = PointCloud2(height=2, width=3)
    cloud.header.stamp = node.get_clock().now().to_msg()
    cloud.header.frame_id = 'camera1_color_optical_frame'
    image = image_message(np.full((2, 3, 3), value, np.uint8), cloud.header, 'rgb8')
    return cloud, image


def finish(node):
    node.segmenter.release.set()
    node.future.result(timeout=3)
    node._tick()


def test_slow_inference_publishes_exact_cloud_and_masks_together(node):
    cloud, image = pair(node)
    node._on_cloud(cloud)  # Color can arrive after depth.
    node._tick()
    assert node.future is None
    node._on_color(image)
    node._tick()
    assert node.segmenter.started.wait(1)
    assert not node.pub_cloud.messages
    finish(node)
    assert node.pub_cloud.messages == [cloud]
    for pub in (node.pub_mask, node.pub_conf, node.pub_color):
        msg = pub.messages[0]
        assert msg.header == cloud.header
        assert (msg.height, msg.width) == (2, 3)
    assert set(node.pub_mask.messages[0].data) == {2}
    assert set(node.pub_conf.messages[0].data) == {204}


def test_busy_worker_skips_superseded_frames(node):
    first, image = pair(node, 10)
    node._on_color(image)
    node._on_cloud(first)
    node._tick()
    assert node.segmenter.started.wait(1)
    for color in (20, 30):
        latest, image = pair(node, color)
        node._on_color(image)
        node._on_cloud(latest)
        node._tick()
    finish(node)
    finish(node)
    assert len(node.segmenter.inputs) == 2
    assert node.segmenter.inputs[-1][0, 0, 0] == 30
    assert [stamp_ns(msg.header) for msg in node.pub_cloud.messages] == [
        stamp_ns(first.header), stamp_ns(latest.header)]


def test_expired_result_is_not_published(node):
    cloud, image = pair(node)
    node._on_color(image)
    node._on_cloud(cloud)
    node._tick()
    node.active_pair = (cloud, decode_rgb(image), time.monotonic() - 3, time.monotonic())
    finish(node)
    assert not node.pub_cloud.messages
    assert not node.pub_mask.messages
    assert node.n_stale == 1


def test_out_of_tolerance_color_and_transposed_cloud_do_not_infer(node):
    cloud, image = pair(node)
    # Give color its own header before shifting its acquisition timestamp.
    image.header = type(cloud.header)(stamp=type(cloud.header.stamp)(
        sec=cloud.header.stamp.sec - 1, nanosec=cloud.header.stamp.nanosec))
    node._on_color(image)
    node._on_cloud(cloud)
    node._tick()
    assert node.future is None
    node.latest_cloud = None
    cloud.width, cloud.height = 2, 3  # Same product, wrong pixel layout.
    node._on_cloud(cloud)
    assert node.latest_cloud is None


def test_worker_failure_drops_entire_triple_and_recovers(node):
    def fail(rgb):
        raise RuntimeError('inference failed')

    infer = node.segmenter.infer
    node.segmenter.infer = fail
    cloud, image = pair(node)
    node._on_color(image)
    node._on_cloud(cloud)
    node._tick()
    with pytest.raises(RuntimeError, match='inference failed'):
        node.future.result(timeout=3)
    node._tick()
    assert node.future is None
    assert not node.pub_cloud.messages
    assert not node.pub_mask.messages
    node.segmenter.infer = infer
    cloud, image = pair(node)
    node._on_color(image)
    node._on_cloud(cloud)
    node._tick()
    finish(node)
    assert node.pub_cloud.messages == [cloud]


def test_padded_bgr_image_is_decoded_without_padding():
    msg = Image(height=2, width=1, encoding='bgr8', step=4,
                data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
    np.testing.assert_array_equal(decode_rgb(msg), [[[3, 2, 1]], [[6, 5, 4]]])


@pytest.mark.parametrize('backend', ['pytorch', 'tensorrt'])
def test_backend_selects_factory_and_default_asset_directory(monkeypatch, backend):
    from navi_semantic_perception import sam3_trt_engine, segmentation_sam3

    captured = {}

    class Capture(ControlledSegmenter):
        def __init__(self, **kwargs):
            captured.update(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(segmentation_sam3, 'Sam3Segmenter', Capture)
    monkeypatch.setattr(sam3_trt_engine, 'Sam3TRTSegmenter', Capture)
    monkeypatch.setenv('SAM3_CHECKPOINT_DIR', '/tmp/navi-test-checkpoint')
    context = Context()
    rclpy.init(context=context, domain_id=168)
    instance = None
    try:
        instance = SegmentationSam3(
            context=context, parameter_overrides=[Parameter('backend', value=backend)])
        expected = '/tmp/navi-test-checkpoint'
        if backend == 'tensorrt':
            expected += '/trt_engines'
        assert captured['model_path'] == expected
        assert captured['prompts'] == [(2, 'grass')]
    finally:
        if instance is not None:
            instance.destroy_node()
        context.shutdown()
