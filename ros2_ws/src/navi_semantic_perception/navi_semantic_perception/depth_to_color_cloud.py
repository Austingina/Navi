#!/usr/bin/env python3
"""把 RealSense 深度图重采样到【彩色像素网格】，发 organized PointCloud2。

为什么需要它
------------
semantic_segmentation_layer 不做投影，它按索引硬关联（segmentation_buffer.cpp:203-206），
要求「点 N == 掩码像素 N」。而 D457 的 depth 与 color 是两套内参、两个光学系：
同索引关联实测偏 12~65 px（横向 6.4~16.5 cm = 1.3~3.3 个 0.05 m 代价图格）。

驱动侧无法解决（已核实源码）：
  - pointcloud_filter.cpp:139,147-152 点云网格永远取自 depth_intrin
  - base_realsense_node.cpp:260/263 点云滤波器排在 align 之前，align_depth 改不了它
  - base_realsense_node.cpp:903 frame_id 硬编码为 depth 光学系
  且 depth/color 都是 640x480，那一层只比 width*height 乘积的尺寸校验会通过 -> 静默错位。

投影数学在 warp.py（与 analysis/rgbd_align/check_align.py 共用同一份实现，
所以「离线验证通过」等价于「这个节点的数学已验证」）。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from .warp import DEPTH_SCALE, DIST_MODELS, DepthToColorWarp, rotation_to_matrix

# realsense2_camera_msgs 在 robot-sensors 工作区，不一定 source 了 -> 软导入
try:
    from realsense2_camera_msgs.msg import Extrinsics
    HAVE_RS_MSGS = True
except ImportError:                                        # pragma: no cover
    Extrinsics = None
    HAVE_RS_MSGS = False

# camera_info：驱动是 VOLATILE 发布的。订阅端若用 TRANSIENT_LOCAL 会 QoS 不兼容、
# 一条都收不到（实测踩过）。所以这里 VOLATILE —— 它对 TRANSIENT_LOCAL 的发布者也兼容。
INFO_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
# extrinsics：全程只发一条、latched。必须 TRANSIENT_LOCAL 才能拿到那条历史。
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DepthToColorCloud(Node):

    def __init__(self):
        super().__init__('depth_to_color_cloud')
        p = self.declare_parameter
        ns = p('camera_ns', '/camera1/camera1').value
        self.depth_topic = p('depth_topic', f'{ns}/depth/image_rect_raw').value
        self.dinfo_topic = p('depth_info_topic', f'{ns}/depth/camera_info').value
        self.cinfo_topic = p('color_info_topic', f'{ns}/color/camera_info').value
        self.ext_topic = p('extrinsics_topic', f'{ns}/extrinsics/depth_to_color').value
        self.out_topic = p('output_topic', '/semantic/camera1/points').value
        self.out_frame = p('output_frame', 'camera1_color_optical_frame').value
        # 外参来源：'topic' 用工厂标定的 Extrinsics（离线验证用的就是它）；
        # 'tf' 用 driver 发的 static TF（两者实测差约 10um）。bag 回放时通常只有 topic。
        self.ext_source = p('extrinsics_source', 'topic').value
        self.target_rate = float(p('target_rate', 10.0).value)
        self.min_range = float(p('min_range', 0.30).value)
        self.max_range = float(p('max_range', 8.0).value)
        # 暂定 none：静止 bag 上 warped 峰位 none=-0.3px vs plumb_bob=+3.5px，
        # 但那段场景只有 2 个 Z 档有样本、四角无内容，分辨力不足 -> 待更好场景复核。
        self.dist_model = p('color_distortion_model', 'none').value
        self.splat = p('splat', 'nearest').value
        self.assert_color_size = list(p('assert_color_size', [640, 480]).value)
        self.nan_warn_lo = float(p('nan_fraction_warn_low', 0.05).value)
        self.nan_warn_hi = float(p('nan_fraction_warn_high', 0.70).value)
        if self.dist_model not in DIST_MODELS:
            raise ValueError(f'color_distortion_model must be one of {DIST_MODELS}')

        self._dinfo = None
        self._cinfo = None
        self._ext = None          # (R_dc, t_dc)
        self._warp = None
        self._last_pub_t = None
        self._n_pub = 0
        self._n_drop_rate = 0

        self.pub = self.create_publisher(PointCloud2, self.out_topic,
                                         rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.dinfo_topic, self._on_dinfo, INFO_QOS)
        self.create_subscription(CameraInfo, self.cinfo_topic, self._on_cinfo, INFO_QOS)
        if self.ext_source == 'topic':
            if not HAVE_RS_MSGS:
                raise RuntimeError(
                    'extrinsics_source=topic 需要 realsense2_camera_msgs；请 source '
                    'robot-sensors 的 install，或把 extrinsics_source 设成 tf')
            self.create_subscription(Extrinsics, self.ext_topic, self._on_ext, LATCHED)
        else:
            import tf2_ros
            self._tfbuf = tf2_ros.Buffer()
            self._tfl = tf2_ros.TransformListener(self._tfbuf, self)
        self.create_subscription(Image, self.depth_topic, self._on_depth,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_timer(10.0, self._stats)
        self.get_logger().info(
            f'depth_to_color_cloud: {self.depth_topic} -> {self.out_topic} '
            f'({self.out_frame}), {self.target_rate:.1f} Hz, range [{self.min_range},'
            f'{self.max_range}] m, distortion={self.dist_model}, ext={self.ext_source}')

    # ---- 标定输入 ---------------------------------------------------------
    def _on_dinfo(self, m):
        if self._dinfo is None:
            self._dinfo = m
            self._try_build()

    def _on_cinfo(self, m):
        if self._cinfo is None:
            self._cinfo = m
            self._try_build()

    def _on_ext(self, m):
        if self._ext is None:
            self._ext = (rotation_to_matrix(m.rotation), np.asarray(m.translation, float))
            self.get_logger().info(
                f'depth->color 外参(工厂标定): t={np.round(self._ext[1], 6).tolist()} m, '
                f'基线 {np.linalg.norm(self._ext[1]) * 1000:.2f} mm')
            self._try_build()

    def _ext_from_tf(self):
        from rclpy.time import Time
        from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
        try:
            tr = self._tfbuf.lookup_transform(
                self._cinfo.header.frame_id, self._dinfo.header.frame_id, Time())
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None
        q = tr.transform.rotation
        x, y, zq, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + zq * zq), 2 * (x * y - zq * w), 2 * (x * zq + y * w)],
            [2 * (x * y + zq * w), 1 - 2 * (x * x + zq * zq), 2 * (y * zq - x * w)],
            [2 * (x * zq - y * w), 2 * (y * zq + x * w), 1 - 2 * (x * x + y * y)]])
        t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                      tr.transform.translation.z])
        return R, t

    def _try_build(self):
        if self._warp is not None or self._dinfo is None or self._cinfo is None:
            return
        if self._ext is None:
            if self.ext_source == 'topic':
                return
            self._ext = self._ext_from_tf()
            if self._ext is None:
                return
        cw, ch = self._cinfo.width, self._cinfo.height
        if self.assert_color_size and [cw, ch] != self.assert_color_size:
            raise RuntimeError(
                f'彩色分辨率 {cw}x{ch} != assert_color_size {self.assert_color_size}。'
                'SAM3 的掩码必须与它逐像素一致，尺寸不符会让那一层只比乘积的校验蒙过去。')
        R, t = self._ext
        self._warp = DepthToColorWarp(
            np.asarray(self._dinfo.k).reshape(3, 3), np.asarray(self._cinfo.k).reshape(3, 3),
            R, t, (self._dinfo.width, self._dinfo.height), (cw, ch),
            Dc=np.asarray(self._cinfo.d), min_range=self.min_range,
            max_range=self.max_range, distortion_model=self.dist_model, splat=self.splat)
        self.get_logger().info(
            f'warp 就绪: depth {self._dinfo.width}x{self._dinfo.height}'
            f'({self._dinfo.header.frame_id}) -> color {cw}x{ch}({self._cinfo.header.frame_id})')

    # ---- 主回调 -----------------------------------------------------------
    def _on_depth(self, msg):
        if self._warp is None:
            self._try_build()
            return
        if self.pub.get_subscription_count() == 0:      # 没人订阅就不算
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_pub_t is not None and (t - self._last_pub_t) < 1.0 / self.target_rate:
            self._n_drop_rate += 1
            return
        if msg.encoding != '16UC1':
            self.get_logger().error(f'深度编码 {msg.encoding}，只支持 16UC1', throttle_duration_sec=5.0)
            return
        d16 = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        xyz, st = self._warp.warp(d16)
        if not (self.nan_warn_lo <= st['nan_frac'] <= self.nan_warn_hi):
            self.get_logger().warning(
                f"nan_fraction={st['nan_frac']:.3f} 超出 [{self.nan_warn_lo},{self.nan_warn_hi}]"
                '（视野全是天空/镜头被挡/IR 发射器关了？稳态应约 0.41）',
                throttle_duration_sec=5.0)
        out = PointCloud2()
        out.header = msg.header                 # stamp 逐位照抄深度头
        out.header.frame_id = self.out_frame
        out.height, out.width = xyz.shape[0], xyz.shape[1]
        out.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                      for n, o in (('x', 0), ('y', 4), ('z', 8))]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = False                    # NaN 占位 -> 必须是 False
        out.data = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
        self.pub.publish(out)
        self._last_pub_t = t
        self._n_pub += 1
        self._last_stats = st

    def _stats(self):
        if self._n_pub == 0:
            return
        st = getattr(self, '_last_stats', {})
        self.get_logger().info(
            f"发布 {self._n_pub} 帧（限速丢 {self._n_drop_rate}）| 有效源点 "
            f"{st.get('src_valid_frac', 0):.3f} 填充 {st.get('fill_frac', 0):.3f} "
            f"NaN {st.get('nan_frac', 0):.3f} z-buffer 碰撞 {st.get('collision_frac', 0):.4f}")
        self._n_pub = self._n_drop_rate = 0


def main():
    rclpy.init()
    n = DepthToColorCloud()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
