#!/usr/bin/env python3
"""假 GNSS 发布器：在没有 GPS 信号（或室内）时喂给室外栈一个可用的 NavSatFix。

真驱动 unicore_gnss 收不到星时照样以 10Hz 发 /gps/fix，但内容是
``status=-1, lat=lon=0, covariance=[-1,...]``，下游全部判定"无解"：

* ``nav_node.is_gps_available()``  —— status < STATUS_FIX(0) 直接 False
* ``gps_heading_align``           —— min_fix_status / max_h_var 都过不了
* ``navsat_transform``            —— 不产出 /odometry/gps

本节点按真驱动的话题、frame_id、QoS(BEST_EFFORT/sensor_data) 原样发布一个
**有解**的 fix，因此对下游是即插即用的替换。用法见 scripts/start_fake_gps.sh。

⚠ 同一时刻只能有一个 /gps/fix 发布者。真驱动还在跑的话两边消息交替到达，
"取最新一帧"的判据（is_gps_available）会 true/false 抖动，比没有更糟。

两种模式：

* ``follow_odom=false``（默认）—— 死死钉在 (lat, lon)。够用于测模式切换、
  RPC、campus_nav 目标下发；但 gps_heading_align 要走够 min_total_distance
  (3m) 的 GPS 位移才锁航向，位置不动 => 永远不锁 => 没有 /odometry/gps，
  map->odom 停在 identity（见 map_odom_from_gps 的 publish_identity_until_gps）。
* ``follow_odom=true`` —— 把 LIO 的 /Odometry 平移换算成经纬度增量叠加到
  (lat, lon) 上，等于"GPS 完美跟随里程计"。这样航向能锁、navsat 有输出、
  整条室外链路都能跑通。代价是 GPS 不再是独立信息源（航向 offset 收敛到
  odom_yaw_offset 给的值），只适合联调，不能用来验证 GPS/LIO 融合精度。
  另注意 /Odometry 的 x/y 在 LIO 启动帧里，不是 ENU；开机朝向不是正北时用
  ``odom_yaw_offset``（rad，odom->ENU 的旋转）补偿。
"""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

# 与 gps_heading_align.py / map_odom_from_gps.py 保持同一个球半径，避免
# 两边经纬度<->米换算差出常数比例。
_R_EARTH = 6378137.0


class FakeGpsPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_gps_publisher")

        # 默认点：maps/scene_xili/scene.json 里的场景点之一。
        self.declare_parameter("lat", 22.57348767)
        self.declare_parameter("lon", 113.93442003)
        self.declare_parameter("alt", 30.0)

        self.declare_parameter("fix_topic", "/gps/fix")
        # 设成空字符串则不发速度（下游 navsat/heading_align 都不订阅它，
        # 发出来只是让话题图看起来跟真驱动一致）。
        self.declare_parameter("vel_topic", "/gps/vel")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("rate", 10.0)  # 真驱动 BESTNAVA 0.1 => 10Hz

        # NavSatStatus: -1 无解 / 0 单点 / 1 SBAS / 2 GBAS(RTK 固定)。
        # 默认 2，模拟 RTK 固定解。
        self.declare_parameter("status", int(NavSatStatus.STATUS_GBAS_FIX))
        # 水平/垂直 1σ (m)。gps_heading_align 的 max_h_var 是 25 m²（5m σ），
        # 这里给 0.05 m 表示 RTK 固定解，稳过门限。
        self.declare_parameter("sigma_h", 0.05)
        self.declare_parameter("sigma_v", 0.10)
        # 叠加高斯噪声的 1σ (m)，0 = 完全不动的理想值。
        self.declare_parameter("noise_sigma", 0.0)

        self.declare_parameter("follow_odom", False)
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("odom_yaw_offset", 0.0)  # odom->ENU 旋转 (rad)

        self.lat0 = float(self.get_parameter("lat").value)
        self.lon0 = float(self.get_parameter("lon").value)
        self.alt0 = float(self.get_parameter("alt").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.status = int(self.get_parameter("status").value)
        self.sigma_h = float(self.get_parameter("sigma_h").value)
        self.sigma_v = float(self.get_parameter("sigma_v").value)
        self.noise_sigma = float(self.get_parameter("noise_sigma").value)
        self.follow_odom = bool(self.get_parameter("follow_odom").value)
        self.odom_yaw_offset = float(self.get_parameter("odom_yaw_offset").value)

        fix_topic = str(self.get_parameter("fix_topic").value)
        vel_topic = str(self.get_parameter("vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        rate = max(float(self.get_parameter("rate").value), 0.1)

        self._cos_lat0 = math.cos(math.radians(self.lat0))
        self._odom = None  # (x, y, vx, vy) in odom frame

        # QoS 必须跟真驱动一致（sensor_data = BEST_EFFORT）：nav_node 的
        # /gps/fix 订阅是 BEST_EFFORT，RELIABLE 的发布者跟它匹配不上，一帧都收不到。
        self.pub_fix = self.create_publisher(
            NavSatFix, fix_topic, qos_profile_sensor_data)
        self.pub_vel = None
        if vel_topic:
            self.pub_vel = self.create_publisher(
                TwistWithCovarianceStamped, vel_topic, qos_profile_sensor_data)

        if self.follow_odom:
            self.create_subscription(
                Odometry, odom_topic, self._on_odom, 10)

        self.create_timer(1.0 / rate, self._on_timer)

        mode = f"follow_odom({odom_topic})" if self.follow_odom else "static"
        self.get_logger().warning(
            f"fake_gps_publisher【假 GPS，勿用于精度验证】mode={mode} "
            f"topic={fix_topic} rate={rate}Hz status={self.status} "
            f"origin=({self.lat0:.8f}, {self.lon0:.8f}, {self.alt0:.2f})")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )

    def _enu_offset(self):
        """当前相对 (lat0, lon0) 的 (east, north, v_east, v_north)，单位 m / m/s。"""
        if not self.follow_odom or self._odom is None:
            return 0.0, 0.0, 0.0, 0.0
        x, y, vx, vy = self._odom
        c = math.cos(self.odom_yaw_offset)
        s = math.sin(self.odom_yaw_offset)
        return (x * c - y * s, x * s + y * c,
                vx * c - vy * s, vx * s + vy * c)

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        east, north, v_east, v_north = self._enu_offset()
        if self.noise_sigma > 0.0:
            east += random.gauss(0.0, self.noise_sigma)
            north += random.gauss(0.0, self.noise_sigma)

        fix = NavSatFix()
        fix.header = Header(stamp=stamp, frame_id=self.frame_id)
        fix.status.status = self.status
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self.lat0 + math.degrees(north / _R_EARTH)
        fix.longitude = self.lon0 + math.degrees(
            east / (_R_EARTH * max(self._cos_lat0, 1e-9)))
        fix.altitude = self.alt0

        # 对角协方差，ENU 顺序（跟真驱动 _publish_fix 一致：[0]=East, [4]=North）。
        cov = [0.0] * 9
        cov[0] = self.sigma_h ** 2
        cov[4] = self.sigma_h ** 2
        cov[8] = self.sigma_v ** 2
        fix.position_covariance = cov
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        if self.pub_vel is not None:
            vel = TwistWithCovarianceStamped()
            vel.header = Header(stamp=stamp, frame_id=self.frame_id)
            vel.twist.twist.linear.x = v_east
            vel.twist.twist.linear.y = v_north
            vel.twist.twist.linear.z = 0.0
            vcov = [0.0] * 36
            vcov[0] = vcov[7] = 0.01
            vcov[14] = 0.04
            vel.twist.covariance = vcov
            self.pub_vel.publish(vel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGpsPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM（tmux kill-session、start_fake_gps.sh stop）走这里，
        # 别把正常停止打成 traceback。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
