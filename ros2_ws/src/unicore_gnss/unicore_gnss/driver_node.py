#!/usr/bin/env python3
"""ROS 2 serial driver for Unicore N4 BESTNAVA.

Reads ``/dev/ttyUSB0`` @ 115200 by default, optionally sends LOG commands to
enable BESTNAVA, and publishes:

* ``/gps/fix`` — sensor_msgs/NavSatFix (WGS84 ellipsoid height)
* ``/gps/vel`` — geometry_msgs/TwistWithCovarianceStamped (ENU m/s)
* ``/unicore/bestnava`` — std_msgs/String (raw ASCII sentence)
* ``/unicore/bestnava_info`` — std_msgs/String (human-readable summary)
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, String

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required: sudo apt install python3-serial"
    ) from exc

from unicore_gnss.bestnava import BestNavA, extract_ascii_messages, parse_bestnava


class UnicoreGnssNode(Node):
    def __init__(self) -> None:
        super().__init__("unicore_gnss")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("fix_topic", "/gps/fix")
        self.declare_parameter("vel_topic", "/gps/vel")
        self.declare_parameter("raw_topic", "/unicore/bestnava")
        self.declare_parameter("info_topic", "/unicore/bestnava_info")
        self.declare_parameter("check_crc", True)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("read_timeout_s", 0.2)
        # 手册 abbreviated ASCII（§7.3.26）。空列表 = 不配置。
        self.declare_parameter(
            "config_commands",
            [
                "UNLOG",
                "BESTNAVA 0.1",
            ],
        )
        self.declare_parameter("config_command_delay_s", 0.1)
        self.declare_parameter("use_gps_time", False)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.check_crc = bool(self.get_parameter("check_crc").value)
        self.reconnect_period_s = float(self.get_parameter("reconnect_period_s").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.config_commands: List[str] = list(
            self.get_parameter("config_commands").value
        )
        self.config_command_delay_s = float(
            self.get_parameter("config_command_delay_s").value
        )
        self.use_gps_time = bool(self.get_parameter("use_gps_time").value)

        fix_topic = str(self.get_parameter("fix_topic").value)
        vel_topic = str(self.get_parameter("vel_topic").value)
        raw_topic = str(self.get_parameter("raw_topic").value)
        info_topic = str(self.get_parameter("info_topic").value)

        self.pub_fix = self.create_publisher(
            NavSatFix, fix_topic, qos_profile_sensor_data
        )
        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, vel_topic, qos_profile_sensor_data
        )
        self.pub_raw = self.create_publisher(String, raw_topic, 10)
        self.pub_info = self.create_publisher(String, info_topic, 10)

        self._ser: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Unicore BESTNAVA driver: port={self.port} baud={self.baudrate} "
            f"fix={fix_topic}"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._close_serial()
        return super().destroy_node()

    def _close_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout_s,
                write_timeout=1.0,
            )
            self._ser.reset_input_buffer()
            self._send_config_commands()
            self.get_logger().info(f"Opened {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as exc:
            self.get_logger().warn(f"Cannot open {self.port}: {exc}")
            self._close_serial()
            return False

    def _send_config_commands(self) -> None:
        if self._ser is None or not self.config_commands:
            return
        for cmd in self.config_commands:
            cmd = str(cmd).strip()
            if not cmd:
                continue
            line = (cmd if cmd.endswith("\r\n") else cmd + "\r\n").encode("ascii")
            try:
                self._ser.write(line)
                self._ser.flush()
                self.get_logger().info(f"Sent: {cmd}")
            except serial.SerialException as exc:
                self.get_logger().error(f"Failed to send '{cmd}': {exc}")
                return
            time.sleep(self.config_command_delay_s)

    def _io_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            if self._ser is None or not self._ser.is_open:
                if not self._open_serial():
                    self._stop.wait(self.reconnect_period_s)
                    continue
            try:
                chunk = self._ser.read(1024)
                if chunk:
                    self._buffer.extend(chunk)
                    messages, self._buffer = extract_ascii_messages(self._buffer)
                    for msg in messages:
                        self._handle_message(msg)
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial error: {exc}; reconnecting")
                self._close_serial()
                self._buffer.clear()
                self._stop.wait(self.reconnect_period_s)

    def _handle_message(self, message: str) -> None:
        upper = message.upper()
        if not upper.startswith("#BESTNAVA"):
            return

        parsed = parse_bestnava(message, check_crc=self.check_crc)
        if parsed is None:
            self.get_logger().warn("BESTNAVA CRC/parse failed")
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_raw(message)
        self._publish_info(parsed)
        self._publish_fix(parsed, stamp)
        self._publish_vel(parsed, stamp)

    def _publish_raw(self, message: str) -> None:
        msg = String()
        msg.data = message.replace("\r", "").replace("\n", "")
        self.pub_raw.publish(msg)

    def _publish_info(self, nav: BestNavA) -> None:
        msg = String()
        msg.data = (
            f"sol={nav.sol_status}/{nav.pos_type} "
            f"lat={nav.lat_deg:.8f} lon={nav.lon_deg:.8f} "
            f"alt_ell={nav.height_ellipsoid_m:.3f} "
            f"sv={nav.num_soln_svs}/{nav.num_svs} "
            f"std=[{nav.lat_std_m:.3f},{nav.lon_std_m:.3f},{nav.hgt_std_m:.3f}] "
            f"vel={nav.hor_spd_mps:.3f}m/s track={nav.track_ground_deg:.1f}deg "
            f"week={nav.header.week} tow_ms={nav.header.ms}"
        )
        self.pub_info.publish(msg)

    def _publish_fix(self, nav: BestNavA, stamp) -> None:
        msg = NavSatFix()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.status.status = nav.navsat_status_fix()
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(nav.lat_deg)
        msg.longitude = float(nav.lon_deg)
        msg.altitude = float(nav.height_ellipsoid_m)

        # Diagonal covariance in ENU: lat/lon/alt std^2; unknown off-diagonals = 0.
        cov = [0.0] * 9
        lat_var = max(nav.lat_std_m, 0.0) ** 2
        lon_var = max(nav.lon_std_m, 0.0) ** 2
        alt_var = max(nav.hgt_std_m, 0.0) ** 2
        if not nav.has_position or math.isnan(lat_var):
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            cov[0] = cov[4] = cov[8] = -1.0
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            cov[0] = lon_var  # East
            cov[4] = lat_var  # North
            cov[8] = alt_var  # Up
        msg.position_covariance = cov
        self.pub_fix.publish(msg)

    def _publish_vel(self, nav: BestNavA, stamp) -> None:
        east, north, up = nav.east_north_up_velocity()
        msg = TwistWithCovarianceStamped()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        if math.isnan(east):
            return
        msg.twist.twist.linear.x = east
        msg.twist.twist.linear.y = north
        msg.twist.twist.linear.z = up
        cov = [0.0] * 36
        hs = max(nav.hor_spd_std_mps, 0.0) ** 2
        vs = max(nav.vert_spd_std_mps, 0.0) ** 2
        # Approximate ENU linear covariance from horizontal/vertical speed std.
        cov[0] = hs  # vx (east)
        cov[7] = hs  # vy (north)
        cov[14] = vs  # vz (up)
        msg.twist.covariance = cov
        self.pub_vel.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnicoreGnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
