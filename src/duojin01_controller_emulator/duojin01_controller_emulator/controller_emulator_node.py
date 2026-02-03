import math
import os
import threading
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
SEND_DATA_SIZE = 11
RECEIVE_DATA_SIZE = 24

GYROSCOPE_RATIO = 0.00026644  # rad/s per raw unit
ACCEL_RATIO = 1671.84  # raw per m/s^2 (base driver divides raw by this)
G_REF = 9.80665  # m/s^2


def _xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def _decode_i16_be(high: int, low: int) -> int:
    v = (high << 8) | low
    if v & 0x8000:
        v -= 0x10000
    return v


def _encode_i16_be(v: int) -> tuple[int, int]:
    v = int(v)
    if v < 0:
        v = (1 << 16) + v
    return (v >> 8) & 0xFF, v & 0xFF


def _saturate_i16(v: float) -> int:
    if not math.isfinite(v):
        return 0
    i = int(round(v))
    if i > 32767:
        return 32767
    if i < -32768:
        return -32768
    return i


@dataclass
class _Twist2D:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp_monotonic: float = 0.0


class Duojin01ControllerEmulator(Node):
    """
    模拟车辆底盘的MCU并创建虚拟串口供仿真使用，以共用并测试duojin01_base_driver包

    Data flow (driver-equivalent sim):
      ROS /cmd_vel -> duojin01_base_driver(pkg) -> PTY (Duojin01 protocol) -> this node
      this node -> ROS /cmd_vel_sim -> ros_gz_bridge -> Gazebo
      Gazebo /sim/odom -> this node -> PTY feedback frames -> duojin01_base_driver -> /odom + /imu
    """

    def __init__(self) -> None:
        super().__init__("duojin01_controller_emulator")

        self.declare_parameter("pty_link_path", "/tmp/duojin01_controller")
        self.declare_parameter("odom_topic", "/sim/odom")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_sim")

        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("cmd_pub_hz", 50.0)
        self.declare_parameter("feedback_hz", 200.0)

        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_vy", 0.6)
        self.declare_parameter("max_wz", 2.0)

        self.declare_parameter("acc_lim_xy", 2.0)
        self.declare_parameter("acc_lim_wz", 3.2)

        self.declare_parameter("battery_voltage", 24.0) 

        self._pty_link_path = str(self.get_parameter("pty_link_path").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)

        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._cmd_pub_hz = float(self.get_parameter("cmd_pub_hz").value)
        self._feedback_hz = float(self.get_parameter("feedback_hz").value)

        self._max_vx = float(self.get_parameter("max_vx").value)
        self._max_vy = float(self.get_parameter("max_vy").value)
        self._max_wz = float(self.get_parameter("max_wz").value)

        self._acc_lim_xy = float(self.get_parameter("acc_lim_xy").value)
        self._acc_lim_wz = float(self.get_parameter("acc_lim_wz").value)

        self._battery_voltage = float(self.get_parameter("battery_voltage").value)

        self._master_fd: Optional[int] = None
        self._slave_fd: Optional[int] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_thread_stop = threading.Event()

        self._cmd_lock = threading.Lock()
        self._target_cmd = _Twist2D()
        self._applied_cmd = _Twist2D()
        self._applied_last_mono = time.monotonic()

        self._odom_lock = threading.Lock()
        self._last_odom_twist: Optional[_Twist2D] = None

        self._make_pty_link()

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 20)

        self.create_timer(1.0 / max(1.0, self._cmd_pub_hz), self._on_cmd_timer)
        self.create_timer(1.0 / max(1.0, self._feedback_hz), self._on_feedback_timer)

        self.get_logger().info(
            f"controller emulator started: pty_link_path='{self._pty_link_path}', "
            f"odom_topic='{self._odom_topic}', cmd_vel_out_topic='{self._cmd_vel_out_topic}'"
        )

    def _make_pty_link(self) -> None:
        master_fd, slave_fd = os.openpty()
        tty.setraw(slave_fd)
        slave_name = os.ttyname(slave_fd)

        link_dir = os.path.dirname(self._pty_link_path) or "."
        os.makedirs(link_dir, exist_ok=True)

        try:
            if os.path.islink(self._pty_link_path) or os.path.exists(self._pty_link_path):
                os.remove(self._pty_link_path)
        except OSError as exc:
            raise RuntimeError(f"failed to remove existing pty_link_path '{self._pty_link_path}': {exc}") from exc

        os.symlink(slave_name, self._pty_link_path)

        self._master_fd = master_fd
        self._slave_fd = slave_fd

        self.get_logger().info(f"PTY ready: slave='{slave_name}' -> link='{self._pty_link_path}'")

        self._rx_thread = threading.Thread(target=self._rx_loop, name="duojin01_pty_rx", daemon=True)
        self._rx_thread.start()

    def destroy_node(self) -> bool:
        self._rx_thread_stop.set()
        try:
            if self._master_fd is not None:
                os.close(self._master_fd)
        except OSError:
            pass
        try:
            if self._slave_fd is not None:
                os.close(self._slave_fd)
        except OSError:
            pass
        try:
            if os.path.islink(self._pty_link_path):
                os.remove(self._pty_link_path)
        except OSError:
            pass
        return super().destroy_node()

    def _rx_loop(self) -> None:
        assert self._master_fd is not None
        buf = bytearray()

        while rclpy.ok() and not self._rx_thread_stop.is_set():
            try:
                data = os.read(self._master_fd, 1024)
            except OSError:
                time.sleep(0.01)
                continue

            if not data:
                time.sleep(0.001)
                continue

            buf.extend(data)

            while True:
                try:
                    start = buf.index(FRAME_HEADER)
                except ValueError:
                    buf.clear()
                    break

                if len(buf) - start < SEND_DATA_SIZE:
                    if start > 0:
                        del buf[:start]
                    break

                frame = bytes(buf[start : start + SEND_DATA_SIZE])
                del buf[: start + SEND_DATA_SIZE]

                if frame[-1] != FRAME_TAIL:
                    continue

                if _xor_checksum(frame[:9]) != frame[9]:
                    continue

                vx = _decode_i16_be(frame[3], frame[4]) * 0.001
                vy = _decode_i16_be(frame[5], frame[6]) * 0.001
                wz = _decode_i16_be(frame[7], frame[8]) * 0.001

                now_m = time.monotonic()
                with self._cmd_lock:
                    self._target_cmd = _Twist2D(vx=vx, vy=vy, wz=wz, stamp_monotonic=now_m)

    def _odom_cb(self, msg: Odometry) -> None:
        t = msg.twist.twist
        with self._odom_lock:
            self._last_odom_twist = _Twist2D(
                vx=float(t.linear.x),
                vy=float(t.linear.y),
                wz=float(t.angular.z),
                stamp_monotonic=time.monotonic(),
            )

    def _apply_limits(self, current: float, target: float, max_delta: float) -> float:
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def _on_cmd_timer(self) -> None:
        now_m = time.monotonic()
        dt = now_m - self._applied_last_mono
        self._applied_last_mono = now_m

        with self._cmd_lock:
            target = self._target_cmd

        if (now_m - target.stamp_monotonic) > self._cmd_timeout:
            target = _Twist2D()

        target_vx = max(-self._max_vx, min(self._max_vx, target.vx))
        target_vy = max(-self._max_vy, min(self._max_vy, target.vy))
        target_wz = max(-self._max_wz, min(self._max_wz, target.wz))

        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        max_dv = self._acc_lim_xy * dt
        max_dw = self._acc_lim_wz * dt

        self._applied_cmd.vx = self._apply_limits(self._applied_cmd.vx, target_vx, max_dv)
        self._applied_cmd.vy = self._apply_limits(self._applied_cmd.vy, target_vy, max_dv)
        self._applied_cmd.wz = self._apply_limits(self._applied_cmd.wz, target_wz, max_dw)

        msg = Twist()
        msg.linear.x = float(self._applied_cmd.vx)
        msg.linear.y = float(self._applied_cmd.vy)
        msg.angular.z = float(self._applied_cmd.wz)
        self._cmd_pub.publish(msg)

    def _build_feedback_frame(self) -> bytes:
        with self._odom_lock:
            odom = self._last_odom_twist

        if odom is None or (time.monotonic() - odom.stamp_monotonic) > 0.5:
            vx = self._applied_cmd.vx
            vy = self._applied_cmd.vy
            wz = self._applied_cmd.wz
        else:
            vx = odom.vx
            vy = odom.vy
            wz = odom.wz

        rx = bytearray(RECEIVE_DATA_SIZE)
        rx[0] = FRAME_HEADER
        rx[1] = 0  # i think the  dbd  didn't use this one

        vx_mmps = _saturate_i16(vx * 1000.0)
        vy_mmps = _saturate_i16(vy * 1000.0)
        wz_mradps = _saturate_i16(wz * 1000.0)

        rx[2], rx[3] = _encode_i16_be(vx_mmps)
        rx[4], rx[5] = _encode_i16_be(vy_mmps)
        rx[6], rx[7] = _encode_i16_be(wz_mradps)

        ax_raw = 0
        ay_raw = 0
        az_raw = _saturate_i16(G_REF * ACCEL_RATIO)

        gx_raw = 0
        gy_raw = 0
        gz_raw = _saturate_i16(wz / GYROSCOPE_RATIO)

        rx[8], rx[9] = _encode_i16_be(ax_raw)
        rx[10], rx[11] = _encode_i16_be(ay_raw)
        rx[12], rx[13] = _encode_i16_be(az_raw)
        rx[14], rx[15] = _encode_i16_be(gx_raw)
        rx[16], rx[17] = _encode_i16_be(gy_raw)
        rx[18], rx[19] = _encode_i16_be(gz_raw)

        v_mv = _saturate_i16(self._battery_voltage * 1000.0)
        rx[20], rx[21] = _encode_i16_be(v_mv)

        rx[22] = _xor_checksum(bytes(rx[:22]))
        rx[23] = FRAME_TAIL
        return bytes(rx)

    def _on_feedback_timer(self) -> None:
        if self._master_fd is None:
            return
        try:
            os.write(self._master_fd, self._build_feedback_frame())
        except OSError:
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Duojin01ControllerEmulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
