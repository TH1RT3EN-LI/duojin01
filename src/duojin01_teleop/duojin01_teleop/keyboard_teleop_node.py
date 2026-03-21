#!/usr/bin/env python3
import fcntl
import glob
import os
import select
import struct
import threading
import time

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

from duojin01_teleop.keyboard_teleop_core import KeyboardTeleopCore

MSG = """
当前按键布局：
   Q    W    E
   A    S    D
        X

W/S : 前进/后退 (线速度 x)
A/D : 左移/右移 (线速度 y)
Q/E : 左转/右转 (角速度 z)
X   : 紧急停止

I/K : 线速度 增加/减少（固定步长）
O/L : 角速度 增加/减少（固定步长）

速度变化采用平滑加减速
方向键可同时组合使用

CTRL-C 退出
"""

EV_KEY = 0x01
KEY_LEFTCTRL = 29
KEY_RIGHTCTRL = 97
KEY_C = 46

KEY_CODE_TO_CHAR = {
    16: 'q',
    17: 'w',
    18: 'e',
    23: 'i',
    24: 'o',
    30: 'a',
    31: 's',
    32: 'd',
    37: 'k',
    38: 'l',
    45: 'x',
}

EVIOCGRAB = 0x40044590
ACCEL_LIMIT_LINEAR = 2.0
DECEL_LIMIT_LINEAR = 3.0
ACCEL_LIMIT_ANGULAR = 6.0
DECEL_LIMIT_ANGULAR = 8.0
IDLE_TIMEOUT_SEC = 0.0


def vels(linear_speed, angular_speed):
    return '当前速度:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (linear_speed, angular_speed)


class CmdVelPublisherThread(threading.Thread):
    def __init__(self, node, publisher, twist_msg, twist, stamped, core, rate_hz, stop_event):
        super().__init__(daemon=True)
        self.node = node
        self.publisher = publisher
        self.twist_msg = twist_msg
        self.twist = twist
        self.stamped = stamped
        self.core = core
        self.stop_event = stop_event
        self.period = 1.0 / rate_hz if rate_hz > 0.0 else 0.01

    def _publish_command(self, command):
        if self.stamped:
            self.twist_msg.header.stamp = self.node.get_clock().now().to_msg()

        self.twist.linear.x = command.linear_x
        self.twist.linear.y = command.linear_y
        self.twist.linear.z = command.linear_z
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = command.angular_z
        self.publisher.publish(self.twist_msg)

    def run(self):
        while rclpy.ok() and not self.stop_event.is_set():
            self._publish_command(self.core.snapshot())
            self.stop_event.wait(self.period)

        self.core.emergency_stop()
        self._publish_command(self.core.zero_command())


def discover_evdev_keyboard_devices(explicit_device):
    if explicit_device:
        if os.path.exists(explicit_device):
            return [explicit_device]
        return []

    devices = sorted(glob.glob('/dev/input/by-path/*-event-kbd'))
    if devices:
        return devices

    fallback = []
    try:
        with open('/proc/bus/input/devices', 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line.startswith('H: Handlers='):
                    continue
                handlers = line.split('=', 1)[1].split()
                if 'kbd' not in handlers:
                    continue
                for handler in handlers:
                    if handler.startswith('event'):
                        fallback.append(f'/dev/input/{handler}')
    except OSError:
        return []
    return sorted(set(fallback))


class EvdevKeyboardThread(threading.Thread):
    def __init__(self, node, core, stop_event, device_paths, grab_device, poll_timeout):
        super().__init__(daemon=True)
        self.node = node
        self.core = core
        self.stop_event = stop_event
        self.device_paths = device_paths
        self.grab_device = bool(grab_device)
        self.poll_timeout = max(0.001, float(poll_timeout))

    def run(self):
        event_format = 'llHHI'
        event_size = struct.calcsize(event_format)
        fd_to_path = {}
        buffers = {}
        ctrl_pressed = False

        try:
            for path in self.device_paths:
                fd = None
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    if self.grab_device:
                        fcntl.ioctl(fd, EVIOCGRAB, 1)
                except OSError:
                    try:
                        if fd is not None:
                            os.close(fd)
                    except OSError:
                        pass
                    continue
                fd_to_path[fd] = path
                buffers[fd] = b''

            if not fd_to_path:
                self.stop_event.set()
                return

            while rclpy.ok() and not self.stop_event.is_set():
                ready_fds, _, _ = select.select(list(fd_to_path.keys()), [], [], self.poll_timeout)
                if not ready_fds:
                    continue

                now = time.monotonic()
                for fd in ready_fds:
                    try:
                        chunk = os.read(fd, event_size * 128)
                    except BlockingIOError:
                        continue
                    except OSError:
                        continue

                    if not chunk:
                        continue

                    buffer = buffers[fd] + chunk
                    event_count = len(buffer) // event_size
                    offset = 0

                    for _ in range(event_count):
                        _, _, event_type, code, value = struct.unpack_from(event_format, buffer, offset)
                        offset += event_size

                        if event_type != EV_KEY:
                            continue

                        if code in (KEY_LEFTCTRL, KEY_RIGHTCTRL):
                            ctrl_pressed = value in (1, 2)
                            continue

                        if code == KEY_C and value == 1 and ctrl_pressed:
                            self.stop_event.set()
                            break

                        key = KEY_CODE_TO_CHAR.get(code)
                        if key is None:
                            continue

                        if self.core.is_move_key(key):
                            if value == 0:
                                self.core.handle_key_release(key, now)
                            elif value in (1, 2):
                                self.core.handle_key_press(key, now)
                        elif self.core.is_speed_key(key) and value == 1:
                            self.core.handle_key_press(key, now)
                            status = self.core.status()
                            print(vels(status.linear_speed, status.angular_speed))

                    buffers[fd] = buffer[offset:]
                    if self.stop_event.is_set():
                        break
        except Exception as exception:
            self.node.get_logger().error(f'Evdev keyboard thread failed: {exception}')
        finally:
            self.core.clear_move_keys(time.monotonic())
            for fd in fd_to_path:
                try:
                    if self.grab_device:
                        fcntl.ioctl(fd, EVIOCGRAB, 0)
                except OSError:
                    pass
                try:
                    os.close(fd)
                except OSError:
                    pass


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop_node')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
    speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value
    speed_step = node.declare_parameter('speed_step', 0.05, read_only_descriptor).value
    turn_step = node.declare_parameter('turn_step', 0.1, read_only_descriptor).value
    cmd_vel_topic = node.declare_parameter('cmd_vel_topic', '/cmd_vel', read_only_descriptor).value
    publish_rate = node.declare_parameter('publish_rate', 100.0, read_only_descriptor).value
    evdev_device = node.declare_parameter('evdev_device', '', read_only_descriptor).value
    grab_device = node.declare_parameter('grab_device', True, read_only_descriptor).value
    evdev_poll_timeout = node.declare_parameter('evdev_poll_timeout', 0.005, read_only_descriptor).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        twist_msg_type = geometry_msgs.msg.TwistStamped
    else:
        twist_msg_type = geometry_msgs.msg.Twist

    publisher = node.create_publisher(twist_msg_type, cmd_vel_topic, 10)
    twist_msg = twist_msg_type()
    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    device_paths = discover_evdev_keyboard_devices(str(evdev_device))
    readable_devices = [path for path in device_paths if os.access(path, os.R_OK)]
    if not readable_devices:
        node.get_logger().error('No readable keyboard evdev device found. Please set evdev_device.')
        node.destroy_node()
        rclpy.shutdown()
        return

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    stop_event = threading.Event()
    core = KeyboardTeleopCore(
        linear_speed=float(speed),
        angular_speed=float(turn),
        speed_step=float(speed_step),
        turn_step=float(turn_step),
        accel_limit_linear=ACCEL_LIMIT_LINEAR,
        decel_limit_linear=DECEL_LIMIT_LINEAR,
        accel_limit_angular=ACCEL_LIMIT_ANGULAR,
        decel_limit_angular=DECEL_LIMIT_ANGULAR,
        idle_timeout_sec=IDLE_TIMEOUT_SEC,
    )

    publisher_thread = CmdVelPublisherThread(
        node=node,
        publisher=publisher,
        twist_msg=twist_msg,
        twist=twist,
        stamped=stamped,
        core=core,
        rate_hz=float(publish_rate),
        stop_event=stop_event,
    )

    keyboard_thread = EvdevKeyboardThread(
        node=node,
        core=core,
        stop_event=stop_event,
        device_paths=readable_devices,
        grab_device=grab_device,
        poll_timeout=evdev_poll_timeout,
    )

    try:
        print(MSG)
        print('平滑限制:\t线加速 %.2f m/s²\t线减速 %.2f m/s²\t角加速 %.2f rad/s²\t角减速 %.2f rad/s²' % (
            ACCEL_LIMIT_LINEAR,
            DECEL_LIMIT_LINEAR,
            ACCEL_LIMIT_ANGULAR,
            DECEL_LIMIT_ANGULAR,
        ))
        print(vels(float(speed), float(turn)))
        publisher_thread.start()
        keyboard_thread.start()

        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        core.emergency_stop()
        if keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
        if publisher_thread.is_alive():
            publisher_thread.join(timeout=1.0)
        rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == '__main__':
    main()
