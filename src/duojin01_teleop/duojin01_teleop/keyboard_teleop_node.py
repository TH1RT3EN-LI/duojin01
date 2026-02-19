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

无按键输入时自动停止（速度归零）
方向键可同时组合使用

CTRL-C 退出
"""

MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
    'x': (0.0, 0.0, 0.0, 0.0),
}

SPEED_BINDINGS = {
    'i': (1.0, 0.0),
    'k': (-1.0, 0.0),
    'o': (0.0, 1.0),
    'l': (0.0, -1.0),
}

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


def vels(speed, turn):
    return '当前速度:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (speed, turn)


class SharedState:
    def __init__(self, speed, turn, speed_step, turn_step):
        self.lock = threading.Lock()
        self.pressed_move_keys = set()
        self.speed = float(speed)
        self.turn = float(turn)
        self.speed_step = max(0.0, float(speed_step))
        self.turn_step = max(0.0, float(turn_step))

    def update_move_key(self, key, pressed):
        if key not in MOVE_BINDINGS:
            return

        with self.lock:
            if key == 'x':
                if pressed:
                    self.pressed_move_keys.clear()
                return

            if pressed:
                self.pressed_move_keys.add(key)
            else:
                self.pressed_move_keys.discard(key)

    def update_speed_key(self, key):
        if key not in SPEED_BINDINGS:
            return

        linear_step_direction, angular_step_direction = SPEED_BINDINGS[key]
        with self.lock:
            self.speed = max(0.0, self.speed + linear_step_direction * self.speed_step)
            self.turn = max(0.0, self.turn + angular_step_direction * self.turn_step)
            speed = self.speed
            turn = self.turn
        print(vels(speed, turn))

    def snapshot(self):
        with self.lock:
            x = 0.0
            y = 0.0
            z = 0.0
            th = 0.0
            for key in self.pressed_move_keys:
                dx, dy, dz, dth = MOVE_BINDINGS[key]
                x += dx
                y += dy
                z += dz
                th += dth

            x = max(-1.0, min(1.0, x))
            y = max(-1.0, min(1.0, y))
            z = max(-1.0, min(1.0, z))
            th = max(-1.0, min(1.0, th))
            return x, y, z, th, self.speed, self.turn

    def clear_moves(self):
        with self.lock:
            self.pressed_move_keys.clear()


class CmdVelPublisherThread(threading.Thread):
    def __init__(self, node, publisher, twist_msg, twist, stamped, state, rate_hz, stop_event):
        super().__init__(daemon=True)
        self.node = node
        self.publisher = publisher
        self.twist_msg = twist_msg
        self.twist = twist
        self.stamped = stamped
        self.state = state
        self.stop_event = stop_event
        self.period = 1.0 / rate_hz if rate_hz > 0.0 else 0.01

    def publish_snapshot(self):
        x, y, z, th, speed, turn = self.state.snapshot()
        if self.stamped:
            self.twist_msg.header.stamp = self.node.get_clock().now().to_msg()

        self.twist.linear.x = x * speed
        self.twist.linear.y = y * speed
        self.twist.linear.z = z * speed
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = th * turn
        self.publisher.publish(self.twist_msg)

    def publish_zero(self):
        if self.stamped:
            self.twist_msg.header.stamp = self.node.get_clock().now().to_msg()

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist_msg)

    def run(self):
        while rclpy.ok() and not self.stop_event.is_set():
            self.publish_snapshot()
            self.stop_event.wait(self.period)
        self.publish_zero()


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
    def __init__(self, state, stop_event, device_paths, grab_device, poll_timeout):
        super().__init__(daemon=True)
        self.state = state
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
                    except Exception:
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

                        if key in MOVE_BINDINGS:
                            if value == 0:
                                self.state.update_move_key(key, pressed=False)
                            elif value in (1, 2):
                                self.state.update_move_key(key, pressed=True)
                        elif key in SPEED_BINDINGS and value == 1:
                            self.state.update_speed_key(key)

                    buffers[fd] = buffer[offset:]
                    if self.stop_event.is_set():
                        break
        finally:
            self.state.clear_moves()
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
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    publisher = node.create_publisher(TwistMsg, cmd_vel_topic, 10)
    twist_msg = TwistMsg()
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
    state = SharedState(speed=speed, turn=turn, speed_step=speed_step, turn_step=turn_step)

    publisher_thread = CmdVelPublisherThread(
        node=node,
        publisher=publisher,
        twist_msg=twist_msg,
        twist=twist,
        stamped=stamped,
        state=state,
        rate_hz=float(publish_rate),
        stop_event=stop_event,
    )

    keyboard_thread = EvdevKeyboardThread(
        state=state,
        stop_event=stop_event,
        device_paths=readable_devices,
        grab_device=grab_device,
        poll_timeout=evdev_poll_timeout,
    )

    try:
        print(MSG)
        print('调速步长:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (speed_step, turn_step))
        print(vels(speed, turn))
        publisher_thread.start()
        keyboard_thread.start()

        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
        if publisher_thread.is_alive():
            publisher_thread.join(timeout=1.0)
        rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == '__main__':
    main()
