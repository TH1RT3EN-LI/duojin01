#!/usr/bin/env python3
import os
import select
import struct
import sys
import termios
import threading
import time
import tty

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

from duojin01_teleop.keyboard_teleop_core import KeyboardTeleopCore
from duojin01_teleop.keyboard_teleop_defaults import (
    ACCEL_LIMIT_ANGULAR,
    ACCEL_LIMIT_LINEAR,
    DECEL_LIMIT_ANGULAR,
    DECEL_LIMIT_LINEAR,
    DEFAULT_ANGULAR_SPEED,
    DEFAULT_ANGULAR_SPEED_STEP,
    DEFAULT_CMD_VEL_TOPIC,
    DEFAULT_FRAME_ID,
    DEFAULT_IDLE_TIMEOUT_SEC,
    DEFAULT_LINEAR_SPEED,
    DEFAULT_LINEAR_SPEED_STEP,
    DEFAULT_MAX_ANGULAR_SPEED,
    DEFAULT_MAX_LINEAR_SPEED,
    DEFAULT_PUBLISH_RATE,
    DEFAULT_READ_POLL_TIMEOUT_SEC,
    DEFAULT_REPEAT_TIMEOUT_SEC,
    DEFAULT_SAVE_MAP_COMMAND,
    DEFAULT_SAVE_MAP_COOLDOWN_SEC,
    DEFAULT_SAVE_MAP_KEY,
    DEFAULT_STAMPED,
    DEFAULT_TTY_DEVICE_PATH,
    DEFAULT_KEYBOARD_BACKEND,
    DEFAULT_EVENT_DEVICE_PATH,
)
from duojin01_teleop.shell_command_hotkey import ShellCommandHotkey

MSG = """键位: W/S A/D Q/E X
调速: I/K O/L M
CTRL-C 退出"""

IDLE_TIMEOUT_SEC = DEFAULT_IDLE_TIMEOUT_SEC
SUPPORTED_KEYBOARD_BACKENDS = {'tty', 'event', 'tk'}
LINUX_INPUT_EVENT_STRUCT = struct.Struct('llHHi')
EV_KEY = 0x01
KEY_EVENT_RELEASE = 0
KEY_EVENT_PRESS = 1
KEY_EVENT_REPEAT = 2
LINUX_KEY_CODE_BINDINGS = {
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
    50: 'm',
}


def vels(linear_speed, angular_speed):
    return '当前速度:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (linear_speed, angular_speed)


def normalize_keyboard_backend(raw_backend: str) -> str:
    backend = str(raw_backend).strip().lower()
    if backend not in SUPPORTED_KEYBOARD_BACKENDS:
        raise RuntimeError(
            f"Unsupported keyboard_backend '{raw_backend}', expected one of {sorted(SUPPORTED_KEYBOARD_BACKENDS)}"
        )
    return backend


def resolve_tty_device_path(explicit_device_path: str) -> str:
    candidates = []
    if explicit_device_path:
        candidates.append(explicit_device_path)

    try:
        candidates.append(os.ttyname(0))
    except OSError:
        pass

    candidates.append('/dev/tty')

    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)

        fd = -1
        try:
            fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
            if os.isatty(fd):
                return candidate
        except OSError:
            continue
        finally:
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass

    raise RuntimeError('No interactive tty available')


def resolve_event_device_path(explicit_device_path: str) -> str:
    event_device_path = str(explicit_device_path).strip()
    if not event_device_path:
        raise RuntimeError("keyboard_backend='event' 时必须设置 event_device_path")

    if not os.path.exists(event_device_path):
        raise RuntimeError(f'键盘事件设备不存在: {event_device_path}')

    if not os.access(event_device_path, os.R_OK):
        raise RuntimeError(f'没有权限读取键盘事件设备: {event_device_path}')

    return event_device_path


class ConsoleStatusDisplay:
    def __init__(self):
        self._lock = threading.Lock()
        self._status_text = ''

    def update_status(self, text: str) -> None:
        status_text = str(text)
        with self._lock:
            if status_text == self._status_text:
                return
            self._status_text = status_text
            sys.stdout.write(f'{status_text}\n')
            sys.stdout.flush()

    def update_speed_status(self, linear_speed: float, angular_speed: float) -> None:
        self.update_status('速度: %.2f / %.2f' % (float(linear_speed), float(angular_speed)))

    def print_event(self, text: str) -> None:
        with self._lock:
            sys.stdout.write(f'{text}\n')
            sys.stdout.flush()

    def clear_status(self) -> None:
        with self._lock:
            self._status_text = ''

    def update_motion(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        return


class TkStatusDisplay:
    def __init__(self, root, status_var, motion_var, message_var):
        self.root = root
        self.status_var = status_var
        self.motion_var = motion_var
        self.message_var = message_var
        self._status_text = ''
        self._lock = threading.Lock()
        self._closed = False
        self._main_thread_id = threading.get_ident()

    def close(self) -> None:
        with self._lock:
            self._closed = True

    def update_status(self, text: str) -> None:
        status_text = str(text)
        with self._lock:
            self._status_text = status_text

        def apply_update():
            if self._closed or not self.status_var:
                return
            self.status_var.set(status_text)

        self._call_on_gui_thread(apply_update)

    def update_speed_status(self, linear_speed: float, angular_speed: float) -> None:
        status_text = '档位: 线 %.2f m/s  角 %.2f rad/s' % (
            float(linear_speed),
            float(angular_speed),
        )
        self.update_status(status_text)

    def print_event(self, text: str) -> None:
        message_text = str(text)

        def apply_update():
            if self._closed or not self.message_var:
                return
            self.message_var.set(message_text)

        self._call_on_gui_thread(apply_update)

    def clear_status(self) -> None:
        with self._lock:
            self._status_text = ''

        def apply_update():
            if self._closed or not self.status_var:
                return
            self.status_var.set('')

        self._call_on_gui_thread(apply_update)

    def update_motion(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        motion_text = '当前方向速度: x=%.2f m/s  y=%.2f m/s  w=%.2f rad/s' % (
            float(linear_x),
            float(linear_y),
            float(angular_z),
        )

        def apply_update():
            if self._closed or not self.motion_var:
                return
            self.motion_var.set(motion_text)

        self._call_on_gui_thread(apply_update)

    def _call_on_gui_thread(self, callback) -> None:
        if self._closed:
            return

        if threading.get_ident() == self._main_thread_id:
            callback()
            return

        self.root.after(0, callback)


class TkKeyboardWindow:
    def __init__(self, core, stop_event):
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exception:
            raise RuntimeError(f'无法导入 tkinter: {exception}') from exception

        self._tk = tk
        self._ttk = ttk
        self.core = core
        self.stop_event = stop_event
        self.command_handler = None
        self._closed = False

        self.root = tk.Tk()
        self.root.title('duojin01 键盘遥控')
        self.root.geometry('520x360')
        self.root.minsize(500, 340)

        self.status_var = tk.StringVar(value='档位: --')
        self.motion_var = tk.StringVar(value='方向: x=0.00 m/s  y=0.00 m/s  w=0.00 rad/s')
        self.message_var = tk.StringVar(value='M 保存地图')
        self.status_display = TkStatusDisplay(self.root, self.status_var, self.motion_var, self.message_var)

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<KeyRelease>', self._on_key_release)
        self.root.bind_all('<Escape>', self._on_escape)
        self.root.after(100, self._poll_stop)

        self._build_ui()
        self.root.after(0, self.root.focus_force)

    def bind_command_handler(self, command_handler) -> None:
        self.command_handler = command_handler
        status = self.core.status()
        self.status_display.update_speed_status(status.linear_speed, status.angular_speed)
        self.status_display.update_motion(0.0, 0.0, 0.0)

    def run(self) -> None:
        self.root.mainloop()

    def _build_ui(self) -> None:
        container = self._ttk.Frame(self.root, padding=12)
        container.pack(fill='both', expand=True)

        header = self._ttk.Frame(container)
        header.pack(fill='x', pady=(0, 10))
        self._ttk.Label(header, text='键盘遥控', font=('TkDefaultFont', 16, 'bold')).pack(side='left')
        self._ttk.Label(header, text='M 保存地图', anchor='e').pack(side='right')

        control_frame = self._ttk.Frame(container)
        control_frame.pack(fill='x', pady=(0, 10))

        self._build_motion_pad(control_frame)
        self._build_speed_pad(control_frame)

        status_frame = self._ttk.LabelFrame(container, text='状态', padding=10)
        status_frame.pack(fill='x', pady=(0, 10))
        self._ttk.Label(status_frame, textvariable=self.status_var, anchor='w').pack(fill='x', pady=(0, 2))
        self._ttk.Label(status_frame, textvariable=self.motion_var, anchor='w').pack(fill='x', pady=(0, 2))
        self._ttk.Label(status_frame, textvariable=self.message_var, anchor='w').pack(fill='x')

    def _build_motion_pad(self, parent) -> None:
        frame = self._ttk.LabelFrame(parent, text='运动')
        frame.pack(side='left', fill='both', expand=True, padx=(0, 8))

        grid = self._ttk.Frame(frame)
        grid.pack(padx=8, pady=8)

        self._add_movement_button(grid, 'Q', 'q', 0, 0)
        self._add_movement_button(grid, 'W', 'w', 0, 1)
        self._add_movement_button(grid, 'E', 'e', 0, 2)
        self._add_movement_button(grid, 'A', 'a', 1, 0)
        self._add_movement_button(grid, 'S', 's', 1, 1)
        self._add_movement_button(grid, 'D', 'd', 1, 2)
        self._add_movement_button(grid, 'X', 'x', 2, 1, sticky='nsew')

        for col in range(3):
            grid.columnconfigure(col, weight=1)
        for row in range(3):
            grid.rowconfigure(row, weight=1)

    def _build_speed_pad(self, parent) -> None:
        frame = self._ttk.LabelFrame(parent, text='调速')
        frame.pack(side='left', fill='both', expand=True)

        grid = self._ttk.Frame(frame)
        grid.pack(padx=8, pady=8)

        self._add_action_button(grid, 'I +', 'i', 0, 0)
        self._add_action_button(grid, 'K -', 'k', 0, 1)
        self._add_action_button(grid, 'O +', 'o', 1, 0)
        self._add_action_button(grid, 'L -', 'l', 1, 1)
        self._add_action_button(grid, 'M 保存', 'm', 2, 0, columnspan=2)

        for col in range(2):
            grid.columnconfigure(col, weight=1)
        for row in range(3):
            grid.rowconfigure(row, weight=1)

    def _add_movement_button(self, parent, label, key, row, column, sticky='nsew') -> None:
        button = self._ttk.Button(parent, text=label, width=10)
        button.grid(row=row, column=column, padx=5, pady=5, sticky=sticky)
        button.bind('<ButtonPress-1>', lambda _event, key=key: self._press_key(key))
        button.bind('<ButtonRelease-1>', lambda _event, key=key: self._release_key(key))

    def _add_action_button(self, parent, label, key, row, column, columnspan=1) -> None:
        button = self._ttk.Button(parent, text=label, width=10, command=lambda key=key: self._trigger_action(key))
        button.grid(row=row, column=column, columnspan=columnspan, padx=5, pady=5, sticky='nsew')

    def _press_key(self, key: str) -> None:
        if self.command_handler is None:
            return
        self.command_handler.handle_key_press(key, time.monotonic())

    def _release_key(self, key: str) -> None:
        if self.command_handler is None:
            return
        self.command_handler.handle_key_release(key, time.monotonic())

    def _trigger_action(self, key: str) -> None:
        if self.command_handler is None:
            return
        self.command_handler.handle_key_press(key, time.monotonic())

    def _on_key_press(self, event) -> str | None:
        if self.command_handler is None:
            return None

        key = self._event_to_key(event)
        if key is None:
            return None

        self.command_handler.handle_key_press(key, time.monotonic())
        return 'break'

    def _on_key_release(self, event) -> str | None:
        if self.command_handler is None:
            return None

        key = self._event_to_key(event)
        if key is None:
            return None

        self.command_handler.handle_key_release(key, time.monotonic())
        return 'break'

    def _on_escape(self, _event) -> str:
        self._on_close()
        return 'break'

    def _on_close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if not self.stop_event.is_set():
            self.stop_event.set()
        self.status_display.close()
        self.root.quit()
        self.root.destroy()

    def _poll_stop(self) -> None:
        if self._closed:
            return
        if self.stop_event.is_set():
            self._on_close()
            return
        self.root.after(100, self._poll_stop)

    @staticmethod
    def _event_to_key(event) -> str | None:
        if event is None:
            return None

        char = getattr(event, 'char', '') or ''
        if char:
            normalized = char.lower()
            if normalized in {'w', 'a', 's', 'd', 'q', 'e', 'x', 'i', 'k', 'o', 'l', 'm'}:
                return normalized

        keysym = str(getattr(event, 'keysym', '')).lower()
        if keysym in {'w', 'a', 's', 'd', 'q', 'e', 'x', 'i', 'k', 'o', 'l', 'm'}:
            return keysym

        return None


class CmdVelPublisherThread(threading.Thread):
    def __init__(self, node, publisher, twist_msg, twist, stamped, core, rate_hz, stop_event, stale_key_timeout_sec, status_display):
        super().__init__(daemon=True)
        self.node = node
        self.publisher = publisher
        self.twist_msg = twist_msg
        self.twist = twist
        self.stamped = stamped
        self.core = core
        self.stop_event = stop_event
        self.period = 1.0 / rate_hz if rate_hz > 0.0 else 0.01
        self.stale_key_timeout_sec = stale_key_timeout_sec
        self.status_display = status_display

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
            command = self.core.snapshot(stale_key_timeout_sec=self.stale_key_timeout_sec)
            self._publish_command(command)
            if hasattr(self.status_display, 'update_motion'):
                self.status_display.update_motion(command.linear_x, command.linear_y, command.angular_z)
            self.stop_event.wait(self.period)

        self.core.emergency_stop()
        self._publish_command(self.core.zero_command())


class KeyboardCommandHandler:
    def __init__(self, core, stop_event, save_map_hotkey, status_display):
        self.core = core
        self.stop_event = stop_event
        self.save_map_hotkey = save_map_hotkey
        self.status_display = status_display

    def handle_key_press(self, key: str, now: float) -> None:
        normalized_key = str(key).lower()

        if self.save_map_hotkey.matches(normalized_key):
            try:
                trigger_result = self.save_map_hotkey.trigger(now)
            except Exception as exception:
                self.status_display.print_event(f'触发地图保存失败: {exception}')
                return

            if trigger_result == 'started':
                self.status_display.print_event(
                    f'快捷键 {self.save_map_hotkey.trigger_key.upper()} -> 开始保存当前地图'
                )
            elif trigger_result == 'running':
                self.status_display.print_event('地图保存进行中，本次按键已忽略')
            elif trigger_result == 'cooldown':
                self.status_display.print_event('地图保存快捷键冷却中，本次按键已忽略')
            elif trigger_result == 'disabled':
                self.status_display.print_event('地图保存快捷键已禁用，请检查 save_map_key / save_map_command')
            return

        if self.core.handle_key_press(normalized_key, now) and self.core.is_speed_key(normalized_key):
            status = self.core.status()
            self.status_display.update_speed_status(status.linear_speed, status.angular_speed)

    def handle_key_release(self, key: str, now: float) -> None:
        normalized_key = str(key).lower()
        if self.save_map_hotkey.matches(normalized_key):
            return
        self.core.handle_key_release(normalized_key, now)


class TtyKeyboardThread(threading.Thread):
    def __init__(self, command_handler, stop_event, tty_device_path, read_poll_timeout, status_display):
        super().__init__(daemon=True)
        self.command_handler = command_handler
        self.stop_event = stop_event
        self.tty_device_path = str(tty_device_path)
        self.read_poll_timeout = max(0.001, float(read_poll_timeout))
        self.status_display = status_display

    def _handle_input_char(self, char_code, now):
        if char_code == 3:
            self.stop_event.set()
            return

        self.command_handler.handle_key_press(chr(char_code).lower(), now)

    def run(self):
        tty_fd = -1
        old_termios = None

        try:
            tty_fd = os.open(self.tty_device_path, os.O_RDONLY | os.O_NONBLOCK)
            old_termios = termios.tcgetattr(tty_fd)
            tty.setraw(tty_fd)
            new_termios = termios.tcgetattr(tty_fd)
            new_termios[1] |= termios.OPOST | termios.ONLCR
            termios.tcsetattr(tty_fd, termios.TCSADRAIN, new_termios)
            self.status_display.print_event(f'TTY 键盘模式已连接到 {self.tty_device_path}')

            while rclpy.ok() and not self.stop_event.is_set():
                ready_fds, _, _ = select.select([tty_fd], [], [], self.read_poll_timeout)
                if not ready_fds:
                    continue

                try:
                    chunk = os.read(tty_fd, 64)
                except BlockingIOError:
                    continue
                except OSError:
                    continue

                if not chunk:
                    continue

                now = time.monotonic()
                for char_code in chunk:
                    self._handle_input_char(char_code, now)
                    if self.stop_event.is_set():
                        break
        except Exception as exception:
            self.status_display.print_event(f'TTY 键盘线程失败: {exception}')
            self.stop_event.set()
        finally:
            self.command_handler.core.clear_move_keys(time.monotonic())
            if tty_fd >= 0 and old_termios is not None:
                try:
                    termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_termios)
                except OSError:
                    pass
            if tty_fd >= 0:
                try:
                    os.close(tty_fd)
                except OSError:
                    pass


class EventKeyboardThread(threading.Thread):
    def __init__(self, command_handler, stop_event, event_device_path, read_poll_timeout, status_display):
        super().__init__(daemon=True)
        self.command_handler = command_handler
        self.stop_event = stop_event
        self.event_device_path = str(event_device_path)
        self.read_poll_timeout = max(0.001, float(read_poll_timeout))
        self.status_display = status_display

    def _handle_input_event(self, event_type, key_code, key_value, now):
        if event_type != EV_KEY:
            return

        key = LINUX_KEY_CODE_BINDINGS.get(key_code)
        if key is None:
            return

        if key_value == KEY_EVENT_RELEASE:
            self.command_handler.handle_key_release(key, now)
            return

        if key_value == KEY_EVENT_PRESS:
            self.command_handler.handle_key_press(key, now)
            return

        if key_value == KEY_EVENT_REPEAT and self.command_handler.core.is_speed_key(key):
            self.command_handler.handle_key_press(key, now)

    def run(self):
        event_fd = -1
        raw_buffer = b''

        try:
            event_fd = os.open(self.event_device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.status_display.print_event(f'事件键盘模式已连接到 {self.event_device_path}')

            while rclpy.ok() and not self.stop_event.is_set():
                ready_fds, _, _ = select.select([event_fd], [], [], self.read_poll_timeout)
                if not ready_fds:
                    continue

                try:
                    chunk = os.read(event_fd, LINUX_INPUT_EVENT_STRUCT.size * 64)
                except BlockingIOError:
                    continue
                except OSError as exception:
                    self.status_display.print_event(f'读取键盘事件失败: {exception}')
                    self.stop_event.set()
                    break

                if not chunk:
                    continue

                raw_buffer += chunk
                while len(raw_buffer) >= LINUX_INPUT_EVENT_STRUCT.size:
                    raw_event = raw_buffer[:LINUX_INPUT_EVENT_STRUCT.size]
                    raw_buffer = raw_buffer[LINUX_INPUT_EVENT_STRUCT.size:]
                    _, _, event_type, key_code, key_value = LINUX_INPUT_EVENT_STRUCT.unpack(raw_event)
                    self._handle_input_event(event_type, key_code, key_value, time.monotonic())
                    if self.stop_event.is_set():
                        break
        except Exception as exception:
            self.status_display.print_event(f'事件键盘线程失败: {exception}')
            self.stop_event.set()
        finally:
            self.command_handler.core.clear_move_keys(time.monotonic())
            if event_fd >= 0:
                try:
                    os.close(event_fd)
                except OSError:
                    pass


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop_node')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', DEFAULT_STAMPED, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', DEFAULT_FRAME_ID, read_only_descriptor).value
    speed = node.declare_parameter('speed', DEFAULT_LINEAR_SPEED, read_only_descriptor).value
    turn = node.declare_parameter('turn', DEFAULT_ANGULAR_SPEED, read_only_descriptor).value
    max_speed = node.declare_parameter('max_speed', DEFAULT_MAX_LINEAR_SPEED, read_only_descriptor).value
    max_turn = node.declare_parameter('max_turn', DEFAULT_MAX_ANGULAR_SPEED, read_only_descriptor).value
    speed_step = node.declare_parameter('speed_step', DEFAULT_LINEAR_SPEED_STEP, read_only_descriptor).value
    turn_step = node.declare_parameter('turn_step', DEFAULT_ANGULAR_SPEED_STEP, read_only_descriptor).value
    cmd_vel_topic = node.declare_parameter('cmd_vel_topic', DEFAULT_CMD_VEL_TOPIC, read_only_descriptor).value
    publish_rate = node.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE, read_only_descriptor).value
    repeat_timeout_sec = node.declare_parameter(
        'repeat_timeout_sec',
        DEFAULT_REPEAT_TIMEOUT_SEC,
        read_only_descriptor,
    ).value
    read_poll_timeout = node.declare_parameter(
        'read_poll_timeout',
        DEFAULT_READ_POLL_TIMEOUT_SEC,
        read_only_descriptor,
    ).value
    tty_device_path = node.declare_parameter('tty_device_path', DEFAULT_TTY_DEVICE_PATH, read_only_descriptor).value
    keyboard_backend = normalize_keyboard_backend(
        node.declare_parameter('keyboard_backend', DEFAULT_KEYBOARD_BACKEND, read_only_descriptor).value
    )
    event_device_path = node.declare_parameter(
        'event_device_path',
        DEFAULT_EVENT_DEVICE_PATH,
        read_only_descriptor,
    ).value
    save_map_key = node.declare_parameter('save_map_key', DEFAULT_SAVE_MAP_KEY, read_only_descriptor).value
    save_map_command = node.declare_parameter(
        'save_map_command',
        DEFAULT_SAVE_MAP_COMMAND,
        read_only_descriptor,
    ).value
    save_map_cooldown_sec = node.declare_parameter(
        'save_map_cooldown_sec',
        DEFAULT_SAVE_MAP_COOLDOWN_SEC,
        read_only_descriptor,
    ).value

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

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    stop_event = threading.Event()
    status_display = ConsoleStatusDisplay()
    tk_window = None
    core = KeyboardTeleopCore(
        linear_speed=float(speed),
        angular_speed=float(turn),
        speed_step=float(speed_step),
        turn_step=float(turn_step),
        max_linear_speed=float(max_speed),
        max_angular_speed=float(max_turn),
        accel_limit_linear=ACCEL_LIMIT_LINEAR,
        decel_limit_linear=DECEL_LIMIT_LINEAR,
        accel_limit_angular=ACCEL_LIMIT_ANGULAR,
        decel_limit_angular=DECEL_LIMIT_ANGULAR,
        idle_timeout_sec=IDLE_TIMEOUT_SEC,
    )

    if keyboard_backend == 'tk':
        tk_window = TkKeyboardWindow(core=core, stop_event=stop_event)
        status_display = tk_window.status_display

    publisher_thread = CmdVelPublisherThread(
        node=node,
        publisher=publisher,
        twist_msg=twist_msg,
        twist=twist,
        stamped=stamped,
        core=core,
        rate_hz=float(publish_rate),
        stop_event=stop_event,
        stale_key_timeout_sec=None if keyboard_backend == 'event' else max(0.0, float(repeat_timeout_sec)),
        status_display=status_display,
    )

    save_map_hotkey = ShellCommandHotkey(
        trigger_key=save_map_key,
        command=save_map_command,
        cooldown_sec=float(save_map_cooldown_sec),
    )
    command_handler = KeyboardCommandHandler(
        core=core,
        stop_event=stop_event,
        save_map_hotkey=save_map_hotkey,
        status_display=status_display,
    )
    if tk_window is not None:
        tk_window.bind_command_handler(command_handler)

    keyboard_thread = None

    def report_save_map_status():
        return_code = save_map_hotkey.consume_exit_code()
        if return_code is None:
            return

        if return_code == 0:
            status_display.print_event('当前地图保存完成')
            return

        status_display.print_event(f'当前地图保存失败，退出码 {return_code}')

    node.create_timer(0.2, report_save_map_status)

    try:
        if keyboard_backend == 'tty':
            resolved_input_path = resolve_tty_device_path(str(tty_device_path))
            keyboard_thread = TtyKeyboardThread(
                command_handler=command_handler,
                stop_event=stop_event,
                tty_device_path=resolved_input_path,
                read_poll_timeout=read_poll_timeout,
                status_display=status_display,
            )
        elif keyboard_backend == 'tk':
            resolved_input_path = 'Tk GUI'
        else:
            resolved_input_path = resolve_event_device_path(str(event_device_path))
            keyboard_thread = EventKeyboardThread(
                command_handler=command_handler,
                stop_event=stop_event,
                event_device_path=resolved_input_path,
                read_poll_timeout=read_poll_timeout,
                status_display=status_display,
            )

        print(MSG)
        print(f'后端: {keyboard_backend}')
        if keyboard_backend == 'tty':
            print('TTY: %.2f s / %.3f s' % (
                max(0.0, float(repeat_timeout_sec)),
                max(0.001, float(read_poll_timeout)),
            ))
            print(f'设备: {resolved_input_path}')
        elif keyboard_backend == 'tk':
            print('Tk: 窗口按钮/快捷键')
        else:
            print(f'事件: {resolved_input_path}')
        if save_map_hotkey.is_enabled():
            print('保存: %s, %.2f s' % (
                save_map_hotkey.trigger_key.upper(),
                float(save_map_cooldown_sec),
            ))
        print('限速: %.2f / %.2f' % (
            float(max_speed),
            float(max_turn),
        ))
        status_display.update_speed_status(float(speed), float(turn))
        publisher_thread.start()
        if tk_window is not None:
            status_display.print_event('Tk GUI 键盘模式已启动')
            tk_window.run()
        else:
            keyboard_thread.start()
            while rclpy.ok() and not stop_event.is_set():
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exception:
        status_display.print_event(f'键盘控制启动失败: {exception}')
    finally:
        status_display.clear_status()
        stop_event.set()
        core.emergency_stop()
        if tk_window is not None:
            tk_window._on_close()
        if keyboard_thread is not None and keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
        if publisher_thread.is_alive():
            publisher_thread.join(timeout=1.0)
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
