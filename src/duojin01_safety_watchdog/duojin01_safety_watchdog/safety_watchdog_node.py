import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, FunctionDiagnosticTask
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


class SafetyWatchdogNode(Node):
    """
    diagnostic_updater + twist_mux lock
    检测内容包括：
    1. 电池电压过低
    2. IMU倾斜
    3. /odom, /imu, /battery_voltage 话题超时
    """

    REASON_ODOM_TIMEOUT = "底盘里程计 (/odom) 超时，疑似串口断联"
    REASON_IMU_TIMEOUT = "IMU (/imu) 超时，疑似串口断联"
    REASON_BATTERY_TIMEOUT = "电池电压 (/battery_voltage) 超时，疑似串口断联"
    REASON_BATTERY_LOW = "电池电压过低 (滑动窗口均值)"
    REASON_IMU_TILT = "IMU 连续检测到姿态异常（倾斜角过大）"

    def __init__(self):
        super().__init__("safety_watchdog")

        self.declare_parameter("check_rate_hz", 50.0)
        self.declare_parameter("odom_timeout_sec", 2.0)
        self.declare_parameter("imu_timeout_sec", 2.0)
        self.declare_parameter("battery_timeout_sec", 5.0)
        self.declare_parameter("grace_period_sec", 15.0)
        self.declare_parameter("enable_odom_check", True)
        self.declare_parameter("enable_imu_check", True)
        self.declare_parameter("enable_battery_check", True)

        self.declare_parameter("enable_battery_voltage_check", True)
        self.declare_parameter("battery_min_voltage", 10.0)
        self.declare_parameter("battery_window_size", 50)    
        self.declare_parameter("battery_hysteresis_voltage", 0.5) 

        self.declare_parameter("enable_tilt_check", True)
        self.declare_parameter("max_tilt_degrees", 45.0)
        self.declare_parameter("tilt_confirm_count", 20)      

        self.declare_parameter("diag_period_sec", 1.0)

        self._check_rate = self.get_parameter("check_rate_hz").value
        self._odom_timeout = self.get_parameter("odom_timeout_sec").value
        self._imu_timeout = self.get_parameter("imu_timeout_sec").value
        self._battery_timeout = self.get_parameter("battery_timeout_sec").value
        self._grace_period = self.get_parameter("grace_period_sec").value
        self._enable_odom = self.get_parameter("enable_odom_check").value
        self._enable_imu = self.get_parameter("enable_imu_check").value
        self._enable_battery = self.get_parameter("enable_battery_check").value

        self._enable_battery_v = self.get_parameter("enable_battery_voltage_check").value
        self._battery_min_v = self.get_parameter("battery_min_voltage").value
        self._battery_window_size = self.get_parameter("battery_window_size").value
        self._battery_hysteresis = self.get_parameter("battery_hysteresis_voltage").value

        self._enable_tilt = self.get_parameter("enable_tilt_check").value
        self._max_tilt_rad = math.radians(self.get_parameter("max_tilt_degrees").value)
        self._tilt_confirm_count = self.get_parameter("tilt_confirm_count").value

        diag_period = self.get_parameter("diag_period_sec").value

        self._triggered = False
        self._trigger_reason = ""
        self._trigger_time = None          
        self._last_reminder_time = None     
        self._start_time = self.get_clock().now()

        self._last_odom_time = None
        self._last_imu_time = None
        self._last_battery_time = None

        self._battery_window = collections.deque(maxlen=self._battery_window_size)
        self._battery_avg = None  

        self._tilt_consecutive = 0
        self._last_roll_deg = 0.0
        self._last_pitch_deg = 0.0

        lock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._lock_pub = self.create_publisher(Bool, "/watchdog/lock", lock_qos)


        self._status_pub = self.create_publisher(String, "/watchdog/status", 10)

        unlock_msg = Bool()
        unlock_msg.data = False
        self._lock_pub.publish(unlock_msg)

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)
        self.create_subscription(Float32, "/battery_voltage", self._battery_cb, 10)

        self.create_service(Trigger, "/watchdog/reset", self._reset_cb)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("duojin01")
        self._diag_updater.period = diag_period

        self._diag_updater.add(FunctionDiagnosticTask(
            "总状态", self._diag_overall))
        self._diag_updater.add(FunctionDiagnosticTask(
            "底盘通信 (odom)", self._diag_odom))
        self._diag_updater.add(FunctionDiagnosticTask(
            "底盘通信 (imu)", self._diag_imu))
        self._diag_updater.add(FunctionDiagnosticTask(
            "底盘通信 (battery)", self._diag_battery_topic))
        self._diag_updater.add(FunctionDiagnosticTask(
            "电池电压", self._diag_battery_voltage))
        self._diag_updater.add(FunctionDiagnosticTask(
            "IMU 姿态", self._diag_tilt))

        self._timer = self.create_timer(1.0 / self._check_rate, self._check_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  watchdog 已启动")
        self.get_logger().info(f"  宽限期: {self._grace_period}s")
        self.get_logger().info(f"  odom 超时: {self._odom_timeout}s ({'ON' if self._enable_odom else 'OFF'})")
        self.get_logger().info(f"  IMU 超时: {self._imu_timeout}s ({'ON' if self._enable_imu else 'OFF'})")
        self.get_logger().info(f"  电池超时: {self._battery_timeout}s ({'ON' if self._enable_battery else 'OFF'})")
        self.get_logger().info(f"  电压保护: <{self._battery_min_v}V 窗口={self._battery_window_size} 迟滞={self._battery_hysteresis}V ({'ON' if self._enable_battery_v else 'OFF'})")
        self.get_logger().info(f"  倾斜保护: >{math.degrees(self._max_tilt_rad):.0f}° 连续{self._tilt_confirm_count}帧 ({'ON' if self._enable_tilt else 'OFF'})")
        self.get_logger().info("=" * 60)


    def _odom_cb(self, msg: Odometry):
        self._last_odom_time = self.get_clock().now()

    def _imu_cb(self, msg: Imu):
        self._last_imu_time = self.get_clock().now()
        if self._enable_tilt and not self._triggered:
            self._update_tilt(msg)

    def _battery_cb(self, msg: Float32):
        self._last_battery_time = self.get_clock().now()
        voltage = msg.data

        if voltage > 0.5:  
            self._battery_window.append(voltage)

        if len(self._battery_window) >= 5:  
            self._battery_avg = sum(self._battery_window) / len(self._battery_window)
        else:
            self._battery_avg = None

        if self._enable_battery_v and not self._triggered and self._battery_avg is not None:
            if self._battery_avg < self._battery_min_v:
                self._trigger_estop(
                    f"{self.REASON_BATTERY_LOW}: 均值={self._battery_avg:.2f}V < {self._battery_min_v:.2f}V "
                    f"(窗口{len(self._battery_window)}帧)"
                )


    def _reset_cb(self, request, response):
        if self._triggered:
            self.get_logger().warn("watchdog 已被手动重置，解除 twist_mux lock")
            self._triggered = False
            self._trigger_reason = ""
            self._trigger_time = None
            self._last_reminder_time = None
            unlock_msg = Bool()
            unlock_msg.data = False
            self._lock_pub.publish(unlock_msg)
            now = self.get_clock().now()
            self._start_time = now
            self._last_odom_time = None
            self._last_imu_time = None
            self._last_battery_time = None
            self._battery_window.clear()
            self._battery_avg = None
            self._tilt_consecutive = 0
            response.success = True
            response.message = "watchdog 已重置，twist_mux lock 已解除"
        else:
            response.success = True
            response.message = "watchdog 当前未处于触发状态"
        return response


    def _check_loop(self):
        if self._triggered:
            lock_msg = Bool()
            lock_msg.data = True
            self._lock_pub.publish(lock_msg)

            elapsed = (self.get_clock().now() - self._trigger_time).nanoseconds / 1e9
            status_msg = String()
            status_msg.data = f"LOCKED ({elapsed:.0f}s) | {self._trigger_reason}"
            self._status_pub.publish(status_msg)

            now = self.get_clock().now()
            since_reminder = (now - self._last_reminder_time).nanoseconds / 1e9
            if since_reminder >= 5.0:
                self._last_reminder_time = now
                self.get_logger().warn(
                    f"[紧急停车持续中 {elapsed:.0f}s] {self._trigger_reason} "
                    f"| 恢复: ros2 service call /watchdog/reset std_srvs/srv/Trigger")
            return

        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed < self._grace_period:
            status_msg = String()
            status_msg.data = f"GRACE_PERIOD ({elapsed:.0f}/{self._grace_period:.0f}s)"
            self._status_pub.publish(status_msg)
            return

        if self._enable_odom:
            if self._last_odom_time is None:
                self._trigger_estop(f"{self.REASON_ODOM_TIMEOUT} (宽限期后仍未收到)")
                return
            age = (now - self._last_odom_time).nanoseconds / 1e9
            if age > self._odom_timeout:
                self._trigger_estop(f"{self.REASON_ODOM_TIMEOUT} (已 {age:.1f}s 未收到)")
                return

        if self._enable_imu:
            if self._last_imu_time is None:
                self._trigger_estop(f"{self.REASON_IMU_TIMEOUT} (宽限期后仍未收到)")
                return
            age = (now - self._last_imu_time).nanoseconds / 1e9
            if age > self._imu_timeout:
                self._trigger_estop(f"{self.REASON_IMU_TIMEOUT} (已 {age:.1f}s 未收到)")
                return

        if self._enable_battery:
            if self._last_battery_time is None:
                self._trigger_estop(f"{self.REASON_BATTERY_TIMEOUT} (宽限期后仍未收到)")
                return
            age = (now - self._last_battery_time).nanoseconds / 1e9
            if age > self._battery_timeout:
                self._trigger_estop(f"{self.REASON_BATTERY_TIMEOUT} (已 {age:.1f}s 未收到)")
                return

        status_msg = String()
        status_msg.data = "OK"
        self._status_pub.publish(status_msg)


    def _update_tilt(self, imu_msg: Imu):
        q = imu_msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        self._last_roll_deg = math.degrees(roll)
        self._last_pitch_deg = math.degrees(pitch)

        if abs(roll) > self._max_tilt_rad or abs(pitch) > self._max_tilt_rad:
            self._tilt_consecutive += 1
            if self._tilt_consecutive >= self._tilt_confirm_count:
                self._trigger_estop(
                    f"{self.REASON_IMU_TILT} "
                    f"(roll={self._last_roll_deg:.1f}°, pitch={self._last_pitch_deg:.1f}°, "
                    f"连续{self._tilt_consecutive}帧)"
                )
        else:
            self._tilt_consecutive = 0  


    def _trigger_estop(self, reason: str):
        self._triggered = True
        self._trigger_reason = reason
        self._trigger_time = self.get_clock().now()
        self._last_reminder_time = self._trigger_time

        lock_msg = Bool()
        lock_msg.data = True
        for _ in range(10):
            self._lock_pub.publish(lock_msg)

        self.get_logger().error("!" * 60)
        self.get_logger().error("!  ██  紧急停车 (twist_mux LOCKED)  ██")
        self.get_logger().error(f"!  原因: {reason}")
        self.get_logger().error("!  twist_mux 已锁定，一切上位机速度指令被阻断")
        self.get_logger().error("!  手动恢复: ros2 service call /watchdog/reset std_srvs/srv/Trigger")
        self.get_logger().error("!  下位机可能仍在错误工作，如果车辆仍在运动建议断电急停")
        self.get_logger().error("!" * 60)



    def _diag_overall(self, stat):
        if self._triggered:
            stat.summary(DiagnosticStatus.ERROR, f"紧急停车: {self._trigger_reason}")
        else:
            now = self.get_clock().now()
            elapsed = (now - self._start_time).nanoseconds / 1e9
            if elapsed < self._grace_period:
                stat.summary(DiagnosticStatus.WARN, f"启动宽限期中 ({elapsed:.0f}/{self._grace_period:.0f}s)")
            else:
                stat.summary(DiagnosticStatus.OK, "所有检查正常")
        stat.add("触发状态", "是" if self._triggered else "否")
        stat.add("触发原因", self._trigger_reason if self._trigger_reason else "无")
        return stat

    def _diag_odom(self, stat):
        if not self._enable_odom:
            stat.summary(DiagnosticStatus.OK, "检查已禁用")
            return stat
        if self._last_odom_time is None:
            stat.summary(DiagnosticStatus.WARN, "等待首条消息")
        else:
            age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
            stat.add("最后消息距今 (s)", f"{age:.2f}")
            stat.add("超时阈值 (s)", f"{self._odom_timeout}")
            if age > self._odom_timeout:
                stat.summary(DiagnosticStatus.ERROR, f"超时 ({age:.1f}s > {self._odom_timeout}s)")
            elif age > self._odom_timeout * 0.5:
                stat.summary(DiagnosticStatus.WARN, f"接近超时 ({age:.1f}s)")
            else:
                stat.summary(DiagnosticStatus.OK, f"正常 ({age:.2f}s)")
        return stat

    def _diag_imu(self, stat):
        if not self._enable_imu:
            stat.summary(DiagnosticStatus.OK, "检查已禁用")
            return stat
        if self._last_imu_time is None:
            stat.summary(DiagnosticStatus.WARN, "等待首条消息")
        else:
            age = (self.get_clock().now() - self._last_imu_time).nanoseconds / 1e9
            stat.add("最后消息距今 (s)", f"{age:.2f}")
            stat.add("超时阈值 (s)", f"{self._imu_timeout}")
            if age > self._imu_timeout:
                stat.summary(DiagnosticStatus.ERROR, f"超时 ({age:.1f}s > {self._imu_timeout}s)")
            elif age > self._imu_timeout * 0.5:
                stat.summary(DiagnosticStatus.WARN, f"接近超时 ({age:.1f}s)")
            else:
                stat.summary(DiagnosticStatus.OK, f"正常 ({age:.2f}s)")
        return stat

    def _diag_battery_topic(self, stat):
        if not self._enable_battery:
            stat.summary(DiagnosticStatus.OK, "检查已禁用")
            return stat
        if self._last_battery_time is None:
            stat.summary(DiagnosticStatus.WARN, "等待首条消息")
        else:
            age = (self.get_clock().now() - self._last_battery_time).nanoseconds / 1e9
            stat.add("最后消息距今 (s)", f"{age:.2f}")
            stat.add("超时阈值 (s)", f"{self._battery_timeout}")
            if age > self._battery_timeout:
                stat.summary(DiagnosticStatus.ERROR, f"超时 ({age:.1f}s > {self._battery_timeout}s)")
            elif age > self._battery_timeout * 0.5:
                stat.summary(DiagnosticStatus.WARN, f"接近超时 ({age:.1f}s)")
            else:
                stat.summary(DiagnosticStatus.OK, f"正常 ({age:.2f}s)")
        return stat

    def _diag_battery_voltage(self, stat):
        if not self._enable_battery_v:
            stat.summary(DiagnosticStatus.OK, "检查已禁用")
            return stat
        if self._battery_avg is None:
            stat.summary(DiagnosticStatus.WARN, "采样不足, 等待数据")
            stat.add("窗口采样数", f"{len(self._battery_window)}/{self._battery_window_size}")
        else:
            stat.add("窗口均值 (V)", f"{self._battery_avg:.2f}")
            stat.add("最低阈值 (V)", f"{self._battery_min_v:.2f}")
            stat.add("迟滞 (V)", f"{self._battery_hysteresis:.2f}")
            stat.add("窗口采样数", f"{len(self._battery_window)}/{self._battery_window_size}")
            if self._battery_avg < self._battery_min_v:
                stat.summary(DiagnosticStatus.ERROR,
                    f"欠压! 均值={self._battery_avg:.2f}V < {self._battery_min_v:.2f}V")
            elif self._battery_avg < self._battery_min_v + self._battery_hysteresis:
                stat.summary(DiagnosticStatus.WARN,
                    f"电压偏低 均值={self._battery_avg:.2f}V")
            else:
                stat.summary(DiagnosticStatus.OK,
                    f"正常 均值={self._battery_avg:.2f}V")
        return stat

    def _diag_tilt(self, stat):
        if not self._enable_tilt:
            stat.summary(DiagnosticStatus.OK, "检查已禁用")
            return stat
        max_deg = math.degrees(self._max_tilt_rad)
        stat.add("roll (°)", f"{self._last_roll_deg:.1f}")
        stat.add("pitch (°)", f"{self._last_pitch_deg:.1f}")
        stat.add("连续超限帧数", f"{self._tilt_consecutive}/{self._tilt_confirm_count}")
        stat.add("最大允许角度 (°)", f"{max_deg:.0f}")

        if self._tilt_consecutive >= self._tilt_confirm_count:
            stat.summary(DiagnosticStatus.ERROR,
                f"姿态异常! roll={self._last_roll_deg:.1f}° pitch={self._last_pitch_deg:.1f}°")
        elif self._tilt_consecutive > 0:
            stat.summary(DiagnosticStatus.WARN,
                f"倾斜波动中 ({self._tilt_consecutive}/{self._tilt_confirm_count}帧)")
        else:
            stat.summary(DiagnosticStatus.OK, "姿态正常")
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
