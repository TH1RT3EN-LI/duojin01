import serial
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class MissionExecutorNode(Node):
    """
    任务执行器底层 API 节点。

    此节点不内置任务序列，仅封装三类能力供用户脚本调用：
      - 导航：发布 /mission/nav_goal  → 结果从 /mission/nav_result 获取
      - 摄像：发布 /mission/capture_trigger  → 帧从 /mission/image_capture 获取
      - 串口：发布 /mission/gcode_cmd  → 结果从 /mission/gcode_result 获取

    用户脚本示例见 scripts/ 目录。
    """

    def __init__(self):
        super().__init__('mission_executor_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('gcode_response_timeout', 5.0)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('serial_baudrate').value
        self._gcode_timeout = self.get_parameter('gcode_response_timeout').value

        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=self._gcode_timeout)
            self.get_logger().info(f'串口已打开: {serial_port}@{baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self.ser = None

        self.cb_group = ReentrantCallbackGroup()

        # ── Nav2 Action Client ──────────────────────────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        # ── 输入话题（用户脚本发布到这里）─────────────────────────────
        self.create_subscription(
            PoseStamped, '/mission/nav_goal',
            self._on_nav_goal, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Bool, '/mission/capture_trigger',
            self._on_capture_trigger, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String, '/mission/gcode_cmd',
            self._on_gcode_cmd, 10,
            callback_group=self.cb_group,
        )

        # ── 结果话题（用户脚本订阅这里）────────────────────────────────
        self.pub_nav_result = self.create_publisher(Bool, '/mission/nav_result', 10)
        self.pub_image = self.create_publisher(Image, '/mission/image_capture', 10)
        self.pub_gcode_result = self.create_publisher(Bool, '/mission/gcode_result', 10)

        self._navigating = False
        self._image_sub = None

        self.get_logger().info('mission_executor_node 就绪')

    # ──────────────────────────────────────────────────────────────────
    # 导航 API
    # ──────────────────────────────────────────────────────────────────

    def _on_nav_goal(self, msg: PoseStamped):
        if self._navigating:
            self.get_logger().warn('导航中，忽略新目标点')
            return
        self._navigating = True
        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f'导航目标: x={x:.2f}, y={y:.2f}')

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server 不可用')
            self._navigating = False
            self._publish_nav_result(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback,
        )
        future.add_done_callback(self._nav_goal_response)

    def _nav_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'剩余距离: {dist:.2f} m', throttle_duration_sec=2.0)

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self._navigating = False
            self._publish_nav_result(False)
            return
        goal_handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        success = future.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'导航{"成功" if success else "失败"}')
        self._navigating = False
        self._publish_nav_result(success)

    def _publish_nav_result(self, success: bool):
        msg = Bool()
        msg.data = success
        self.pub_nav_result.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # 摄像 API
    # ──────────────────────────────────────────────────────────────────

    def _on_capture_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self._image_sub is not None:
            self.get_logger().warn('摄像头已在捕获中')
            return
        self.get_logger().info('触发摄像头捕获')
        self._image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self._on_image, 1,
            callback_group=self.cb_group,
        )

    def _on_image(self, msg: Image):
        self.get_logger().info(f'图像已捕获: {msg.width}x{msg.height} {msg.encoding}')
        self.destroy_subscription(self._image_sub)
        self._image_sub = None
        self.pub_image.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # 串口 / G 代码 API
    # ──────────────────────────────────────────────────────────────────

    def _on_gcode_cmd(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return
        success = self._send_gcode(cmd)
        result = Bool()
        result.data = success
        self.pub_gcode_result.publish(result)

    def _send_gcode(self, cmd: str) -> bool:
        """发送单条 G 代码并等待 'ok' 响应。"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('串口不可用')
            return False
        try:
            self.ser.write((cmd.strip() + '\n').encode())
            self.get_logger().info(f'G代码 → {cmd}')
            response = self.ser.readline().decode(errors='replace').strip()
            self.get_logger().info(f'G代码响应: {response!r}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'串口发送失败: {e}')
            return False

    # ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
