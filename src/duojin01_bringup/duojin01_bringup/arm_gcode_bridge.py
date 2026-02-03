#!/usr/bin/env python3
"""订阅 arm_gcode_command (std_msgs/String)，将 G 代码写入机械臂串口。"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    print("请安装 pyserial: pip3 install pyserial", file=sys.stderr)
    sys.exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = Node("arm_gcode_bridge")

    port_name = node.declare_parameter("port_name", "/dev/ttyUSB0").value
    baud_rate = node.declare_parameter("baud_rate", 115200).value
    topic_name = node.declare_parameter("gcode_command_topic", "arm_gcode_command").value

    ser = None
    try:
        ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=1.0)
        node.get_logger().info("串口已打开: %s @ %d" % (port_name, baud_rate))
    except serial.SerialException as e:
        node.get_logger().fatal("无法打开串口 %s: %s" % (port_name, e))
        rclpy.shutdown()
        sys.exit(1)

    def on_gcode(msg):
        line = (msg.data or "").strip()
        if not line:
            return
        to_send = line if line.endswith("\r\n") else line + "\r\n"
        try:
            ser.write(to_send.encode("utf-8"))
            node.get_logger().info("已发送: %s" % line)
        except serial.SerialException as e:
            node.get_logger().error("串口写入失败: %s" % e)

    sub = node.create_subscription(String, topic_name, on_gcode, 10)

    def shutdown():
        if ser and ser.is_open:
            ser.close()
            node.get_logger().info("串口已关闭")

    try:
        rclpy.spin(node)
    finally:
        shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
