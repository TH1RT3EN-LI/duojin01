#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rosgraph_msgs.msg import Clock


class ClockGuard(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-01-29
    由于设备性能的不确定性以及仿真器、桥接器的不同组合导致的/clock倒退可能扰乱仿真的正常运行，编写该节点来丢弃时钟时间早于上一条被该节点转发的消息的记录
    """

    def __init__(self) -> None:
        super().__init__("duojin01_clock_guard")
        # ROS Topic /clock_raw 定义于 src/duojin01_bringup/config/ros_gz_bridge.yaml，将 Gazebo 的 clock 先引流到 /clock_raw，经过该 node 清洗后发布到 /clock
        self.declare_parameter("input_topic", "/clock_raw")
        self.declare_parameter("output_topic", "/clock")
        self.declare_parameter("allow_clock_reset", True)
        self.declare_parameter("reset_newer_than_sec", 30.0)
        self.declare_parameter("reset_older_than_sec", 5.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._allow_clock_reset = bool(self.get_parameter("allow_clock_reset").value)
        self._reset_newer_than_ns = int(float(self.get_parameter("reset_newer_than_sec").value) * 1_000_000_000)
        self._reset_older_than_ns = int(float(self.get_parameter("reset_older_than_sec").value) * 1_000_000_000)

        in_qos = QoSProfile(depth=1)
        in_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        in_qos.durability = DurabilityPolicy.VOLATILE

        out_qos = QoSProfile(depth=1)
        out_qos.reliability = ReliabilityPolicy.RELIABLE
        out_qos.durability = DurabilityPolicy.VOLATILE

        self._pub = self.create_publisher(Clock, output_topic, out_qos)
        self._sub = self.create_subscription(Clock, input_topic, self._on_clock, in_qos)

        self._last_ns = None 

        self.get_logger().info(f"clock_guard started: {input_topic} -> {output_topic}")

    def _on_clock(self, msg: Clock) -> None:
        now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self._last_ns is not None and now_ns < self._last_ns:
            if (
                self._allow_clock_reset
                and self._last_ns >= self._reset_newer_than_ns
                and now_ns <= self._reset_older_than_ns
            ):
                self.get_logger().warn(
                    f"clock reset detected: {self._last_ns / 1e9:.3f}s -> {now_ns / 1e9:.3f}s, accepting new epoch"
                )
                self._last_ns = now_ns
                self._pub.publish(msg)
                return
            return

        self._last_ns = now_ns
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ClockGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
