#!/usr/bin/env python3
from functools import partial
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class E4JointTestBridge(Node):
    def __init__(self) -> None:
        super().__init__("e4_joint_test_bridge")

        self.declare_parameter("command_topic", "/e4_joint_test_cmd")
        self.declare_parameter("trajectory_topic", "/e4_joint_trajectory")
        self.declare_parameter("default_duration_sec", 1.0)
        self.declare_parameter(
            "joint_names",
            ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4"],
        )
        self.declare_parameter(
            "joint_command_topics",
            ["/e4_joint1_cmd", "/e4_joint2_cmd", "/e4_joint3_cmd", "/e4_joint4_cmd"],
        )

        self._command_topic = str(self.get_parameter("command_topic").value)
        self._trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self._default_duration_sec = float(self.get_parameter("default_duration_sec").value)
        self._joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        self._joint_command_topics = [
            str(topic) for topic in self.get_parameter("joint_command_topics").value
        ]
        self._targets = [0.0] * len(self._joint_names)

        if len(self._joint_names) != len(self._joint_command_topics):
            raise ValueError("joint_names and joint_command_topics must have the same length")

        self._trajectory_pub = self.create_publisher(JointTrajectory, self._trajectory_topic, 10)
        self._full_sub = self.create_subscription(
            Float64MultiArray,
            self._command_topic,
            self._on_full_command,
            10,
        )
        self._joint_subs = []
        for index, topic in enumerate(self._joint_command_topics):
            self._joint_subs.append(
                self.create_subscription(
                    Float64,
                    topic,
                    partial(self._on_single_command, index),
                    10,
                )
            )

        self.get_logger().info(
            f"E4 joint test bridge started: full={self._command_topic}; trajectory={self._trajectory_topic}; joint_names={self._joint_names}"
        )
        self.get_logger().info(
            f"Single-joint topics will update the target vector: {self._joint_command_topics}"
        )

    def _parse_full_command(self, data: Sequence[float]) -> Tuple[List[float], float]:
        joint_count = len(self._joint_names)
        if len(data) == joint_count:
            return list(data), self._default_duration_sec
        if len(data) == joint_count + 1:
            return list(data[:joint_count]), float(data[-1])
        raise ValueError(f"expected {joint_count} or {joint_count + 1} values, got {len(data)}")

    def _duration_msg(self, seconds: float) -> Duration:
        clamped = max(0.05, float(seconds))
        whole = int(clamped)
        nanos = int(round((clamped - whole) * 1_000_000_000))
        if nanos >= 1_000_000_000:
            whole += 1
            nanos -= 1_000_000_000
        return Duration(sec=whole, nanosec=nanos)

    def _publish_targets(self, duration_sec: float, reason: str) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(self._targets)
        point.time_from_start = self._duration_msg(duration_sec)
        msg.points = [point]

        self._trajectory_pub.publish(msg)
        formatted_targets = ", ".join(
            f"{joint_name}={target:.3f}"
            for joint_name, target in zip(self._joint_names, self._targets)
        )
        self.get_logger().info(
            f"published E4 trajectory ({reason}, {duration_sec:.2f}s): {formatted_targets}"
        )

    def _on_full_command(self, msg: Float64MultiArray) -> None:
        try:
            positions, duration_sec = self._parse_full_command(list(msg.data))
        except ValueError as exc:
            self.get_logger().warn(f"ignored E4 full command: {exc}")
            return

        for index, value in enumerate(positions):
            self._targets[index] = float(value)
        self._publish_targets(duration_sec, "full command")

    def _on_single_command(self, index: int, msg: Float64) -> None:
        self._targets[index] = float(msg.data)
        self._publish_targets(
            self._default_duration_sec,
            f"single joint update: {self._joint_names[index]}",
        )


def main() -> None:
    rclpy.init()
    node = E4JointTestBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
