#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64, Float64MultiArray


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a one-shot test command to the four E4 arm joints."
    )
    parser.add_argument(
        "values",
        nargs="*",
        type=float,
        help="either 4 target angles for arm_joint1..4, or 1 target angle when used with --joint",
    )
    parser.add_argument(
        "--joint",
        type=int,
        choices=[1, 2, 3, 4],
        help="send a command to only one joint instead of all four",
    )
    parser.add_argument(
        "--deg",
        action="store_true",
        help="interpret the four joint values as degrees and convert them to radians before publishing",
    )
    parser.add_argument(
        "--time",
        type=float,
        default=1.0,
        help="trajectory duration in seconds; default: 1.0",
    )
    parser.add_argument(
        "--topic",
        default="/e4_joint_test_cmd",
        help="ROS topic for 4-joint absolute commands; default: /e4_joint_test_cmd",
    )
    parser.add_argument(
        "--joint-topic-prefix",
        default="/e4_joint",
        help="prefix for single-joint command topics; default: /e4_joint",
    )
    parser.add_argument(
        "--wait-for-subscriber",
        type=float,
        default=2.0,
        help="seconds to wait for the test bridge subscriber before publishing; default: 2.0",
    )
    return parser.parse_args(remove_ros_args(args=None)[1:])


class E4JointTestCli(Node):
    def __init__(self, topic: str, joint_topic_prefix: str) -> None:
        super().__init__("e4_joint_test_cli")
        self._vector_pub = self.create_publisher(Float64MultiArray, topic, 10)
        self._joint_topics = {
            index: f"{joint_topic_prefix}{index}_cmd"
            for index in range(1, 5)
        }
        self._single_pubs = {
            index: self.create_publisher(Float64, self._joint_topics[index], 10)
            for index in range(1, 5)
        }

    @property
    def vector_publisher(self):
        return self._vector_pub

    def single_publisher(self, joint_index: int):
        return self._single_pubs[joint_index]

    def single_topic(self, joint_index: int) -> str:
        return self._joint_topics[joint_index]


def main() -> None:
    args = _parse_args()

    rclpy.init()
    node = E4JointTestCli(args.topic, args.joint_topic_prefix)
    try:
        deadline = time.monotonic() + max(0.0, float(args.wait_for_subscriber))

        if args.joint is not None:
            if len(args.values) != 1:
                raise SystemExit("single-joint mode requires exactly 1 value; example: --joint 1 --deg 30")

            value = args.values[0]
            if args.deg:
                value = math.radians(value)

            publisher = node.single_publisher(args.joint)
            while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
                rclpy.spin_once(node, timeout_sec=0.1)

            msg = Float64()
            msg.data = float(value)

            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

            topic = node.single_topic(args.joint)
            print(f"Published E4 single-joint command to {topic}: joint{args.joint}={value:.3f} rad")
            if publisher.get_subscription_count() == 0:
                print("Warning: no subscriber was observed on the topic while publishing.")
            return

        if len(args.values) != 4:
            raise SystemExit("4-joint mode requires exactly 4 values; example: --deg 0 30 -45 15")

        positions = list(args.values)
        if args.deg:
            positions = [math.radians(value) for value in positions]

        msg = Float64MultiArray()
        msg.data = positions + [float(args.time)]

        while time.monotonic() < deadline and node.vector_publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.vector_publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

        formatted_positions = ", ".join(f"{position:.3f}" for position in positions)
        print(f"Published E4 full command to {args.topic}: [{formatted_positions}] rad in {args.time:.2f}s")
        if node.vector_publisher.get_subscription_count() == 0:
            print("Warning: no subscriber was observed on the topic while publishing.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
