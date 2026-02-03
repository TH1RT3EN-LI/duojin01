#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateStampFix(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-01-29   
     
    确保 /joint_state 拥有单调递增的时间戳

    由于 robot_state_publisher 节点是利用消息中的时间戳来发布 TF 坐标变换的，一旦出现时间倒退，tf2 就会清空其缓存，这会导致 SLAM 或 Nav2 运行中断，并在 Foxglove 中表现为机器人位姿的抽搐现象。
    """

    def __init__(self) -> None:
        super().__init__("joint_state_stamp_fix")
        # ROS Topic /joint_states_raw 定义于 src/duojin01_bringup/config/ros_gz_bridge.yaml，将 Gazebo 的 joint_state 先引流到 /joint_states_raw，经过该 node 清洗后发布到 /joint_states
        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)

        in_qos = QoSProfile(depth=50)
        in_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        in_qos.durability = DurabilityPolicy.VOLATILE

        self._pub = self.create_publisher(JointState, self._output_topic, 20)
        self._sub = self.create_subscription(JointState, self._input_topic, self._on_msg, in_qos)

        self.get_logger().info(f"joint_state_stamp_fix started: {self._input_topic} -> {self._output_topic}")

    def _on_msg(self, msg: JointState) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateStampFix()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

