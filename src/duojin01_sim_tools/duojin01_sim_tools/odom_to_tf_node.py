#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTfNode(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-01-30
    
    发布 TF odom->base_*

    由于底盘驱动 duojin01_base_drive 发布 /odom Topic, 但不广播 TF 变换(因为要用 EKF 融合里程和IMU数据)，而仿真中为了提高性能省去了 EKF 的环节；
    为了让 Nav2 可以使用完整的 TF 树，编写本节点来手动进行变换
    """

    def __init__(self) -> None:
        super().__init__("odom_to_tf")

        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_footprint")
        # TF 频率和 /odom 频率解耦，提高性能
        self.declare_parameter("publish_hz", 50.0)

        self._topic = str(self.get_parameter("input_odom_topic").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self._tf = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._topic, self._on_odom, qos)

        self._latest_odom: Odometry | None = None
        self._last_pub_stamp_ns: int | None = None

        publish_period = 1.0 / max(1e-3, self._publish_hz)
        self._timer = self.create_timer(publish_period, self._on_timer)

        self.get_logger().info(
            f"odom_to_tf started: odom='{self._topic}' -> TF {self._odom_frame_id}->{self._child_frame_id} "
            f"@{self._publish_hz:.1f}Hz (BEST_EFFORT input)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_timer(self) -> None:
        msg = self._latest_odom
        if msg is None:
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_pub_stamp_ns is not None and stamp_ns <= self._last_pub_stamp_ns:
            return
        self._last_pub_stamp_ns = stamp_ns

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._odom_frame_id
        tf.child_frame_id = self._child_frame_id
        tf.transform.translation.x = float(msg.pose.pose.position.x)
        tf.transform.translation.y = float(msg.pose.pose.position.y)
        tf.transform.translation.z = float(msg.pose.pose.position.z)
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
