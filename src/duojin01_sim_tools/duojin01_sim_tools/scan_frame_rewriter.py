#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameRewriter(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-01-31
    
    Gazebo 的 scan 的默认 frame 与所需不符，编写该 node 中继，修改 frame_id 后重新发布
    """
    def __init__(self):
        super().__init__('scan_frame_rewriter')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_frame_id', 'laser')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._output_frame_id = str(self.get_parameter('output_frame_id').value)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(LaserScan, input_topic, self.cb, sub_qos)
        self.pub = self.create_publisher(LaserScan, output_topic, pub_qos)

        self.get_logger().info(
            f"scan_frame_rewriter started: {input_topic} (BEST_EFFORT) -> {output_topic} (RELIABLE), frame_id='{self._output_frame_id}'"
        )

    def cb(self, msg):
        msg.header.frame_id = self._output_frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameRewriter()
    rclpy.spin(node)
    rclpy.try_shutdown()
    
if __name__ == "__main__":
    main()
