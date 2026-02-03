#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-02-01
    在导航初始化时将车辆的 initial_pose 发布出去(连续发布 {publish_count} 次，确保送达)，避免需要在 rviz 手动初始化
    """
    def __init__(self):
        super().__init__("initial_pose_publisher")

        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)    # init x
        self.declare_parameter("y", 0.0)    # init y
        self.declare_parameter("yaw", 0.0)  # init yaw 
        self.declare_parameter("delay_sec", 1.0)
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("publish_period_sec", 0.2)

        topic = str(self.get_parameter("topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._yaw = float(self.get_parameter("yaw").value)

        self._remaining = int(self.get_parameter("publish_count").value)
        self._period = float(self.get_parameter("publish_period_sec").value)
        delay = float(self.get_parameter("delay_sec").value)

        self._pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        self._timer = self.create_timer(delay, self._start_publishing)

        self.get_logger().info(
            f"initial_pose_publisher armed: will publish {self._remaining}x to {topic} in frame '{self._frame_id}' "
            f"after {delay:.2f}s (x={self._x:.3f}, y={self._y:.3f}, yaw={self._yaw:.3f})"
        )

    def _start_publishing(self):
        self._timer.cancel()
        self._timer = self.create_timer(self._period, self._publish_once)

    def _publish_once(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        
        half = 0.5 * self._yaw
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(math.sin(half))
        msg.pose.pose.orientation.w = float(math.cos(half))

        # Conservative covariance
        # 单位：x, y -- 米; yaw -- 弧度.
        cov = [0.0] * 36
        cov[0] = 0.25  # x
        cov[7] = 0.25  # y
        cov[35] = math.radians(10.0) ** 2  # yaw
        msg.pose.covariance = cov

        self._pub.publish(msg)
        self._remaining -= 1

        if self._remaining <= 0:
            self.get_logger().info("initial_pose_publisher done; shutting down.")
            rclpy.try_shutdown()


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
