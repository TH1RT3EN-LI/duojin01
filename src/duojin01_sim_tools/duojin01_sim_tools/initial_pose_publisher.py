#!/usr/bin/env python3
import math
import time

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
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)    # init x
        self.declare_parameter("y", 0.0)    # init y
        self.declare_parameter("yaw", 0.0)  # init yaw 
        self.declare_parameter("delay_sec", 1.0)
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("require_subscriber", True)
        self.declare_parameter("wait_for_amcl_pose", True)
        self.declare_parameter("max_wait_sec", 120.0)
        self.declare_parameter("status_log_period_sec", 2.0)

        self._topic = str(self.get_parameter("topic").value)
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._yaw = float(self.get_parameter("yaw").value)

        self._publish_count = max(1, int(self.get_parameter("publish_count").value))
        self._period = float(self.get_parameter("publish_period_sec").value)
        delay = float(self.get_parameter("delay_sec").value)
        self._require_subscriber = bool(self.get_parameter("require_subscriber").value)
        self._wait_for_amcl_pose = bool(self.get_parameter("wait_for_amcl_pose").value)
        self._max_wait_sec = float(self.get_parameter("max_wait_sec").value)
        self._status_log_period_sec = max(0.1, float(self.get_parameter("status_log_period_sec").value))

        self._published_count = 0
        self._finished_initial_burst = False
        self._amcl_pose_received = False
        self._done = False
        self._start_monotonic = time.monotonic()
        self._last_status_log_monotonic = self._start_monotonic

        self._pub = self.create_publisher(PoseWithCovarianceStamped, self._topic, 10)
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_pose_topic,
            self._on_amcl_pose,
            10,
        )

        self._timer = self.create_timer(delay, self._start_publishing)

        self.get_logger().info(
            f"initial_pose_publisher armed: will publish to {self._topic} in frame '{self._frame_id}' "
            f"after {delay:.2f}s (x={self._x:.3f}, y={self._y:.3f}, yaw={self._yaw:.3f}, "
            f"initial_burst={self._publish_count}, wait_for_amcl_pose={self._wait_for_amcl_pose})"
        )

    @property
    def done(self) -> bool:
        return self._done

    def _log_status(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_status_log_monotonic >= self._status_log_period_sec:
            self.get_logger().info(message)
            self._last_status_log_monotonic = now

    def _mark_done(self, message: str) -> None:
        if self._done:
            return
        self._done = True
        self.get_logger().info(message)
        if self._timer is not None:
            self._timer.cancel()

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._done or self._amcl_pose_received:
            return
        self._amcl_pose_received = True
        self._mark_done("AMCL pose received; initial pose accepted, shutting down.")

    def _start_publishing(self):
        self._timer.cancel()
        self._timer = self.create_timer(self._period, self._publish_once)
        self.get_logger().info("initial_pose_publisher active; waiting for Nav2/AMCL to consume the pose.")

    def _publish_once(self):
        if self._done:
            return

        if self._max_wait_sec > 0.0 and time.monotonic() - self._start_monotonic > self._max_wait_sec:
            self._mark_done(
                f"Timed out after {self._max_wait_sec:.1f}s waiting for AMCL to accept the initial pose."
            )
            return

        if self._require_subscriber and self.count_subscribers(self._topic) == 0:
            self._log_status(f"Waiting for a subscriber on {self._topic} before publishing the initial pose...")
            return

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
        self._published_count += 1

        if not self._finished_initial_burst and self._published_count >= self._publish_count:
            self._finished_initial_burst = True
            if self._wait_for_amcl_pose:
                self.get_logger().info(
                    f"Published initial burst ({self._publish_count}x); continuing retries until {self._amcl_pose_topic} arrives."
                )
            else:
                self._mark_done("Initial pose burst published; shutting down.")
                return

        if self._wait_for_amcl_pose and not self._amcl_pose_received:
            self._log_status(
                f"Published {self._published_count} initial pose message(s); still waiting for {self._amcl_pose_topic}..."
            )


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
