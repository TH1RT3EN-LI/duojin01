#!/usr/bin/env python3
import copy
import math

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class SimMonoCameraCompatNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_mono_camera_compat_node")

        self.declare_parameter("input_image_topic", "/camera_sim/arm_mono/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera_sim/arm_mono/camera_info")
        self.declare_parameter("output_image_topic", "/camera/image_raw")
        self.declare_parameter("output_camera_info_topic", "/camera/camera_info")
        self.declare_parameter("output_frame_id", "camera_optical_frame")
        self.declare_parameter("output_encoding", "bgr8")
        self.declare_parameter("default_distortion_model", "plumb_bob")
        self.declare_parameter("default_horizontal_fov_rad", 1.5707)
        self.declare_parameter("lazy_input_subscription", True)

        self._input_image_topic = str(self.get_parameter("input_image_topic").value)
        self._input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        self._output_image_topic = str(self.get_parameter("output_image_topic").value)
        self._output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)
        self._output_frame_id = str(self.get_parameter("output_frame_id").value)
        self._output_encoding = str(self.get_parameter("output_encoding").value)
        self._default_distortion_model = str(self.get_parameter("default_distortion_model").value)
        self._default_horizontal_fov_rad = float(self.get_parameter("default_horizontal_fov_rad").value)
        self._lazy_input_subscription = bool(self.get_parameter("lazy_input_subscription").value)
        self._latest_camera_info: CameraInfo | None = None
        self._warned_missing_camera_info = False
        self._logged_default_intrinsics = False

        self._bridge = CvBridge()

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub_qos = sub_qos
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._image_pub = self.create_publisher(Image, self._output_image_topic, pub_qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, self._output_camera_info_topic, pub_qos)

        self._image_sub = None
        self._camera_info_sub = None
        self._subscriptions_active = False
        if self._lazy_input_subscription:
            self._subscription_watchdog = self.create_timer(0.5, self._refresh_input_subscriptions)
            self._refresh_input_subscriptions()
        else:
            self._subscription_watchdog = None
            self._ensure_input_subscriptions()

        self.get_logger().info(
            "sim_mono_camera_compat_node started: "
            f"{self._input_image_topic} -> {self._output_image_topic}, "
            f"{self._input_camera_info_topic} -> {self._output_camera_info_topic}, "
            f"frame_id={self._output_frame_id}, encoding={self._output_encoding}, "
            f"lazy_input_subscription={self._lazy_input_subscription}"
        )

    def _has_output_subscribers(self) -> bool:
        return (self._image_pub.get_subscription_count() + self._camera_info_pub.get_subscription_count()) > 0

    def _ensure_input_subscriptions(self) -> None:
        if self._image_sub is None:
            self._image_sub = self.create_subscription(Image, self._input_image_topic, self._on_image, self._sub_qos)
        if self._camera_info_sub is None:
            self._camera_info_sub = self.create_subscription(
                CameraInfo, self._input_camera_info_topic, self._on_camera_info, self._sub_qos
            )
        if not self._subscriptions_active:
            self._subscriptions_active = True
            self.get_logger().info("sim_mono_camera_compat: upstream subscriptions activated")

    def _release_input_subscriptions(self) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._camera_info_sub is not None:
            self.destroy_subscription(self._camera_info_sub)
            self._camera_info_sub = None
        if self._subscriptions_active:
            self._subscriptions_active = False
            self.get_logger().info("sim_mono_camera_compat: upstream subscriptions suspended (no downstream subscribers)")

    def _refresh_input_subscriptions(self) -> None:
        if not self._lazy_input_subscription:
            return
        if self._has_output_subscribers():
            self._ensure_input_subscriptions()
        else:
            self._release_input_subscriptions()

    def _on_image(self, msg: Image) -> None:
        if not self._has_output_subscribers():
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding=self._output_encoding)
            out = self._bridge.cv2_to_imgmsg(cv_image, encoding=self._output_encoding)
            out.header = msg.header
            out.header.frame_id = self._output_frame_id
            self._image_pub.publish(out)
            self._camera_info_pub.publish(self._build_camera_info(out))
        except CvBridgeError as exc:
            self.get_logger().warning(f"failed to convert mono camera image to {self._output_encoding}: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = copy.deepcopy(msg)

    def _has_valid_intrinsics(self, msg: CameraInfo) -> bool:
        return (
            len(msg.k) == 9
            and float(msg.k[0]) > 0.0
            and float(msg.k[4]) > 0.0
            and len(msg.p) >= 12
            and float(msg.p[0]) > 0.0
            and float(msg.p[5]) > 0.0
        )

    def _fill_default_intrinsics(self, camera_info: CameraInfo, image_msg: Image) -> None:
        width = max(1, int(camera_info.width or image_msg.width))
        height = max(1, int(camera_info.height or image_msg.height))
        hfov = max(1e-6, float(self._default_horizontal_fov_rad))
        fx = width / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        cx = (width - 1.0) / 2.0
        cy = (height - 1.0) / 2.0

        camera_info.width = width
        camera_info.height = height
        camera_info.k = [
            fx,
            0.0,
            cx,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.p = [
            fx,
            0.0,
            cx,
            0.0,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        camera_info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.d = [0.0] * 5
        camera_info.distortion_model = self._default_distortion_model

        if not self._logged_default_intrinsics:
            self.get_logger().info(
                "using synthesized ideal mono camera intrinsics: "
                f"width={width}, height={height}, hfov={hfov:.4f} rad, "
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )
            self._logged_default_intrinsics = True

    def _build_camera_info(self, image_msg: Image) -> CameraInfo:
        if self._latest_camera_info is None:
            if not self._warned_missing_camera_info:
                self.get_logger().info(
                    "no upstream mono camera_info received from Gazebo; publishing synthesized ideal camera_info"
                )
                self._warned_missing_camera_info = True
            out = CameraInfo()
            out.width = int(image_msg.width)
            out.height = int(image_msg.height)
            out.distortion_model = self._default_distortion_model
            out.d = [0.0] * 5
            out.k = [0.0] * 9
            out.r = [0.0] * 9
            out.p = [0.0] * 12
        else:
            out = copy.deepcopy(self._latest_camera_info)
            if out.width == 0:
                out.width = int(image_msg.width)
            if out.height == 0:
                out.height = int(image_msg.height)
            if not out.distortion_model:
                out.distortion_model = self._default_distortion_model
            if not out.d:
                out.d = [0.0] * 5
            if not out.r:
                out.r = [0.0] * 9
            if not out.k:
                out.k = [0.0] * 9
            if not out.p:
                out.p = [0.0] * 12

        if not self._has_valid_intrinsics(out):
            self._fill_default_intrinsics(out, image_msg)

        out.header.stamp = image_msg.header.stamp
        out.header.frame_id = self._output_frame_id
        return out


def main() -> None:
    rclpy.init()
    node = SimMonoCameraCompatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
