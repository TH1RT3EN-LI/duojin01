#!/usr/bin/env python3
import copy
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs.msg import PointField


class OrbbecTopicCompatNode(Node):
    """
    @author litianshun (litianshun.cn@gmail.com)
    @date 2026-02-10
    从Gazebo桥接深度相机的图像到ros
    """

    def __init__(self):
        super().__init__("orbbec_topic_compat_node")

        self.declare_parameter("input_color_image_topic", "/camera_sim/color/image_raw")
        self.declare_parameter("input_color_info_topic", "/camera_sim/color/camera_info")
        self.declare_parameter("input_depth_image_topic", "/camera_sim/depth/image_raw")
        self.declare_parameter("input_depth_info_topic", "/camera_sim/depth/camera_info")
        self.declare_parameter("input_depth_points_topic", "/camera_sim/depth/points")

        self.declare_parameter("output_color_image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_depth_image_topic", "/camera/depth/image_raw")
        self.declare_parameter("output_depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_depth_points_topic", "/camera/depth/points")
        self.declare_parameter("output_ir_image_topic", "/camera/ir/image_raw")
        self.declare_parameter("output_ir_info_topic", "/camera/ir/camera_info")

        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("ir_frame_id", "camera_ir_optical_frame")
        self.declare_parameter("cloud_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("publish_fake_ir", True)
        self.declare_parameter("transform_cloud_to_optical", True)

        self._input_color_image_topic = str(self.get_parameter("input_color_image_topic").value)
        self._input_color_info_topic = str(self.get_parameter("input_color_info_topic").value)
        self._input_depth_image_topic = str(self.get_parameter("input_depth_image_topic").value)
        self._input_depth_info_topic = str(self.get_parameter("input_depth_info_topic").value)
        self._input_depth_points_topic = str(self.get_parameter("input_depth_points_topic").value)

        self._output_color_image_topic = str(self.get_parameter("output_color_image_topic").value)
        self._output_color_info_topic = str(self.get_parameter("output_color_info_topic").value)
        self._output_depth_image_topic = str(self.get_parameter("output_depth_image_topic").value)
        self._output_depth_info_topic = str(self.get_parameter("output_depth_info_topic").value)
        self._output_depth_points_topic = str(self.get_parameter("output_depth_points_topic").value)
        self._output_ir_image_topic = str(self.get_parameter("output_ir_image_topic").value)
        self._output_ir_info_topic = str(self.get_parameter("output_ir_info_topic").value)

        self._color_frame_id = str(self.get_parameter("color_frame_id").value)
        self._depth_frame_id = str(self.get_parameter("depth_frame_id").value)
        self._ir_frame_id = str(self.get_parameter("ir_frame_id").value)
        self._cloud_frame_id = str(self.get_parameter("cloud_frame_id").value)
        self._publish_fake_ir = bool(self.get_parameter("publish_fake_ir").value)
        self._transform_cloud_to_optical = bool(
            self.get_parameter("transform_cloud_to_optical").value
        )
        self._warned_cloud_format = False

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._color_image_pub = self.create_publisher(Image, self._output_color_image_topic, pub_qos)
        self._color_info_pub = self.create_publisher(CameraInfo, self._output_color_info_topic, pub_qos)
        self._depth_image_pub = self.create_publisher(Image, self._output_depth_image_topic, pub_qos)
        self._depth_info_pub = self.create_publisher(CameraInfo, self._output_depth_info_topic, pub_qos)
        self._depth_points_pub = self.create_publisher(PointCloud2, self._output_depth_points_topic, pub_qos)
        self._ir_image_pub = self.create_publisher(Image, self._output_ir_image_topic, pub_qos)
        self._ir_info_pub = self.create_publisher(CameraInfo, self._output_ir_info_topic, pub_qos)

        self.create_subscription(Image, self._input_color_image_topic, self._on_color_image, sub_qos)
        self.create_subscription(CameraInfo, self._input_color_info_topic, self._on_color_info, sub_qos)
        self.create_subscription(Image, self._input_depth_image_topic, self._on_depth_image, sub_qos)
        self.create_subscription(CameraInfo, self._input_depth_info_topic, self._on_depth_info, sub_qos)
        self.create_subscription(PointCloud2, self._input_depth_points_topic, self._on_depth_points, sub_qos)

        self.get_logger().info(
            "orbbec_topic_compat_node started: "
            f"{self._input_color_image_topic} -> {self._output_color_image_topic}, "
            f"{self._input_depth_image_topic} -> {self._output_depth_image_topic}, "
            f"{self._input_depth_points_topic} -> {self._output_depth_points_topic}, "
            f"fake_ir={'on' if self._publish_fake_ir else 'off'}"
        )

    def _on_color_image(self, msg: Image) -> None:
        msg.header.frame_id = self._color_frame_id
        self._color_image_pub.publish(msg)

    def _on_color_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self._color_frame_id
        self._color_info_pub.publish(msg)

    def _on_depth_image(self, msg: Image) -> None:
        msg.header.frame_id = self._depth_frame_id
        self._depth_image_pub.publish(msg)
        if self._publish_fake_ir:
            ir_msg = copy.deepcopy(msg)
            ir_msg.header.frame_id = self._ir_frame_id
            self._ir_image_pub.publish(ir_msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self._depth_frame_id
        self._depth_info_pub.publish(msg)
        if self._publish_fake_ir:
            ir_info = copy.deepcopy(msg)
            ir_info.header.frame_id = self._ir_frame_id
            self._ir_info_pub.publish(ir_info)

    def _on_depth_points(self, msg: PointCloud2) -> None:
        out = msg
        if self._transform_cloud_to_optical:
            out = self._transform_cloud_camera_to_optical(msg)
            if out is None:
                return
        out.header.frame_id = self._cloud_frame_id
        self._depth_points_pub.publish(out)

    def _transform_cloud_camera_to_optical(self, msg: PointCloud2):
        # Convert point coordinates from camera frame (x forward, y left, z up)
        # to optical frame (x right, y down, z forward):
        # x' = -y, y' = -z, z' = x
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = (field.offset, field.datatype)
        required = ("x", "y", "z")
        if not all(name in offsets for name in required):
            if not self._warned_cloud_format:
                self.get_logger().warning("point cloud lacks x/y/z fields; skip cloud republish")
                self._warned_cloud_format = True
            return None

        for name in required:
            _, dtype = offsets[name]
            if dtype != PointField.FLOAT32:
                if not self._warned_cloud_format:
                    self.get_logger().warning(
                        "point cloud x/y/z are not FLOAT32; skip coordinate transform"
                    )
                    self._warned_cloud_format = True
                out = copy.deepcopy(msg)
                return out

        out = copy.deepcopy(msg)
        data = bytearray(out.data)
        x_off = offsets["x"][0]
        y_off = offsets["y"][0]
        z_off = offsets["z"][0]

        height = out.height if out.height > 0 else 1
        width = out.width
        point_step = out.point_step
        row_step = out.row_step if out.row_step > 0 else width * point_step

        for row in range(height):
            row_base = row * row_step
            for col in range(width):
                base = row_base + col * point_step
                x = struct.unpack_from("<f", data, base + x_off)[0]
                y = struct.unpack_from("<f", data, base + y_off)[0]
                z = struct.unpack_from("<f", data, base + z_off)[0]
                x_opt = -y
                y_opt = -z
                z_opt = x
                struct.pack_into("<f", data, base + x_off, x_opt)
                struct.pack_into("<f", data, base + y_off, y_opt)
                struct.pack_into("<f", data, base + z_off, z_opt)

        out.data = bytes(data)
        return out


def main():
    rclpy.init()
    node = OrbbecTopicCompatNode()
    rclpy.spin(node)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
