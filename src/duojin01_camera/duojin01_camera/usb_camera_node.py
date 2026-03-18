import os
import cv2
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        self.declare_parameter('device_id', 3)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('calibration_file', '')   # 标定 YAML 路径，空字符串=未标定

        device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        calibration_file = self.get_parameter('calibration_file').value

        self._calib = self._load_calibration(calibration_file)

        # 强制使用 V4L2 后端，避免 GStreamer pipeline 失败
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        # 使用 MJPG 格式：1280x720 下 YUYV 只有 9fps，MJPG 支持 30fps
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera device {device_id}')
            raise RuntimeError(f'Cannot open camera device {device_id}')

        self.bridge = CvBridge()

        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'USB camera opened: device={device_id}, {self.width}x{self.height}@{fps}fps')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.pub_image.publish(img_msg)

        info_msg = self._build_camera_info(now)
        self.pub_info.publish(info_msg)

    def _load_calibration(self, path: str):
        """从 YAML 加载标定参数，返回 dict 或 None（未配置时）。"""
        if not path:
            self.get_logger().warn('未指定标定文件，camera_info 将发布零内参')
            return None
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().error(f'标定文件不存在: {path}')
            return None
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.get_logger().info(f'已加载标定文件: {path}')
            return data
        except Exception as e:
            self.get_logger().error(f'读取标定文件失败: {e}')
            return None

    def _build_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'

        if self._calib is not None:
            msg.k = self._calib['camera_matrix']['data']
            msg.d = self._calib['distortion_coefficients']['data']
            msg.r = self._calib['rectification_matrix']['data']
            msg.p = self._calib['projection_matrix']['data']
        else:
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [0.0] * 9
            msg.r = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
            msg.p = [0.0] * 12
        return msg

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
