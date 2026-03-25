#!/usr/bin/env python3
"""
取放任务 Demo 脚本
-----------------
流程：
  1. 发布导航目标 → 等待到达
  2. 回零机械臂
  3. 移动到取件位置 + 吸气
  4. 触发摄像头抓帧 → AprilTag 检测 → 发布标注图
  5. 根据检测结果决定抓取坐标
  6. 提起 → 导航到放件点 → 放下
  7. 导航回原点

用法：
  ros2 run duojin01_mission pick_and_place_demo
  ros2 run duojin01_mission pick_and_place_demo \\
      --ros-args -p calibration_file:=~/calibration.yaml -p tag_size:=0.05
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from duojin01_mission.april_tag_detector import AprilTagDetector
from duojin01_msgs.msg import TargetInfo


# ── 任务参数（根据实际环境修改）──────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────


def make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')

        # ── ROS 参数 ──────────────────────────────────────────────────────
        self.declare_parameter('calibration_file', '')   # 标定 YAML 路径
        self.declare_parameter('tag_families',     'tag36h11')
        self.declare_parameter('tag_size',         0.05)  # Tag 边长，米

        calib_file  = self.get_parameter('calibration_file').value
        tag_families = self.get_parameter('tag_families').value
        tag_size     = self.get_parameter('tag_size').value

        # ── AprilTag 检测器 ───────────────────────────────────────────────
        self._detector = AprilTagDetector(
            families=tag_families,
            tag_size=tag_size,
            calibration_file=calib_file if calib_file else None,
        )
        if self._detector.calibrated:
            self.get_logger().info('[demo] 检测器已加载标定参数，3D 姿态估算已启用')
        else:
            self.get_logger().warn('[demo] 未加载标定参数，pose_valid 将为 False')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 输出（发给 mission_executor_node）────────────────────────────
        self.pub_nav         = self.create_publisher(PoseStamped, '/mission/nav_goal',        qos)
        self.pub_gcode       = self.create_publisher(String,      '/mission/gcode_cmd',       qos)
        self.pub_capture     = self.create_publisher(Bool,        '/mission/capture_trigger', qos)
        # 检测后标注图发布到此话题（可用 rqt_image_view 查看）
        self.pub_detected    = self.create_publisher(Image,       '/mission/image_detected',  qos_latched)

        # ── 输入（从 mission_executor_node 接收结果）──────────────────────
        self.create_subscription(Bool,  '/mission/nav_result',    self._on_nav_result,  qos)
        self.create_subscription(Bool,  '/mission/gcode_result',  self._on_gcode_result, qos)
        self.create_subscription(Image, '/mission/image_capture', self._on_image,        qos)

        # ── 同步原语 ──────────────────────────────────────────────────────
        self._nav_event   = threading.Event()
        self._nav_ok      = False
        self._gcode_event = threading.Event()
        self._gcode_ok    = False
        self._image_event = threading.Event()
        self._last_image: Optional[Image] = None   # 最近一帧原始图像

        threading.Thread(target=self._run_mission, daemon=True).start()

    # ── 回调 ──────────────────────────────────────────────────────────────

    def _on_nav_result(self, msg: Bool):
        self._nav_ok = msg.data
        self._nav_event.set()

    def _on_gcode_result(self, msg: Bool):
        self._gcode_ok = msg.data
        self._gcode_event.set()

    def _on_image(self, msg: Image):
        self._last_image = msg      # 保存原始帧
        self._image_event.set()

    # ── 阻塞式辅助方法 ────────────────────────────────────────────────────

    def nav_to(self, x: float, y: float, yaw: float = 0.0,
               timeout: float = 60.0) -> bool:
        """发布导航目标，阻塞直到到达或超时。"""
        self._nav_event.clear()
        pose = make_pose(x, y, yaw)
        pose.header.stamp = self.get_clock().now().to_msg()
        self.pub_nav.publish(pose)
        self.get_logger().info(f'[demo] 导航 → ({x}, {y})')
        if not self._nav_event.wait(timeout):
            self.get_logger().error('[demo] 导航超时')
            return False
        return self._nav_ok

    def gcode(self, cmd: str, timeout: float = 10.0) -> bool:
        """发送 G 代码，阻塞等待 ok 响应。"""
        self._gcode_event.clear()
        msg = String()
        msg.data = cmd
        self.pub_gcode.publish(msg)
        self.get_logger().info(f'[demo] G代码: {cmd}')
        if not self._gcode_event.wait(timeout):
            self.get_logger().error(f'[demo] G代码超时: {cmd}')
            return False
        return self._gcode_ok

    def capture(
        self, timeout: float = 5.0
    ) -> Tuple[Optional[Image], List[TargetInfo]]:
        """
        触发摄像头抓帧，运行 AprilTag 检测，发布标注图。

        返回
        ----
        raw_image : sensor_msgs/Image | None
            原始未标注的 ROS Image 消息；超时则为 None。
        targets : List[duojin01_msgs/TargetInfo]
            每个检测到的 Tag 对应一个 TargetInfo。
            targets[i].tag_detected == False 表示该帧无 Tag。

        TargetInfo 字段说明
        -------------------
        tag_detected  bool     是否检测到 Tag
        tag_id        int32    Tag ID
        tag_family    string   族，例如 "tag36h11"
        center_u/v    float32  中心点像素坐标
        height_px     float32  Tag 在图像中的像素高度（角点间距）
        cube_vertices_u/v  []  立方体 8 顶点投影像素坐标
        pose_valid    bool     3D 姿态是否有效（需标定）
        pos_x/y/z     float32  Tag 在摄像机坐标系下的位置（米）
                               pos_z = 距摄像机的物理深度（即"高度"）
        rot_x/y/z/w   float32  旋转四元数
        """
        self._image_event.clear()
        self._last_image = None

        trigger = Bool()
        trigger.data = True
        self.pub_capture.publish(trigger)
        self.get_logger().info('[demo] 触发摄像头抓帧')

        if not self._image_event.wait(timeout):
            self.get_logger().error('[demo] 摄像头抓帧超时')
            return None, []

        raw_image = self._last_image

        # ── AprilTag 检测 ──────────────────────────────────────────────
        annotated, targets = self._detector.detect(raw_image)

        # ── 发布标注图（可用 rqt_image_view 订阅 /mission/image_detected 查看）
        self.pub_detected.publish(annotated)

        # ── 打印检测摘要 ───────────────────────────────────────────────
        detected = [t for t in targets if t.tag_detected]
        if detected:
            for t in detected:
                depth_str = (f'  深度={t.pos_z*100:.1f}cm' if t.pose_valid else '')
                self.get_logger().info(
                    f'[demo] Tag id={t.tag_id} [{t.tag_family}]'
                    f'  中心=({t.center_u:.0f},{t.center_v:.0f})'
                    f'{depth_str}'
                )
        else:
            self.get_logger().info('[demo] 未检测到 AprilTag')

        return raw_image, targets

    # ── 主任务逻辑 ────────────────────────────────────────────────────────

    def _run_mission(self):

        height = 185
        adj_x = 0
        adj_y = 0

        time.sleep(2.0)   # 等节点完全就绪
        self.get_logger().info('===== 任务开始 =====')

        # 机械臂回零
        if not self.gcode('$h\n\t'):
            self.get_logger().error('回零失败，任务终止')
        time.sleep(10)

        # 拍照 + AprilTag 检测
        #    capture() 返回 (原始Image, List[TargetInfo])
        #    标注图已自动发布到 /mission/image_detected
        raw_img, targets = self.capture()
        detected = [t for t in targets if t.tag_detected]
        if detected:
            best = detected[0]   # 取第一个检测到的 Tag
            """
            tag_detected  bool     是否检测到 Tag
            tag_id        int32    Tag ID
            tag_family    string   族，例如 "tag36h11"
            center_u/v    float32  中心点像素坐标
            height_px     float32  Tag 在图像中的像素高度（角点间距）
            cube_vertices_u/v  []  立方体 8 顶点投影像素坐标
            pose_valid    bool     3D 姿态是否有效（需标定）
            pos_x/y/z     float32  Tag 在摄像机坐标系下的位置（米）
                                pos_z = 距摄像机的物理深度（即"高度"）
            rot_x/y/z/w   float32  旋转四元数
            """
            self.get_logger().info(
                f'[demo] 使用 Tag id={best.tag_id} 指导抓取\n'
                f'[demo] Tag id={best.tag_id} 的族为 {best.tag_family}\n'
                f'[demo] 中心点像素坐标({best.center_u} , {best.center_v}\n)'
            )
            # 根据 best.center_u/center_v 调整机械臂坐标（此处为占位示例）
            adj_x = 0 * best.center_u + 0 * best.center_v + 10
            adj_y = 0 * best.center_u + 0 * best.center_v + 10

            # 下降
            if not self.gcode(f'M20 G91 Z{-height:.1f}'):
                return
            time.sleep(3)
            self.gcode(f'M20 G90 X{adj_x:.1f} Y{adj_y:.1f}')
            time.sleep(3)
            
        
        # 吸气
        if not self.gcode('M3 S1000'):
            return
        time.sleep(3)

        # 上提到原点
        self.gcode(f'M20 G90 X{-adj_x:.1f} Y{-adj_y:.1f} Z{height:.1f}')
        time.sleep(3)
        
        self.get_logger().info('===== 任务完成 =====')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
