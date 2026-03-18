"""
AprilTag 检测工具模块
====================
输入: sensor_msgs/Image (ROS 消息)
输出: (标注后的 Image, List[TargetInfo])

用法::

    from duojin01_mission.april_tag_detector import AprilTagDetector

    detector = AprilTagDetector(
        families='tag36h11',          # 识别的 Tag 族，多族用空格分隔
        tag_size=0.05,                # Tag 实体边长，单位米
        calibration_file='/path/to/calibration.yaml',  # 可选，标定后填入
    )
    annotated_msg, targets = detector.detect(ros_image_msg)

    for t in targets:
        if t.tag_detected:
            print(f'id={t.tag_id}  族={t.tag_family}')
            print(f'中心像素 u={t.center_u:.1f}  v={t.center_v:.1f}')
            if t.pose_valid:
                print(f'深度 z={t.pos_z*100:.1f} cm')

依赖安装::

    pip install pupil-apriltags opencv-python pyyaml
"""

import os
import cv2
import numpy as np
import yaml
from typing import List, Optional, Tuple

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage

try:
    from duojin01_msgs.msg import TargetInfo
    _MSGS_AVAILABLE = True
except ImportError:
    _MSGS_AVAILABLE = False

try:
    from pupil_apriltags import Detector as _AprilDetector
    _APRILTAG_AVAILABLE = True
except ImportError:
    _APRILTAG_AVAILABLE = False

_bridge = CvBridge()

# ── 立方体顶点定义（Tag 坐标系，单位与 tag_size 相同）────────────────────
# Tag 坐标系：原点=Tag 中心，z 轴指向摄像机（z>0 朝摄像机，z<0 进入物体表面）
# 立方体：Tag 为顶面（z=0），向物体内延伸一个 tag_size（z=-tag_size）
# 顶点顺序：顶面 0-3（逆时针），底面 4-7（对应顶面各点）
_CUBE_TOP_CORNERS = np.array([
    [-0.5,  0.5, 0.0],   # 0 左上
    [ 0.5,  0.5, 0.0],   # 1 右上
    [ 0.5, -0.5, 0.0],   # 2 右下
    [-0.5, -0.5, 0.0],   # 3 左下
], dtype=np.float32)

_CUBE_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),   # 顶面
    (4, 5), (5, 6), (6, 7), (7, 4),   # 底面
    (0, 4), (1, 5), (2, 6), (3, 7),   # 侧棱
]

_COLOR_TOP  = (0, 200, 255)   # 顶面：橙黄
_COLOR_BOT  = (0, 200, 255)   # 底面：橙黄
_COLOR_SIDE = (0, 255, 100)   # 侧棱：绿


# ── 工具函数 ──────────────────────────────────────────────────────────────

def _rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """旋转矩阵 → 四元数 (x, y, z, w)，Shepperd 方法。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return (float((R[2,1]-R[1,2])*s), float((R[0,2]-R[2,0])*s),
                float((R[1,0]-R[0,1])*s), float(0.25/s))
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return (float(0.25*s), float((R[0,1]+R[1,0])/s),
                float((R[0,2]+R[2,0])/s), float((R[2,1]-R[1,2])/s))
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (float((R[0,1]+R[1,0])/s), float(0.25*s),
                float((R[1,2]+R[2,1])/s), float((R[0,2]-R[2,0])/s))
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return (float((R[0,2]+R[2,0])/s), float((R[1,2]+R[2,1])/s),
                float(0.25*s), float((R[1,0]-R[0,1])/s))


def _build_cube_pts(tag_size: float) -> np.ndarray:
    """生成立方体 8 个顶点（Tag 坐标系，shape=(8,3)）。"""
    top = _CUBE_TOP_CORNERS * tag_size
    bot = top.copy()
    bot[:, 2] = -tag_size
    return np.vstack([top, bot]).astype(np.float32)


def _project_cube(R: np.ndarray, t: np.ndarray,
                  tag_size: float, K: np.ndarray,
                  dist: np.ndarray) -> np.ndarray:
    """
    将立方体 8 顶点从 Tag 坐标系投影到图像像素坐标。
    返回 shape=(8,2) 的浮点数组。
    """
    pts_3d = _build_cube_pts(tag_size)
    rvec, _ = cv2.Rodrigues(R)
    pts_2d, _ = cv2.projectPoints(pts_3d, rvec, t, K, dist)
    return pts_2d.reshape(8, 2)


def _draw_cube(img: np.ndarray, verts: np.ndarray) -> None:
    """在图像上绘制立方体线框。"""
    v = verts.astype(int)
    top_edges  = [(0,1),(1,2),(2,3),(3,0)]
    bot_edges  = [(4,5),(5,6),(6,7),(7,4)]
    side_edges = [(0,4),(1,5),(2,6),(3,7)]
    for i, j in top_edges:
        cv2.line(img, tuple(v[i]), tuple(v[j]), _COLOR_TOP, 2)
    for i, j in bot_edges:
        cv2.line(img, tuple(v[i]), tuple(v[j]), _COLOR_BOT, 2)
    for i, j in side_edges:
        cv2.line(img, tuple(v[i]), tuple(v[j]), _COLOR_SIDE, 2)
    # 绘制底面中心点
    cx = int(np.mean(v[4:, 0]))
    cy = int(np.mean(v[4:, 1]))
    cv2.drawMarker(img, (cx, cy), (0, 100, 255),
                   cv2.MARKER_CROSS, 12, 2)


# ── 主类 ──────────────────────────────────────────────────────────────────

class AprilTagDetector:
    """
    AprilTag 检测工具类（纯 Python 工具，不依赖 ROS 节点）。

    参数
    ----
    families : str
        识别的 Tag 族，多族用空格分隔。
        可用：tag36h11  tag25h9  tag16h5
              tagCircle21h7  tagStandard41h12
        默认：'tag36h11'
    tag_size : float
        Tag 印刷实体边长，单位：米。用于 3D 姿态估算。
        默认：0.05（5 cm）
    calibration_file : str | None
        相机标定 YAML 路径（由 calibrate_camera 工具生成）。
        为 None 时跳过 3D 姿态估算，cube_vertices 和 pose_* 字段无效。
    """

    def __init__(
        self,
        families: str = 'tag36h11',
        tag_size: float = 0.05,
        calibration_file: Optional[str] = None,
    ):
        if not _APRILTAG_AVAILABLE:
            raise RuntimeError(
                'pupil-apriltags 未安装，请运行: pip install pupil-apriltags'
            )

        self._tag_size = tag_size
        self._detector = _AprilDetector(
            families=families,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        self._K: Optional[np.ndarray] = None
        self._dist: Optional[np.ndarray] = None
        self._cam_params: Optional[List[float]] = None  # [fx, fy, cx, cy]

        if calibration_file is not None:
            self.load_calibration(calibration_file)

    # ── 标定加载 ──────────────────────────────────────────────────────────

    def load_calibration(self, path: str) -> None:
        """从标定 YAML 文件加载内参和畸变系数。"""
        with open(os.path.expanduser(path), 'r') as f:
            data = yaml.safe_load(f)
        K_data = data['camera_matrix']['data']
        self._K = np.array(K_data, dtype=np.float64).reshape(3, 3)
        dist_data = data['distortion_coefficients']['data']
        self._dist = np.array(dist_data, dtype=np.float64)
        self._cam_params = [
            float(self._K[0, 0]),   # fx
            float(self._K[1, 1]),   # fy
            float(self._K[0, 2]),   # cx
            float(self._K[1, 2]),   # cy
        ]

    @property
    def calibrated(self) -> bool:
        """是否已加载标定参数。"""
        return self._K is not None

    # ── 核心检测接口 ──────────────────────────────────────────────────────

    def detect(
        self, ros_image: RosImage
    ) -> Tuple[RosImage, List['TargetInfo']]:
        """
        对输入的 ROS Image 进行 AprilTag 检测。

        参数
        ----
        ros_image : sensor_msgs/Image
            原始相机图像。

        返回
        ----
        annotated_image : sensor_msgs/Image
            标注了 Tag 框、中心点、立方体轮廓的图像。
        targets : List[duojin01_msgs/TargetInfo]
            每个检测到的 Tag 对应一个 TargetInfo。
            未检测到任何 Tag 时返回含一个 tag_detected=False 的列表。

        注意
        ----
        - 未加载标定时，pose_valid=False，cube_vertices 为空列表。
        - 支持同时检测多个 Tag 和多个族。
        """
        if not _MSGS_AVAILABLE:
            raise RuntimeError(
                'duojin01_msgs 尚未编译，请先 colcon build --packages-select duojin01_msgs'
            )

        cv_img = _bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        gray   = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        canvas = cv_img.copy()

        detections = self._detector.detect(
            gray,
            estimate_tag_pose=(self._cam_params is not None),
            camera_params=self._cam_params,
            tag_size=self._tag_size,
        )

        # ── 未检测到 Tag ──────────────────────────────────────────────
        if not detections:
            empty = TargetInfo()
            empty.tag_detected = False
            empty.tag_id = -1
            empty.rot_w = 1.0
            annotated = _bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            annotated.header = ros_image.header
            return annotated, [empty]

        # ── 逐个处理检测结果 ──────────────────────────────────────────
        targets: List[TargetInfo] = []

        for det in detections:
            tag_id = int(det.tag_id)
            family = (det.tag_family.decode()
                      if isinstance(det.tag_family, bytes)
                      else str(det.tag_family))
            cx_px = float(det.center[0])
            cy_px = float(det.center[1])
            corners = det.corners  # ndarray (4,2)

            # Tag 高度：左上角到左下角的像素距离
            height_px = float(np.linalg.norm(corners[0] - corners[3]))

            # 绘制 Tag 边框
            pts = corners.astype(int)
            for i in range(4):
                cv2.line(canvas, tuple(pts[i]), tuple(pts[(i+1)%4]),
                         (255, 80, 0), 2)
            # 绘制中心点
            cv2.circle(canvas, (int(cx_px), int(cy_px)), 6, (0, 0, 255), -1)
            # 标注 ID 和族
            cv2.putText(canvas, f'id={tag_id} [{family}]',
                        (int(cx_px)+10, int(cy_px)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # ── 三维姿态估算（需要标定）────────────────────────────────
            cube_u: List[float] = []
            cube_v: List[float] = []
            pose_valid = False
            pos_x = pos_y = pos_z = 0.0
            rot_x = rot_y = rot_z = 0.0
            rot_w = 1.0

            if (self._cam_params is not None
                    and hasattr(det, 'pose_R')
                    and det.pose_R is not None):

                R = np.array(det.pose_R, dtype=np.float64)
                t = np.array(det.pose_t, dtype=np.float64).flatten()

                # 投影立方体到图像
                verts = _project_cube(R, t, self._tag_size,
                                      self._K, self._dist)
                cube_u = [float(v[0]) for v in verts]
                cube_v = [float(v[1]) for v in verts]
                _draw_cube(canvas, verts)

                # 位置
                pos_x, pos_y, pos_z = float(t[0]), float(t[1]), float(t[2])

                # 深度标注
                cv2.putText(canvas, f'z={pos_z*100:.1f}cm',
                            (int(cx_px)+10, int(cy_px)+18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

                # 四元数
                rot_x, rot_y, rot_z, rot_w = _rot_to_quat(R)
                pose_valid = True

            # ── 填充 TargetInfo ────────────────────────────────────────
            info = TargetInfo()
            info.tag_detected      = True
            info.tag_id            = tag_id
            info.tag_family        = family
            info.center_u          = cx_px
            info.center_v          = cy_px
            info.height_px         = height_px
            info.cube_vertices_u   = cube_u
            info.cube_vertices_v   = cube_v
            info.pose_valid        = pose_valid
            info.pos_x             = pos_x
            info.pos_y             = pos_y
            info.pos_z             = pos_z
            info.rot_x             = rot_x
            info.rot_y             = rot_y
            info.rot_z             = rot_z
            info.rot_w             = rot_w

            targets.append(info)

        annotated = _bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        annotated.header = ros_image.header
        return annotated, targets
