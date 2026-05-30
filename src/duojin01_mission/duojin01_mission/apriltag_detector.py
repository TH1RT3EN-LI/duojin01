from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage

try:
    from pupil_apriltags import Detector as _AprilDetector
except ImportError as exc:  # pragma: no cover - runtime dependency check
    _AprilDetector = None
    _APRILTAG_IMPORT_ERROR = exc
else:
    _APRILTAG_IMPORT_ERROR = None


_bridge = CvBridge()

_CUBE_TOP_CORNERS = np.array(
    [
        [-0.5, 0.5, 0.0],
        [0.5, 0.5, 0.0],
        [0.5, -0.5, 0.0],
        [-0.5, -0.5, 0.0],
    ],
    dtype=np.float32,
)
_COLOR_TOP = (0, 200, 255)
_COLOR_BOT = (0, 200, 255)
_COLOR_SIDE = (0, 255, 100)


@dataclass
class DetectedTag:
    tag_id: int
    tag_family: str
    center_u: float
    center_v: float
    height_px: float
    pose_valid: bool = False
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    rot_x: float = 0.0
    rot_y: float = 0.0
    rot_z: float = 0.0
    rot_w: float = 1.0
    cube_vertices_u: List[float] = field(default_factory=list)
    cube_vertices_v: List[float] = field(default_factory=list)


def _rot_to_quat(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        return (
            float((rotation[2, 1] - rotation[1, 2]) * scale),
            float((rotation[0, 2] - rotation[2, 0]) * scale),
            float((rotation[1, 0] - rotation[0, 1]) * scale),
            float(0.25 / scale),
        )
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        return (
            float(0.25 * scale),
            float((rotation[0, 1] + rotation[1, 0]) / scale),
            float((rotation[0, 2] + rotation[2, 0]) / scale),
            float((rotation[2, 1] - rotation[1, 2]) / scale),
        )
    if rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        return (
            float((rotation[0, 1] + rotation[1, 0]) / scale),
            float(0.25 * scale),
            float((rotation[1, 2] + rotation[2, 1]) / scale),
            float((rotation[0, 2] - rotation[2, 0]) / scale),
        )
    scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
    return (
        float((rotation[0, 2] + rotation[2, 0]) / scale),
        float((rotation[1, 2] + rotation[2, 1]) / scale),
        float(0.25 * scale),
        float((rotation[1, 0] - rotation[0, 1]) / scale),
    )


def _build_cube_points(tag_size: float) -> np.ndarray:
    top = _CUBE_TOP_CORNERS * tag_size
    bottom = top.copy()
    bottom[:, 2] = -tag_size
    return np.vstack([top, bottom]).astype(np.float32)


def _project_cube(
    rotation: np.ndarray,
    translation: np.ndarray,
    tag_size: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    points_3d = _build_cube_points(tag_size)
    rvec, _ = cv2.Rodrigues(rotation)
    points_2d, _ = cv2.projectPoints(points_3d, rvec, translation, camera_matrix, distortion)
    return points_2d.reshape(8, 2)


def _draw_cube(image: np.ndarray, vertices: np.ndarray) -> None:
    ints = vertices.astype(int)
    top_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    bottom_edges = [(4, 5), (5, 6), (6, 7), (7, 4)]
    side_edges = [(0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in top_edges:
        cv2.line(image, tuple(ints[i]), tuple(ints[j]), _COLOR_TOP, 2)
    for i, j in bottom_edges:
        cv2.line(image, tuple(ints[i]), tuple(ints[j]), _COLOR_BOT, 2)
    for i, j in side_edges:
        cv2.line(image, tuple(ints[i]), tuple(ints[j]), _COLOR_SIDE, 2)


class AprilTagDetector:
    def __init__(
        self,
        families: str = "tag36h11",
        tag_size: float = 0.05,
        calibration_file: Optional[str] = None,
    ) -> None:
        if _AprilDetector is None:  # pragma: no cover - runtime dependency check
            raise RuntimeError(
                "pupil_apriltags is not installed. Install it with "
                "`pip install pupil-apriltags` before running sim_task."
            ) from _APRILTAG_IMPORT_ERROR

        self._tag_size = float(tag_size)
        self._detector = _AprilDetector(
            families=families,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        self._camera_matrix: Optional[np.ndarray] = None
        self._distortion: Optional[np.ndarray] = None
        self._cam_params: Optional[List[float]] = None

        if calibration_file:
            self.load_calibration(calibration_file)

    @property
    def calibrated(self) -> bool:
        return self._camera_matrix is not None and self._cam_params is not None

    def load_calibration(self, path: str) -> None:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as file_obj:
            data = yaml.safe_load(file_obj)
        camera_matrix = data["camera_matrix"]["data"]
        distortion = data["distortion_coefficients"]["data"]
        self._camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        self._distortion = np.array(distortion, dtype=np.float64)
        self._cam_params = [
            float(self._camera_matrix[0, 0]),
            float(self._camera_matrix[1, 1]),
            float(self._camera_matrix[0, 2]),
            float(self._camera_matrix[1, 2]),
        ]

    def load_camera_info_msg(self, msg: CameraInfo) -> bool:
        if len(msg.k) != 9:
            return False
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return False

        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        distortion = list(msg.d) if msg.d else [0.0] * 5
        self._distortion = np.array(distortion, dtype=np.float64)
        self._cam_params = [fx, fy, cx, cy]
        return True

    def detect(self, ros_image: RosImage) -> tuple[RosImage, list[DetectedTag]]:
        cv_image = _bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        canvas = cv_image.copy()

        detections = self._detector.detect(
            gray,
            estimate_tag_pose=self._cam_params is not None,
            camera_params=self._cam_params,
            tag_size=self._tag_size,
        )

        targets: list[DetectedTag] = []
        for detection in detections:
            tag_id = int(detection.tag_id)
            tag_family = (
                detection.tag_family.decode()
                if isinstance(detection.tag_family, bytes)
                else str(detection.tag_family)
            )
            center_u = float(detection.center[0])
            center_v = float(detection.center[1])
            corners = detection.corners
            height_px = float(np.linalg.norm(corners[0] - corners[3]))

            points = corners.astype(int)
            for index in range(4):
                cv2.line(
                    canvas,
                    tuple(points[index]),
                    tuple(points[(index + 1) % 4]),
                    (255, 80, 0),
                    2,
                )
            cv2.circle(canvas, (int(center_u), int(center_v)), 6, (0, 0, 255), -1)
            cv2.putText(
                canvas,
                f"id={tag_id} [{tag_family}]",
                (int(center_u) + 10, int(center_v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            target = DetectedTag(
                tag_id=tag_id,
                tag_family=tag_family,
                center_u=center_u,
                center_v=center_v,
                height_px=height_px,
            )

            if (
                self._cam_params is not None
                and self._camera_matrix is not None
                and self._distortion is not None
                and hasattr(detection, "pose_R")
                and detection.pose_R is not None
            ):
                rotation = np.array(detection.pose_R, dtype=np.float64)
                translation = np.array(detection.pose_t, dtype=np.float64).flatten()
                vertices = _project_cube(
                    rotation,
                    translation,
                    self._tag_size,
                    self._camera_matrix,
                    self._distortion,
                )
                _draw_cube(canvas, vertices)
                rot_x, rot_y, rot_z, rot_w = _rot_to_quat(rotation)
                target.pose_valid = True
                target.pos_x = float(translation[0])
                target.pos_y = float(translation[1])
                target.pos_z = float(translation[2])
                target.rot_x = rot_x
                target.rot_y = rot_y
                target.rot_z = rot_z
                target.rot_w = rot_w
                target.cube_vertices_u = [float(vertex[0]) for vertex in vertices]
                target.cube_vertices_v = [float(vertex[1]) for vertex in vertices]
                cv2.putText(
                    canvas,
                    f"z={target.pos_z * 100.0:.1f}cm",
                    (int(center_u) + 10, int(center_v) + 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 200),
                    1,
                )

            targets.append(target)

        annotated = _bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        annotated.header = ros_image.header
        return annotated, targets
