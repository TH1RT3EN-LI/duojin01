from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .apriltag_detector import AprilTagDetector, DetectedTag
from .sim_slot_map import (
    SLOT_NAV_X_OFFSET_M,
    SLOT_NAV_Y_OFFSET_M,
    SlotWaypoint,
    build_slot_waypoints,
    default_slot_world_path,
    load_cube_positions_from_world,
    normalize_slot_id,
    resolve_slot_waypoint,
)


ARM_HOME_CARTESIAN_MM = {"x": 195.0, "y": 0.0, "z": 269.0}
ARM_OBSERVE_X_MM = 0.0
ARM_OBSERVE_Y_MM = -190.0
ARM_OBSERVE_Z_MM = 240.0
SIM_TAG_SIZE_M = 0.03
PREGRASP_OFFSET_MM = 12.0
DESCEND_EXTRA_MM = -3.0
LIFT_MM = 80.0
PLACE_BACK_X_MM = ARM_HOME_CARTESIAN_MM["x"]
PLACE_BACK_Y_MM = ARM_HOME_CARTESIAN_MM["y"]
PLACE_BACK_Z_MM = ARM_HOME_CARTESIAN_MM["z"]
PLACE_BACK_RELEASE_Z_MM = ARM_HOME_CARTESIAN_MM["z"] - 100.0
SLOT_TASK_PLACE_Y_OFFSET_MM = 30.0
SLOT_TASK_RELEASE_Y_OFFSET_MM = -30.0
SECOND_SLOT_RELEASE_Y_OFFSET_MM = 50.0
CAPTURE_TIMEOUT_SEC = 10.0
CAMERA_INFO_TIMEOUT_SEC = 5.0
SETTLE_SEC_AFTER_MOVE = 2.0
PRE_CAPTURE_SLEEP_SEC = 5.0
PRE_RELEASE_SLEEP_SEC = 3.0
TASK_STARTUP_DELAY_SEC = 3.0
NAV_TIMEOUT_SEC = 0.0
GCODE_TIMEOUT_SEC = 10.0
TF_TIMEOUT_SEC = 1.0
POST_NAV_MOVE_X_M = 0.45
POST_NAV_MOVE_SPEED_MPS = 0.30
B_REGION_EXIT_DIRECT_SPEED_MPS = 0.30
C_REGION_EXIT_DIRECT_SPEED_MPS = 0.30
B_REGION_TRANSIT_X = 0.72
B_REGION_TRANSIT_Y = -0.26
C_REGION_TRANSIT_X = 2.33
C_REGION_TRANSIT_Y = -0.26
ENABLE_EXTRA_MANUAL_CYCLE = False
EXTRA_MANUAL_CYCLE_ONLY = False
EXTRA_MANUAL_YAW_DELTA = math.radians(210.0)
EXTRA_MANUAL_PRE_PICK_X_M = 3.14
EXTRA_MANUAL_PRE_PICK_Y_M = -0.66
EXTRA_MANUAL_PICK_X_M = 3.05
EXTRA_MANUAL_PICK_Y_M = -0.32
EXTRA_MANUAL_PICK_DESCEND_MM = 100.0
EXTRA_MANUAL_TRANSFER_NEG_Y_M = 0.30
EXTRA_MANUAL_FINAL_YAW_DELTA = math.radians(100.0)
RETURN_TO_START_YAW_OFFSET_RAD = math.radians(15.0)
FINAL_TASK_NAV_X = 0.374
FINAL_TASK_NAV_Y = -2.7
FINAL_TASK_YAW_DELTA = math.pi + math.radians(40.0)
FINAL_TASK_TURN_SPEED_RADPS = 0.5
FINAL_TASK_MOVE_X_M = -0.5
FINAL_TASK_MOVE_SPEED_MPS = 0.30
FINAL_TASK_OBSERVE_X_MM = ARM_HOME_CARTESIAN_MM["x"] - 50.0
FINAL_TASK_OBSERVE_Y_MM = ARM_HOME_CARTESIAN_MM["y"] + 30.0
FINAL_TASK_OBSERVE_Z_MM = ARM_HOME_CARTESIAN_MM["z"]
FINAL_TASK_PLACE_1_X_MM = 0.0
FINAL_TASK_PLACE_1_Y_MM = -170.0
FINAL_TASK_PLACE_1_Z_MM = 180.0
FINAL_TASK_PLACE_2_X_MM = 10.0
FINAL_TASK_PLACE_2_Y_MM = -170.0
FINAL_TASK_PLACE_2_Z_MM = 180.0

ARM_BASE_FRAME = "arm_base_link"
ARM_SUCTION_FRAME = "arm_suction_link"
BASE_MOVE_FRAME = "base_footprint"


def make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def vec_add(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(a: Tuple[float, float, float], scale: float) -> Tuple[float, float, float]:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def vec_dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_length(a: Tuple[float, float, float]) -> float:
    return math.sqrt(vec_dot(a, a))


def vec_normalize(a: Tuple[float, float, float]) -> Tuple[float, float, float]:
    length = vec_length(a)
    if length < 1e-9:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def quat_to_z_axis(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def arm_base_delta_to_gcode_delta_mm(
    delta_arm_base_m: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    return (
        delta_arm_base_m[0] * 1000.0,
        -delta_arm_base_m[2] * 1000.0,
        delta_arm_base_m[1] * 1000.0,
    )


def make_twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


class SimTask(Node):
    def __init__(self) -> None:
        super().__init__("sim_task")

        self.declare_parameter("first_slot_id", "B1")
        self.declare_parameter("second_slot_id", "C1")
        self.declare_parameter("slot_world_path", str(default_slot_world_path()))
        self.declare_parameter("slot_nav_x_offset_m", SLOT_NAV_X_OFFSET_M)
        self.declare_parameter("slot_nav_y_offset_m", SLOT_NAV_Y_OFFSET_M)
        self.declare_parameter("slot_nav_yaw", math.pi / 2.0)
        self.declare_parameter("nav_timeout_sec", NAV_TIMEOUT_SEC)
        self.declare_parameter("startup_delay_sec", TASK_STARTUP_DELAY_SEC)
        self.declare_parameter("post_nav_move_x_m", POST_NAV_MOVE_X_M)
        self.declare_parameter("post_nav_move_speed_mps", POST_NAV_MOVE_SPEED_MPS)
        self.declare_parameter("b_region_exit_direct_speed_mps", B_REGION_EXIT_DIRECT_SPEED_MPS)
        self.declare_parameter("c_region_exit_direct_speed_mps", C_REGION_EXIT_DIRECT_SPEED_MPS)
        self.declare_parameter("b_region_transit_x", B_REGION_TRANSIT_X)
        self.declare_parameter("b_region_transit_y", B_REGION_TRANSIT_Y)
        self.declare_parameter("c_region_transit_x", C_REGION_TRANSIT_X)
        self.declare_parameter("c_region_transit_y", C_REGION_TRANSIT_Y)
        self.declare_parameter("enable_extra_manual_cycle", ENABLE_EXTRA_MANUAL_CYCLE)
        self.declare_parameter("extra_manual_cycle_only", EXTRA_MANUAL_CYCLE_ONLY)
        self.declare_parameter("extra_manual_pre_pick_x_m", EXTRA_MANUAL_PRE_PICK_X_M)
        self.declare_parameter("extra_manual_pre_pick_y_m", EXTRA_MANUAL_PRE_PICK_Y_M)
        self.declare_parameter("extra_manual_pick_x_m", EXTRA_MANUAL_PICK_X_M)
        self.declare_parameter("extra_manual_pick_y_m", EXTRA_MANUAL_PICK_Y_M)
        self.declare_parameter("extra_manual_pick_descend_mm", EXTRA_MANUAL_PICK_DESCEND_MM)
        self.declare_parameter("extra_manual_transfer_neg_y_m", EXTRA_MANUAL_TRANSFER_NEG_Y_M)
        self.declare_parameter("extra_manual_final_yaw_delta", EXTRA_MANUAL_FINAL_YAW_DELTA)
        self.declare_parameter("return_to_start_yaw_offset_rad", RETURN_TO_START_YAW_OFFSET_RAD)
        self.declare_parameter("final_task_nav_x", FINAL_TASK_NAV_X)
        self.declare_parameter("final_task_nav_y", FINAL_TASK_NAV_Y)
        self.declare_parameter("final_task_yaw_delta", FINAL_TASK_YAW_DELTA)
        self.declare_parameter("final_task_turn_speed_radps", FINAL_TASK_TURN_SPEED_RADPS)
        self.declare_parameter("final_task_move_x_m", FINAL_TASK_MOVE_X_M)
        self.declare_parameter("final_task_move_speed_mps", FINAL_TASK_MOVE_SPEED_MPS)
        self.declare_parameter("final_task_observe_x_mm", FINAL_TASK_OBSERVE_X_MM)
        self.declare_parameter("final_task_observe_y_mm", FINAL_TASK_OBSERVE_Y_MM)
        self.declare_parameter("final_task_observe_z_mm", FINAL_TASK_OBSERVE_Z_MM)
        self.declare_parameter("final_task_place_1_x_mm", FINAL_TASK_PLACE_1_X_MM)
        self.declare_parameter("final_task_place_1_y_mm", FINAL_TASK_PLACE_1_Y_MM)
        self.declare_parameter("final_task_place_1_z_mm", FINAL_TASK_PLACE_1_Z_MM)
        self.declare_parameter("final_task_place_2_x_mm", FINAL_TASK_PLACE_2_X_MM)
        self.declare_parameter("final_task_place_2_y_mm", FINAL_TASK_PLACE_2_Y_MM)
        self.declare_parameter("final_task_place_2_z_mm", FINAL_TASK_PLACE_2_Z_MM)

        self.declare_parameter("tag_families", "tag36h11")
        self.declare_parameter("tag_size", SIM_TAG_SIZE_M)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("pickup_target_tag_id", -1)
        self.declare_parameter("capture_timeout_sec", CAPTURE_TIMEOUT_SEC)
        self.declare_parameter("camera_info_timeout_sec", CAMERA_INFO_TIMEOUT_SEC)
        self.declare_parameter("settle_sec_after_move", SETTLE_SEC_AFTER_MOVE)
        self.declare_parameter("pre_capture_sleep_sec", PRE_CAPTURE_SLEEP_SEC)
        self.declare_parameter("pre_release_sleep_sec", PRE_RELEASE_SLEEP_SEC)

        self.declare_parameter("arm_observe_x_mm", ARM_OBSERVE_X_MM)
        self.declare_parameter("arm_observe_y_mm", ARM_OBSERVE_Y_MM)
        self.declare_parameter("arm_observe_z_mm", ARM_OBSERVE_Z_MM)
        self.declare_parameter("pregrasp_offset_mm", PREGRASP_OFFSET_MM)
        self.declare_parameter("descend_extra_mm", DESCEND_EXTRA_MM)
        self.declare_parameter("lift_mm", LIFT_MM)
        self.declare_parameter("place_back_x_mm", PLACE_BACK_X_MM)
        self.declare_parameter("place_back_y_mm", PLACE_BACK_Y_MM)
        self.declare_parameter("place_back_z_mm", PLACE_BACK_Z_MM)
        self.declare_parameter("place_back_release_z_mm", PLACE_BACK_RELEASE_Z_MM)
        self.declare_parameter("slot_task_place_y_offset_mm", SLOT_TASK_PLACE_Y_OFFSET_MM)
        self.declare_parameter("slot_task_release_y_offset_mm", SLOT_TASK_RELEASE_Y_OFFSET_MM)
        self.declare_parameter("second_slot_release_y_offset_mm", SECOND_SLOT_RELEASE_Y_OFFSET_MM)

        self.declare_parameter("gcode_cmd_topic", "/arm/gcode_cmd")
        self.declare_parameter("gcode_result_topic", "/arm/gcode_result")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("annotated_image_topic", "/sim_task/image_detected")
        self.declare_parameter("nav_action_name", "navigate_to_pose")

        first_slot_id = normalize_slot_id(str(self.get_parameter("first_slot_id").value))
        second_slot_id = normalize_slot_id(str(self.get_parameter("second_slot_id").value))
        self.slot_sequence = [slot_id for slot_id in (first_slot_id, second_slot_id) if slot_id]
        self.slot_world_path = Path(str(self.get_parameter("slot_world_path").value)).expanduser()
        self.slot_nav_x_offset_m = float(self.get_parameter("slot_nav_x_offset_m").value)
        self.slot_nav_y_offset_m = float(self.get_parameter("slot_nav_y_offset_m").value)
        self.slot_nav_yaw = float(self.get_parameter("slot_nav_yaw").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.post_nav_move_x_m = float(self.get_parameter("post_nav_move_x_m").value)
        self.post_nav_move_speed_mps = float(self.get_parameter("post_nav_move_speed_mps").value)
        self.b_region_exit_direct_speed_mps = float(
            self.get_parameter("b_region_exit_direct_speed_mps").value
        )
        self.c_region_exit_direct_speed_mps = float(
            self.get_parameter("c_region_exit_direct_speed_mps").value
        )
        self.b_region_transit_x = float(self.get_parameter("b_region_transit_x").value)
        self.b_region_transit_y = float(self.get_parameter("b_region_transit_y").value)
        self.c_region_transit_x = float(self.get_parameter("c_region_transit_x").value)
        self.c_region_transit_y = float(self.get_parameter("c_region_transit_y").value)
        self.enable_extra_manual_cycle = bool(self.get_parameter("enable_extra_manual_cycle").value)
        self.extra_manual_cycle_only = bool(self.get_parameter("extra_manual_cycle_only").value)
        self.extra_manual_pre_pick_x_m = float(
            self.get_parameter("extra_manual_pre_pick_x_m").value
        )
        self.extra_manual_pre_pick_y_m = float(
            self.get_parameter("extra_manual_pre_pick_y_m").value
        )
        self.extra_manual_pick_x_m = float(self.get_parameter("extra_manual_pick_x_m").value)
        self.extra_manual_pick_y_m = float(self.get_parameter("extra_manual_pick_y_m").value)
        self.extra_manual_pick_descend_mm = float(
            self.get_parameter("extra_manual_pick_descend_mm").value
        )
        self.extra_manual_transfer_neg_y_m = float(
            self.get_parameter("extra_manual_transfer_neg_y_m").value
        )
        self.extra_manual_final_yaw_delta = float(
            self.get_parameter("extra_manual_final_yaw_delta").value
        )
        self.return_to_start_yaw_offset_rad = float(
            self.get_parameter("return_to_start_yaw_offset_rad").value
        )
        self.final_task_nav_x = float(self.get_parameter("final_task_nav_x").value)
        self.final_task_nav_y = float(self.get_parameter("final_task_nav_y").value)
        self.final_task_yaw_delta = float(self.get_parameter("final_task_yaw_delta").value)
        self.final_task_turn_speed_radps = float(
            self.get_parameter("final_task_turn_speed_radps").value
        )
        self.final_task_move_x_m = float(self.get_parameter("final_task_move_x_m").value)
        self.final_task_move_speed_mps = float(self.get_parameter("final_task_move_speed_mps").value)
        self.final_task_observe_pose_mm = (
            float(self.get_parameter("final_task_observe_x_mm").value),
            float(self.get_parameter("final_task_observe_y_mm").value),
            float(self.get_parameter("final_task_observe_z_mm").value),
        )

        calibration_file = str(self.get_parameter("calibration_file").value).strip()
        self.pickup_target_tag_id = int(self.get_parameter("pickup_target_tag_id").value)
        self.capture_timeout_sec = float(self.get_parameter("capture_timeout_sec").value)
        self.camera_info_timeout_sec = float(self.get_parameter("camera_info_timeout_sec").value)
        self.settle_sec_after_move = float(self.get_parameter("settle_sec_after_move").value)
        self.pre_capture_sleep_sec = float(self.get_parameter("pre_capture_sleep_sec").value)
        self.pre_release_sleep_sec = float(self.get_parameter("pre_release_sleep_sec").value)

        self._observe_pose_mm = (
            float(self.get_parameter("arm_observe_x_mm").value),
            float(self.get_parameter("arm_observe_y_mm").value),
            float(self.get_parameter("arm_observe_z_mm").value),
        )
        self.pregrasp_offset_mm = float(self.get_parameter("pregrasp_offset_mm").value)
        self.descend_extra_mm = float(self.get_parameter("descend_extra_mm").value)
        self.lift_mm = float(self.get_parameter("lift_mm").value)
        self.place_back_x_mm = float(self.get_parameter("place_back_x_mm").value)
        self.place_back_y_mm = float(self.get_parameter("place_back_y_mm").value)
        self.place_back_z_mm = float(self.get_parameter("place_back_z_mm").value)
        self.place_back_release_z_mm = float(self.get_parameter("place_back_release_z_mm").value)
        self.slot_task_place_y_offset_mm = float(
            self.get_parameter("slot_task_place_y_offset_mm").value
        )
        self.slot_task_release_y_offset_mm = float(
            self.get_parameter("slot_task_release_y_offset_mm").value
        )
        self.second_slot_release_y_offset_mm = float(
            self.get_parameter("second_slot_release_y_offset_mm").value
        )
        self.final_task_place_poses_mm = [
            (
                float(self.get_parameter("final_task_place_1_x_mm").value),
                float(self.get_parameter("final_task_place_1_y_mm").value),
                float(self.get_parameter("final_task_place_1_z_mm").value),
            ),
            (
                float(self.get_parameter("final_task_place_2_x_mm").value),
                float(self.get_parameter("final_task_place_2_y_mm").value),
                float(self.get_parameter("final_task_place_2_z_mm").value),
            ),
        ]

        self.gcode_cmd_topic = str(self.get_parameter("gcode_cmd_topic").value)
        self.gcode_result_topic = str(self.get_parameter("gcode_result_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        self.nav_action_name = str(self.get_parameter("nav_action_name").value)

        self._detector = AprilTagDetector(
            families=str(self.get_parameter("tag_families").value),
            tag_size=float(self.get_parameter("tag_size").value),
            calibration_file=calibration_file if calibration_file else None,
        )
        self._camera_model_ready_logged = self._detector.calibrated
        if self._detector.calibrated:
            self.get_logger().info("[sim_task] camera calibration loaded from file")

        self.slot_waypoints = self._load_slot_waypoints()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.pub_gcode = self.create_publisher(String, self.gcode_cmd_topic, qos)
        self.pub_detected = self.create_publisher(Image, self.annotated_image_topic, qos_latched)
        self.pub_cmd_vel = self.create_publisher(make_twist().__class__, "/cmd_vel", qos)

        self.create_subscription(Bool, self.gcode_result_topic, self._on_gcode_result, qos)
        self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, qos)

        self._gcode_event = threading.Event()
        self._gcode_ok = False
        self._nav_event = threading.Event()
        self._nav_ok = False
        self._image_event = threading.Event()
        self._camera_info_event = threading.Event()

        self._last_image: Optional[Image] = None
        self._last_camera_info: Optional[CameraInfo] = None
        self._capture_request_time: Optional[Time] = None
        self._start_pose_xy_yaw: Optional[Tuple[float, float, float]] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        threading.Thread(target=self._run_task, daemon=True).start()

    def _load_slot_waypoints(self) -> dict[str, SlotWaypoint]:
        cube_positions = load_cube_positions_from_world(self.slot_world_path)
        slot_waypoints = build_slot_waypoints(
            cube_positions,
            nav_x_offset_m=self.slot_nav_x_offset_m,
            nav_y_offset_m=self.slot_nav_y_offset_m,
        )
        self.get_logger().info(
            "[sim_task] loaded slot waypoints from "
            f"{self.slot_world_path}: {', '.join(sorted(slot_waypoints.keys()))}"
        )
        return slot_waypoints

    def _on_gcode_result(self, msg: Bool) -> None:
        self._gcode_ok = bool(msg.data)
        self._gcode_event.set()

    def _on_image(self, msg: Image) -> None:
        if self._capture_request_time is not None:
            frame_time = Time.from_msg(msg.header.stamp)
            if frame_time < self._capture_request_time:
                return
            self._capture_request_time = None
        self._last_image = msg
        self._image_event.set()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg
        self._camera_info_event.set()
        if self._detector.load_camera_info_msg(msg) and not self._camera_model_ready_logged:
            fx = float(msg.k[0])
            fy = float(msg.k[4])
            cx = float(msg.k[2])
            cy = float(msg.k[5])
            self.get_logger().info(
                "[sim_task] camera_info loaded from topic: "
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )
            self._camera_model_ready_logged = True

    def get_slot_waypoint(self, slot_id: str) -> Optional[SlotWaypoint]:
        normalized_slot_id = normalize_slot_id(slot_id)
        waypoint = resolve_slot_waypoint(self.slot_waypoints, normalized_slot_id)
        if waypoint is not None:
            return waypoint
        self.get_logger().error(
            "[sim_task] unknown slot id "
            f"{slot_id!r}; available slots: {', '.join(sorted(self.slot_waypoints.keys()))}"
        )
        return None

    def wait_for_camera_model(self) -> bool:
        if self._detector.calibrated:
            return True
        if self._last_camera_info is not None and self._detector.load_camera_info_msg(self._last_camera_info):
            return True

        self.get_logger().info("[sim_task] waiting for /camera/camera_info")
        if not self._camera_info_event.wait(self.camera_info_timeout_sec):
            self.get_logger().error("[sim_task] timed out waiting for camera_info")
            return False
        if self._last_camera_info is None:
            self.get_logger().error("[sim_task] camera_info event fired without message")
            return False
        if not self._detector.load_camera_info_msg(self._last_camera_info):
            self.get_logger().error("[sim_task] camera_info did not contain valid intrinsics")
            return False
        return True

    def navigate_to_slot(self, slot_id: str) -> bool:
        waypoint = self.get_slot_waypoint(slot_id)
        if waypoint is None:
            return False

        self.get_logger().info(
            "[sim_task] navigating to "
            f"{waypoint.slot_id}: cube={waypoint.cube_name}, "
            f"cube=({waypoint.cube_x:.3f}, {waypoint.cube_y:.3f}), "
            f"nav=({waypoint.nav_x:.3f}, {waypoint.nav_y:.3f})"
        )
        return self.nav_to(waypoint.nav_x, waypoint.nav_y, self.slot_nav_yaw)

    def get_region_transit_pose(self, slot_id: str) -> Optional[Tuple[float, float]]:
        normalized_slot_id = normalize_slot_id(slot_id)
        if normalized_slot_id.startswith("B"):
            return (self.b_region_transit_x, self.b_region_transit_y)
        if normalized_slot_id.startswith("C"):
            return (self.c_region_transit_x, self.c_region_transit_y)
        self.get_logger().error(f"[sim_task] unknown region for slot id {slot_id!r}")
        return None

    def navigate_to_region_transit(self, slot_id: str, phase: str) -> bool:
        transit_pose = self.get_region_transit_pose(slot_id)
        if transit_pose is None:
            return False
        self.get_logger().info(
            "[sim_task] region transit: "
            f"phase={phase}, slot={slot_id}, transit=({transit_pose[0]:.3f}, {transit_pose[1]:.3f})"
        )
        normalized_slot_id = normalize_slot_id(slot_id)
        if normalized_slot_id.startswith("B") and phase == "exit":
            return self.drive_direct_to_map_point(
                target_x=transit_pose[0],
                target_y=transit_pose[1],
                speed_mps=abs(self.b_region_exit_direct_speed_mps),
                label="b-region exit direct move",
            )
        if normalized_slot_id.startswith("C") and phase == "exit":
            return self.drive_direct_to_map_point(
                target_x=transit_pose[0],
                target_y=transit_pose[1],
                speed_mps=abs(self.c_region_exit_direct_speed_mps),
                label="c-region exit direct move",
            )
        return self.nav_to(transit_pose[0], transit_pose[1], self.slot_nav_yaw)

    def nav_to(self, x: float, y: float, yaw: float) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("[sim_task] Nav2 action server is not available")
            return False

        self._nav_event.clear()
        self._nav_ok = False
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"[sim_task] nav goal -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

        if self.nav_timeout_sec <= 0.0:
            self._nav_event.wait()
        elif not self._nav_event.wait(self.nav_timeout_sec):
            self.get_logger().error("[sim_task] navigation timed out")
            return False
        return self._nav_ok

    def stop_base(self) -> None:
        stop = make_twist()
        for _ in range(5):
            self.pub_cmd_vel.publish(stop)
            time.sleep(0.05)

    def move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> None:
        twist = make_twist(vx=vx, vy=vy, wz=wz)
        self.get_logger().info(
            "[sim_task] base move: "
            f"vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}, duration={duration:.2f}s"
        )
        duration = max(0.0, float(duration))
        if duration <= 0.0:
            self.stop_base()
            return

        deadline = time.monotonic() + duration
        publish_period = 1.0 / 20.0
        while rclpy.ok():
            now = time.monotonic()
            if now >= deadline:
                break
            self.pub_cmd_vel.publish(twist)
            time.sleep(min(publish_period, max(0.0, deadline - now)))
        self.stop_base()

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_in_map_x(self, distance: float, speed: float, label: str) -> None:
        if speed < 1e-6 or abs(distance) < 1e-6:
            self.get_logger().info(f"[sim_task] {label} skipped")
            return

        transform = self.lookup_transform("map", BASE_MOVE_FRAME)
        if transform is None:
            self.get_logger().error(f"[sim_task] {label} failed: missing map -> base_footprint TF")
            return

        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )

        world_vx = speed if distance >= 0.0 else -speed
        world_vy = 0.0
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        duration = abs(distance) / speed

        self.get_logger().info(
            f"[sim_task] {label} in map frame: "
            f"map_dx={distance:.3f} m, speed={speed:.3f} m/s, yaw={yaw:.3f} rad, "
            f"cmd_vel=(vx={body_vx:.3f}, vy={body_vy:.3f})"
        )
        self.move_for_time(vx=body_vx, vy=body_vy, wz=0.0, duration=duration)

    def drive_direct_to_map_point(
        self,
        target_x: float,
        target_y: float,
        speed_mps: float,
        label: str,
    ) -> bool:
        if speed_mps < 1e-6:
            self.get_logger().error(f"[sim_task] {label} failed: speed is too small")
            return False

        pose = self.get_base_pose_in_map()
        if pose is None:
            self.get_logger().error(f"[sim_task] {label} failed: missing current base pose in map")
            return False

        current_x, current_y, yaw = pose
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            self.get_logger().info(f"[sim_task] {label} skipped: already near target")
            return True

        world_vx = speed_mps * dx / distance
        world_vy = speed_mps * dy / distance
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        duration = distance / speed_mps

        self.get_logger().info(
            f"[sim_task] {label}: "
            f"from=({current_x:.3f}, {current_y:.3f}) to=({target_x:.3f}, {target_y:.3f}), "
            f"distance={distance:.3f} m, speed={speed_mps:.3f} m/s, "
            f"cmd_vel=(vx={body_vx:.3f}, vy={body_vy:.3f})"
        )
        self.move_for_time(vx=body_vx, vy=body_vy, wz=0.0, duration=duration)
        return True

    def move_in_map_x_after_nav(self) -> None:
        self.move_in_map_x(
            distance=self.post_nav_move_x_m,
            speed=abs(self.post_nav_move_speed_mps),
            label="post-nav move",
        )

    def move_out_before_next_nav(self) -> None:
        self.move_in_map_x(
            distance=-self.post_nav_move_x_m,
            speed=abs(self.post_nav_move_speed_mps),
            label="pre-next-nav retreat",
        )

    def normalize_yaw(self, yaw: float) -> float:
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def get_base_yaw_in_map(self) -> Optional[float]:
        transform = self.lookup_transform("map", BASE_MOVE_FRAME)
        if transform is None:
            return None

        rotation = transform.transform.rotation
        return self._yaw_from_quaternion(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )

    def get_base_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        transform = self.lookup_transform("map", BASE_MOVE_FRAME)
        if transform is None:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        return (float(translation.x), float(translation.y), yaw)

    def rotate_in_place(self, angle_rad: float, angular_speed_radps: float, label: str) -> None:
        if abs(angle_rad) < 1e-6 or abs(angular_speed_radps) < 1e-6:
            self.get_logger().info(f"[sim_task] {label} skipped")
            return

        direction = 1.0 if angle_rad >= 0.0 else -1.0
        wz = direction * abs(angular_speed_radps)
        duration = abs(angle_rad) / abs(angular_speed_radps)
        self.get_logger().info(
            f"[sim_task] {label}: angle={angle_rad:.3f} rad, "
            f"wz={wz:.3f} rad/s, duration={duration:.2f}s"
        )
        self.move_for_time(vx=0.0, vy=0.0, wz=wz, duration=duration)

    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[sim_task] nav goal was rejected")
            self._nav_ok = False
            self._nav_event.set()
            return
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self._nav_ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"[sim_task] navigation success={self._nav_ok}")
        self._nav_event.set()

    def gcode(self, cmd: str, timeout: float = GCODE_TIMEOUT_SEC) -> bool:
        if self.count_subscribers(self.gcode_cmd_topic) == 0:
            self.get_logger().error(
                "[sim_task] nobody subscribes to "
                f"{self.gcode_cmd_topic}; make sure e4_gcode_adapter is running"
            )
            return False
        if self.count_publishers(self.gcode_result_topic) == 0:
            self.get_logger().error(
                "[sim_task] nobody publishes "
                f"{self.gcode_result_topic}; make sure e4_gcode_adapter is running"
            )
            return False

        self._gcode_event.clear()
        msg = String()
        msg.data = cmd
        self.pub_gcode.publish(msg)
        self.get_logger().info(f"[sim_task] G-code -> {cmd}")
        if not self._gcode_event.wait(timeout):
            self.get_logger().error(f"[sim_task] G-code timed out: {cmd}")
            return False
        if not self._gcode_ok:
            self.get_logger().error(f"[sim_task] G-code failed: {cmd}")
            return False
        return True

    def arm_home(self) -> bool:
        if not self.gcode("$h"):
            return False
        time.sleep(self.settle_sec_after_move)
        return True

    def arm_move_absolute(self, x_mm: float, y_mm: float, z_mm: float) -> bool:
        return self.gcode(f"M20 G90 X{x_mm:.1f} Y{y_mm:.1f} Z{z_mm:.1f}")

    def arm_move_relative(self, dx_mm: float = 0.0, dy_mm: float = 0.0, dz_mm: float = 0.0) -> bool:
        return self.gcode(f"M20 G91 X{dx_mm:.1f} Y{dy_mm:.1f} Z{dz_mm:.1f}")

    def set_suction(self, enabled: bool) -> bool:
        return self.gcode("M3 S1000" if enabled else "M3 S500")

    def capture_and_detect(self) -> tuple[Optional[Image], list[DetectedTag]]:
        if not self.wait_for_camera_model():
            return None, []

        self._image_event.clear()
        self._last_image = None
        self._capture_request_time = self.get_clock().now()
        self.get_logger().info("[sim_task] waiting for a fresh camera frame")

        if not self._image_event.wait(self.capture_timeout_sec):
            self.get_logger().error("[sim_task] timed out waiting for camera frame")
            return None, []

        raw_image = self._last_image
        if raw_image is None:
            self.get_logger().error("[sim_task] image event fired without image payload")
            return None, []

        annotated, targets = self._detector.detect(raw_image)
        self.pub_detected.publish(annotated)
        if targets:
            for target in targets:
                pose_text = (
                    f", pose=({target.pos_x:.4f}, {target.pos_y:.4f}, {target.pos_z:.4f}) m"
                    if target.pose_valid
                    else ", pose=invalid"
                )
                self.get_logger().info(
                    f"[sim_task] detected tag id={target.tag_id}, "
                    f"center=({target.center_u:.1f}, {target.center_v:.1f}){pose_text}"
                )
        else:
            self.get_logger().warning("[sim_task] no tags detected in current frame")
        return raw_image, targets

    def select_target(self, raw_image: Optional[Image], targets: list[DetectedTag]) -> Optional[DetectedTag]:
        if raw_image is None or not targets:
            return None

        candidates = [target for target in targets if target.pose_valid]
        if self.pickup_target_tag_id >= 0:
            candidates = [target for target in candidates if target.tag_id == self.pickup_target_tag_id]
        if not candidates:
            return None

        image_cx = raw_image.width / 2.0
        image_cy = raw_image.height / 2.0

        def score(target: DetectedTag) -> float:
            du = target.center_u - image_cx
            dv = target.center_v - image_cy
            return du * du + dv * dv

        best = min(candidates, key=score)
        pixel_error = math.sqrt(score(best))
        self.get_logger().info(
            "[sim_task] selected target "
            f"id={best.tag_id}, pixel_error={pixel_error:.1f}"
        )
        return best

    def lookup_transform(self, target_frame: str, source_frame: str, timeout_sec: float = TF_TIMEOUT_SEC):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().error(
                f"[sim_task] TF lookup failed: {target_frame} <- {source_frame}: {exc}"
            )
            return None

    def get_current_suction_state_in_arm_base(
        self,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
        transform = self.lookup_transform(ARM_BASE_FRAME, ARM_SUCTION_FRAME)
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        point = (float(translation.x), float(translation.y), float(translation.z))
        z_axis = vec_normalize(
            quat_to_z_axis(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
        )
        return point, z_axis

    def transform_target_point_to_arm_base(
        self,
        target: DetectedTag,
        source_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        transform = self.lookup_transform(ARM_BASE_FRAME, source_frame)
        if transform is None:
            return None

        point_msg = PointStamped()
        point_msg.header.frame_id = source_frame
        point_msg.point.x = float(target.pos_x)
        point_msg.point.y = float(target.pos_y)
        point_msg.point.z = float(target.pos_z)
        transformed = do_transform_point(point_msg, transform)
        return (
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        )

    def compute_approach_axis(
        self,
        target_point_arm_base: Tuple[float, float, float],
        suction_point_arm_base: Tuple[float, float, float],
        suction_z_axis_arm_base: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        vector_to_target = vec_sub(target_point_arm_base, suction_point_arm_base)
        if vec_length(vector_to_target) < 1e-6:
            self.get_logger().error("[sim_task] suction point already overlaps target point")
            return None

        sign = 1.0 if vec_dot(vector_to_target, suction_z_axis_arm_base) >= 0.0 else -1.0
        return vec_normalize(vec_scale(suction_z_axis_arm_base, sign))

    def move_suction_to_arm_base_point(
        self,
        desired_suction_point_arm_base: Tuple[float, float, float],
        label: str,
    ) -> bool:
        suction_state = self.get_current_suction_state_in_arm_base()
        if suction_state is None:
            return False
        current_suction_point_arm_base, _ = suction_state
        delta_arm_base_m = vec_sub(desired_suction_point_arm_base, current_suction_point_arm_base)
        delta_cmd_mm = arm_base_delta_to_gcode_delta_mm(delta_arm_base_m)

        self.get_logger().info(
            "[sim_task] move suction "
            f"{label}: delta_arm_base=({delta_arm_base_m[0] * 1000.0:.1f}, "
            f"{delta_arm_base_m[1] * 1000.0:.1f}, {delta_arm_base_m[2] * 1000.0:.1f}) mm, "
            f"delta_cmd=({delta_cmd_mm[0]:.1f}, {delta_cmd_mm[1]:.1f}, {delta_cmd_mm[2]:.1f}) mm"
        )
        if not self.arm_move_relative(*delta_cmd_mm):
            return False
        time.sleep(self.settle_sec_after_move)
        return True

    def move_to_observe_pose(self) -> bool:
        if not self.arm_move_absolute(*self._observe_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)
        self.get_logger().info(
            "[sim_task] arm moved to observe pose "
            f"({self._observe_pose_mm[0]:.1f}, {self._observe_pose_mm[1]:.1f}, {self._observe_pose_mm[2]:.1f}) mm"
        )
        return True

    def move_to_final_observe_pose(self) -> bool:
        if not self.arm_move_absolute(*self.final_task_observe_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)
        self.get_logger().info(
            "[sim_task] arm moved to final observe pose "
            f"({self.final_task_observe_pose_mm[0]:.1f}, "
            f"{self.final_task_observe_pose_mm[1]:.1f}, "
            f"{self.final_task_observe_pose_mm[2]:.1f}) mm"
        )
        return True

    def best_effort_release(self) -> None:
        try:
            self.set_suction(False)
            time.sleep(0.5)
        except Exception:
            pass

    def execute_single_grasp_cycle(
        self,
        slot_id: str,
        *,
        is_second_slot: bool = False,
        place_pre_pose_mm: Optional[Tuple[float, float, float]] = None,
        place_release_pose_mm: Optional[Tuple[float, float, float]] = None,
        place_return_pose_mm: Optional[Tuple[float, float, float]] = None,
        skip_pre_place_pose: bool = False,
    ) -> Optional[bool]:
        if self.pre_capture_sleep_sec > 0.0:
            self.get_logger().info(
                "[sim_task] waiting before capture: "
                f"{self.pre_capture_sleep_sec:.1f}s"
            )
            time.sleep(self.pre_capture_sleep_sec)

        raw_image, targets = self.capture_and_detect()
        if raw_image is None:
            self.get_logger().warning(
                f"[sim_task] skip slot {slot_id}: capture failed or no fresh image available"
            )
            return None

        target = self.select_target(raw_image, targets)
        if target is None:
            self.get_logger().warning(f"[sim_task] skip slot {slot_id}: no valid grasp target found")
            return None

        source_frame = raw_image.header.frame_id or "camera_optical_frame"
        target_point_arm_base = self.transform_target_point_to_arm_base(target, source_frame)
        if target_point_arm_base is None:
            return False

        suction_state = self.get_current_suction_state_in_arm_base()
        if suction_state is None:
            return False
        suction_point_arm_base, suction_z_axis_arm_base = suction_state

        approach_axis = self.compute_approach_axis(
            target_point_arm_base,
            suction_point_arm_base,
            suction_z_axis_arm_base,
        )
        if approach_axis is None:
            return False

        pregrasp_point = vec_sub(target_point_arm_base, vec_scale(approach_axis, self.pregrasp_offset_mm / 1000.0))
        grasp_point = vec_add(target_point_arm_base, vec_scale(approach_axis, self.descend_extra_mm / 1000.0))

        self.get_logger().info(
            "[sim_task] grasp geometry "
            f"slot={slot_id}, tag_id={target.tag_id}, "
            f"target=({target_point_arm_base[0]:.4f}, {target_point_arm_base[1]:.4f}, {target_point_arm_base[2]:.4f}) m, "
            f"approach=({approach_axis[0]:.4f}, {approach_axis[1]:.4f}, {approach_axis[2]:.4f})"
        )

        if not self.move_suction_to_arm_base_point(pregrasp_point, "pregrasp"):
            return False

        suction_state = self.get_current_suction_state_in_arm_base()
        if suction_state is None:
            return False
        suction_point_arm_base, suction_z_axis_arm_base = suction_state
        approach_axis = self.compute_approach_axis(
            target_point_arm_base,
            suction_point_arm_base,
            suction_z_axis_arm_base,
        )
        if approach_axis is None:
            return False
        grasp_point = vec_add(target_point_arm_base, vec_scale(approach_axis, self.descend_extra_mm / 1000.0))
        lift_point = vec_sub(grasp_point, vec_scale(approach_axis, self.lift_mm / 1000.0))

        if not self.move_suction_to_arm_base_point(grasp_point, "grasp"):
            return False
        if not self.set_suction(True):
            return False
        time.sleep(1.0)

        if not self.move_suction_to_arm_base_point(lift_point, "lift"):
            return False

        if place_pre_pose_mm is None:
            place_pre_pose_mm = (
                self.place_back_x_mm,
                self.place_back_y_mm,
                self.place_back_z_mm,
            )
        if place_release_pose_mm is None:
            release_y_mm = self.place_back_y_mm
            if is_second_slot:
                release_y_mm += self.second_slot_release_y_offset_mm
                self.get_logger().info(
                    "[sim_task] second slot release offset applied: "
                    f"y += {self.second_slot_release_y_offset_mm:.1f} mm -> {release_y_mm:.1f} mm"
                )
            place_release_pose_mm = (
                self.place_back_x_mm,
                release_y_mm,
                self.place_back_release_z_mm,
            )
        if place_return_pose_mm is None:
            place_return_pose_mm = place_pre_pose_mm

        if not skip_pre_place_pose:
            if not self.arm_move_absolute(*place_pre_pose_mm):
                return False
            time.sleep(self.settle_sec_after_move)
        else:
            self.get_logger().info("[sim_task] skipping pre-place pose before release")

        if not self.arm_move_absolute(*place_release_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)

        if self.pre_release_sleep_sec > 0.0:
            self.get_logger().info(
                "[sim_task] waiting before suction off: "
                f"{self.pre_release_sleep_sec:.1f}s"
            )
            time.sleep(self.pre_release_sleep_sec)

        if not self.set_suction(False):
            return False
        time.sleep(1.0)

        if not self.arm_move_absolute(*place_return_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)

        self.get_logger().info(f"[sim_task] grasp cycle completed for slot {slot_id}")
        return True

    def run_slot_task(self, slot_id: str, *, is_second_slot: bool = False) -> bool:
        if not self.navigate_to_region_transit(slot_id, "enter"):
            return False
        if not self.navigate_to_slot(slot_id):
            return False
        time.sleep(1.0)
        self.move_in_map_x_after_nav()
        time.sleep(0.5)

        if not self.arm_home():
            return False
        if not self.move_to_observe_pose():
            return False
        slot_place_y_mm = self.place_back_y_mm + self.slot_task_place_y_offset_mm
        slot_release_y_mm = slot_place_y_mm + self.slot_task_release_y_offset_mm
        if is_second_slot:
            slot_release_y_mm += self.second_slot_release_y_offset_mm

        grasp_result = self.execute_single_grasp_cycle(
            slot_id,
            is_second_slot=False,
            place_pre_pose_mm=(
                self.place_back_x_mm,
                slot_place_y_mm,
                self.place_back_z_mm,
            ),
            place_release_pose_mm=(
                self.place_back_x_mm,
                slot_release_y_mm,
                self.place_back_release_z_mm,
            ),
            place_return_pose_mm=(
                self.place_back_x_mm,
                slot_place_y_mm,
                self.place_back_z_mm,
            ),
        )
        if grasp_result is False:
            self.best_effort_release()
            return False
        if grasp_result is None:
            self.get_logger().warning(f"[sim_task] slot {slot_id} skipped; leaving area and continuing")
        if not self.move_to_observe_pose():
            return False
        if not self.arm_home():
            return False
        self.move_out_before_next_nav()
        time.sleep(0.5)
        if not self.navigate_to_region_transit(slot_id, "exit"):
            return False
        return True

    def run_final_task(self) -> bool:
        c_transit_pose = self.get_region_transit_pose("C1")
        if c_transit_pose is None:
            return False
        if not self.navigate_to_region_transit("C1", "exit"):
            return False
        time.sleep(0.5)

        b_transit_pose = self.get_region_transit_pose("B1")
        if b_transit_pose is None:
            return False
        self.get_logger().info(
            "[sim_task] final task via B transit: "
            f"transit=({b_transit_pose[0]:.3f}, {b_transit_pose[1]:.3f})"
        )
        if not self.nav_to(b_transit_pose[0], b_transit_pose[1], self.slot_nav_yaw):
            return False
        time.sleep(0.5)

        nav_yaw = self.get_base_yaw_in_map()
        if nav_yaw is None:
            nav_yaw = self.slot_nav_yaw
            self.get_logger().warning(
                "[sim_task] final task nav yaw fallback to slot_nav_yaw because current base yaw is unavailable"
            )
        self.get_logger().info(
            "[sim_task] final task nav: "
            f"goal=({self.final_task_nav_x:.3f}, {self.final_task_nav_y:.3f}), "
            f"yaw={nav_yaw:.3f}"
        )
        if not self.nav_to(self.final_task_nav_x, self.final_task_nav_y, nav_yaw):
            return False
        time.sleep(1.0)
        self.rotate_in_place(
            angle_rad=self.final_task_yaw_delta,
            angular_speed_radps=self.final_task_turn_speed_radps,
            label="final task manual turn",
        )
        time.sleep(0.5)

        self.move_in_map_x(
            distance=self.final_task_move_x_m,
            speed=abs(self.final_task_move_speed_mps),
            label="final task map-x move",
        )
        time.sleep(0.5)

        for index, release_pose_mm in enumerate(self.final_task_place_poses_mm, start=1):
            self.get_logger().info(
                "[sim_task] final arm cycle "
                f"{index}/{len(self.final_task_place_poses_mm)}: "
                f"release=({release_pose_mm[0]:.1f}, {release_pose_mm[1]:.1f}, {release_pose_mm[2]:.1f}) mm"
            )
            if not self.arm_home():
                return False
            if not self.move_to_final_observe_pose():
                return False

            pre_place_pose_mm = (
                release_pose_mm[0],
                release_pose_mm[1],
                self.place_back_z_mm,
            )
            grasp_result = self.execute_single_grasp_cycle(
                f"final_{index}",
                place_pre_pose_mm=pre_place_pose_mm,
                place_release_pose_mm=release_pose_mm,
                place_return_pose_mm=self.final_task_observe_pose_mm,
                skip_pre_place_pose=True,
            )
            if grasp_result is False:
                self.best_effort_release()
                return False
            if grasp_result is None:
                self.get_logger().warning(
                    f"[sim_task] final arm cycle {index} skipped; continuing to next cycle"
                )
                continue

        if not self.arm_home():
            return False
        return True

    def return_to_start_pose(self) -> bool:
        if self._start_pose_xy_yaw is None:
            self.get_logger().warning("[sim_task] start pose is unavailable; skip returning to start")
            return True

        self.move_in_map_x(
            distance=self.final_task_move_x_m,
            speed=abs(self.final_task_move_speed_mps),
            label="post-task depart move",
        )
        time.sleep(0.5)

        start_x, start_y, start_yaw = self._start_pose_xy_yaw
        self.get_logger().info(
            "[sim_task] returning to start pose: "
            f"x={start_x:.3f}, y={start_y:.3f}, yaw={start_yaw:.3f}"
        )
        if not self.drive_direct_to_map_point(
            target_x=start_x,
            target_y=start_y,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="manual return to start position",
        ):
            return False
        time.sleep(0.5)

        current_yaw = self.get_base_yaw_in_map()
        if current_yaw is None:
            self.get_logger().warning(
                "[sim_task] current yaw unavailable after manual return; skip final yaw alignment"
            )
            return True

        target_yaw = self.normalize_yaw(start_yaw + self.return_to_start_yaw_offset_rad)
        yaw_error = self.normalize_yaw(target_yaw - current_yaw)
        self.rotate_in_place(
            angle_rad=yaw_error,
            angular_speed_radps=self.final_task_turn_speed_radps,
            label="manual return yaw align",
        )
        time.sleep(0.5)
        return True

    def run_extra_manual_cycle(self) -> bool:
        if self._start_pose_xy_yaw is None:
            self.get_logger().error("[sim_task] extra manual cycle failed: start pose is unavailable")
            return False

        start_x, start_y, start_yaw = self._start_pose_xy_yaw
        target_yaw = self.normalize_yaw(start_yaw + EXTRA_MANUAL_YAW_DELTA)

        current_yaw = self.get_base_yaw_in_map()
        if current_yaw is None:
            self.get_logger().error("[sim_task] extra manual cycle failed: current base yaw is unavailable")
            return False

        turn_delta = (target_yaw - current_yaw) % (2.0 * math.pi)
        self.rotate_in_place(
            angle_rad=turn_delta,
            angular_speed_radps=self.final_task_turn_speed_radps,
            label="extra cycle yaw align",
        )
        time.sleep(0.5)

        b_transit_pose = self.get_region_transit_pose("B1")
        if b_transit_pose is None:
            return False
        if not self.drive_direct_to_map_point(
            target_x=b_transit_pose[0],
            target_y=b_transit_pose[1],
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle to b transit",
        ):
            return False
        time.sleep(0.5)

        if not self.drive_direct_to_map_point(
            target_x=self.extra_manual_pre_pick_x_m,
            target_y=self.extra_manual_pre_pick_y_m,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle to manual pre-pick point",
        ):
            return False
        time.sleep(0.5)

        if not self.arm_home():
            return False
        if not self.move_to_observe_pose():
            return False

        if not self.drive_direct_to_map_point(
            target_x=self.extra_manual_pick_x_m,
            target_y=self.extra_manual_pick_y_m,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle to manual pick point",
        ):
            return False
        time.sleep(0.5)
        self.get_logger().info("[sim_task] extra cycle waiting 5.0s before manual pick descend")
        time.sleep(5.0)

        pick_pose_mm = (
            self._observe_pose_mm[0],
            self._observe_pose_mm[1],
            self._observe_pose_mm[2] - self.extra_manual_pick_descend_mm,
        )
        if not self.arm_move_absolute(*pick_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)
        if not self.set_suction(True):
            return False
        time.sleep(1.0)
        if not self.move_to_observe_pose():
            return False

        if not self.drive_direct_to_map_point(
            target_x=self.extra_manual_pre_pick_x_m,
            target_y=self.extra_manual_pre_pick_y_m,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle back to manual pre-pick point",
        ):
            return False
        time.sleep(0.5)

        current_pose = self.get_base_pose_in_map()
        if current_pose is None:
            self.get_logger().error("[sim_task] extra manual cycle failed: current base pose is unavailable")
            return False
        current_x, current_y, _ = current_pose
        if not self.drive_direct_to_map_point(
            target_x=current_x,
            target_y=current_y - self.extra_manual_transfer_neg_y_m,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle move map -y",
        ):
            return False
        time.sleep(0.5)

        if not self.drive_direct_to_map_point(
            target_x=b_transit_pose[0],
            target_y=b_transit_pose[1],
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle back to b transit",
        ):
            return False
        time.sleep(0.5)

        if not self.drive_direct_to_map_point(
            target_x=self.final_task_nav_x,
            target_y=self.final_task_nav_y,
            speed_mps=abs(self.post_nav_move_speed_mps),
            label="extra cycle back to final point",
        ):
            return False
        time.sleep(0.5)

        self.rotate_in_place(
            angle_rad=self.extra_manual_final_yaw_delta,
            angular_speed_radps=self.final_task_turn_speed_radps,
            label="extra cycle final turn",
        )
        time.sleep(0.5)

        self.move_in_map_x(
            distance=self.final_task_move_x_m,
            speed=abs(self.final_task_move_speed_mps),
            label="extra cycle final approach",
        )
        time.sleep(0.5)

        if self.pre_release_sleep_sec > 0.0:
            self.get_logger().info(
                "[sim_task] extra cycle waiting before suction off: "
                f"{self.pre_release_sleep_sec:.1f}s"
            )
            time.sleep(self.pre_release_sleep_sec)
        if not self.set_suction(False):
            return False
        time.sleep(1.0)
        if not self.arm_home():
            return False
        return True

    def run_extra_manual_cycle_only_flow(self) -> bool:
        if not self.run_extra_manual_cycle():
            return False
        if not self.return_to_start_pose():
            return False
        return True

    def _run_task(self) -> None:
        time.sleep(max(0.0, self.startup_delay_sec))
        self.get_logger().info(f"===== sim_task started, slots={self.slot_sequence} =====")

        self._start_pose_xy_yaw = self.get_base_pose_in_map()
        if self._start_pose_xy_yaw is None:
            self.get_logger().warning("[sim_task] failed to capture start pose; final return will be skipped")
        else:
            self.get_logger().info(
                "[sim_task] captured start pose: "
                f"x={self._start_pose_xy_yaw[0]:.3f}, "
                f"y={self._start_pose_xy_yaw[1]:.3f}, "
                f"yaw={self._start_pose_xy_yaw[2]:.3f}"
            )

        if self.extra_manual_cycle_only:
            self.get_logger().info(
                "[sim_task] extra_manual_cycle_only=true, skip standard slot/final flow"
            )
            if not self.run_extra_manual_cycle_only_flow():
                self.get_logger().error("[sim_task] extra manual cycle-only task failed")
                self.best_effort_release()
                rclpy.shutdown()
                return

            self.best_effort_release()
            self.arm_home()
            self.get_logger().info("[sim_task] extra manual cycle-only task completed")
            rclpy.shutdown()
            return

        if not self.slot_sequence:
            self.get_logger().error("[sim_task] no slot ids were provided")
            rclpy.shutdown()
            return

        total_slots = len(self.slot_sequence)
        for index, slot_id in enumerate(self.slot_sequence, start=1):
            self.get_logger().info(f"===== slot {index}/{len(self.slot_sequence)}: {slot_id} =====")
            if not self.run_slot_task(slot_id, is_second_slot=(index == 2)):
                self.get_logger().error(f"[sim_task] task failed at slot {slot_id}")
                self.best_effort_release()
                rclpy.shutdown()
                return

        if not self.run_final_task():
            self.get_logger().error("[sim_task] final task failed")
            self.best_effort_release()
            rclpy.shutdown()
            return

        if self.enable_extra_manual_cycle:
            self.get_logger().info(
                "[sim_task] enable_extra_manual_cycle=true, return to start before extra cycle"
            )
            if not self.return_to_start_pose():
                self.get_logger().error(
                    "[sim_task] failed to return to start pose before extra manual cycle"
                )
                self.best_effort_release()
                rclpy.shutdown()
                return
            if not self.run_extra_manual_cycle_only_flow():
                self.get_logger().error("[sim_task] extra manual cycle failed after return to start")
                self.best_effort_release()
                rclpy.shutdown()
                return
        elif not self.return_to_start_pose():
            self.get_logger().error("[sim_task] failed to return to start pose")
            self.best_effort_release()
            rclpy.shutdown()
            return

        self.best_effort_release()
        self.arm_home()
        self.get_logger().info("[sim_task] all requested slots completed")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
