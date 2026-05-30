#!/usr/bin/env python3
"""
仿真抓取调试脚本。

支持三种模式：

1. full
   执行完整流程：导航 -> 横移 -> 回零 -> 观察位 -> 单次识别抓取 -> 放回 -> 回观察位
2. prepare_only
   仅执行前四步，把机器人送到观察位后保持节点存活，便于后续单独启动 grasp_debug_loop
3. grasp_debug_loop
   假定底盘已在正确位置，只重新把机械臂送到观察位，然后循环执行识别抓取和放回

当前脚本只面向 race_track.sdf 里的单场景 tagged cube 调试，不做多场景泛化。
"""

import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from duojin01_mission.april_tag_detector import AprilTagDetector
from duojin01_mission.sim_slot_map import (
    SLOT_NAV_X_OFFSET_M,
    SlotWaypoint,
    build_slot_waypoints,
    default_slot_world_path,
    load_cube_positions_from_world,
    normalize_slot_id,
    resolve_slot_waypoint,
)
from duojin01_msgs.msg import TargetInfo


NAV_EXAMPLE_POSE = {"x": 0.9, "y": -1.25, "yaw": math.pi / 2.0}
HOME_POSE = {"x": 0.0, "y": 0.0, "yaw": 0.0}
ARM_HOME_CARTESIAN_MM = {"x": 190.4, "y": 0.0, "z": 305.2}

BASE_TIME_MOVE_DURATION = 1.0
BASE_TIME_MOVE_VX = 0.12
BASE_DISTANCE_MOVE_AXIS = "y"
BASE_DISTANCE_MOVE_METERS = -1.35
BASE_DISTANCE_MOVE_SPEED = 0.10

ARM_EXAMPLE_X_MM = 50.0
ARM_EXAMPLE_Y_MM = -190.0
ARM_EXAMPLE_Z_MM = 240.0

CAPTURE_TIMEOUT_SEC = 10.0
CAMERA_INFO_TIMEOUT_SEC = 5.0
SETTLE_SEC_AFTER_MOVE = 2.0
SIM_TAG_SIZE_M = 0.03
PREGRASP_OFFSET_MM = 12.0
DESCEND_EXTRA_MM = -3.0
LIFT_MM = 80.0
PLACE_BACK_X_MM = 120.0
PLACE_BACK_Y_MM = -120.0
PLACE_BACK_Z_MM = 240.0
PLACE_BACK_RELEASE_Z_MM = 195.0
DEBUG_REPEAT_COUNT = 5
PICKUP_TARGET_TAG_ID = -1

BASE_CMD_STREAM_HZ = 20.0
GCODE_TIMEOUT_SEC = 10.0
TF_TIMEOUT_SEC = 1.0

ARM_BASE_FRAME = "arm_base_link"
ARM_SUCTION_FRAME = "arm_suction_link"


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
    """
    将 arm_base_link 下的位移差值换算成 E4 G-code 的 G91 相对位移。

    当前仿真机械臂的 `arm_move_absolute/relative()` 命令坐标系与 `arm_base_link`
    存在固定轴向置换：
      - cmd +X 约等于 arm_base +X
      - cmd +Y 约等于 arm_base -Z
      - cmd +Z 约等于 arm_base +Y

    本轮只针对当前 sim_demo 单场景抓取调试，直接使用这组固定映射，
    不做更通用的多场景标定。
    """
    return (
        delta_arm_base_m[0] * 1000.0,
        -delta_arm_base_m[2] * 1000.0,
        delta_arm_base_m[1] * 1000.0,
    )


class SimMissionDemo(Node):
    def __init__(self):
        super().__init__("sim_demo")

        self.declare_parameter("mode", "full")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("tag_families", "tag36h11")
        self.declare_parameter("tag_size", SIM_TAG_SIZE_M)

        self.declare_parameter("nav_goal_x", NAV_EXAMPLE_POSE["x"])
        self.declare_parameter("nav_goal_y", NAV_EXAMPLE_POSE["y"])
        self.declare_parameter("nav_goal_yaw", NAV_EXAMPLE_POSE["yaw"])
        self.declare_parameter("nav_timeout_sec", 0.0)
        self.declare_parameter("target_slot_id", "")
        self.declare_parameter("slot_world_path", str(default_slot_world_path()))
        self.declare_parameter("slot_nav_x_offset_m", SLOT_NAV_X_OFFSET_M)
        self.declare_parameter("slot_nav_yaw", NAV_EXAMPLE_POSE["yaw"])
        self.declare_parameter("home_x", HOME_POSE["x"])
        self.declare_parameter("home_y", HOME_POSE["y"])
        self.declare_parameter("home_yaw", HOME_POSE["yaw"])

        self.declare_parameter("base_time_move_duration", BASE_TIME_MOVE_DURATION)
        self.declare_parameter("base_time_move_vx", BASE_TIME_MOVE_VX)
        self.declare_parameter("base_distance_move_axis", BASE_DISTANCE_MOVE_AXIS)
        self.declare_parameter("base_distance_move_meters", BASE_DISTANCE_MOVE_METERS)
        self.declare_parameter("base_distance_move_speed", BASE_DISTANCE_MOVE_SPEED)

        self.declare_parameter("arm_example_x_mm", ARM_EXAMPLE_X_MM)
        self.declare_parameter("arm_example_y_mm", ARM_EXAMPLE_Y_MM)
        self.declare_parameter("arm_example_z_mm", ARM_EXAMPLE_Z_MM)

        self.declare_parameter("capture_timeout_sec", CAPTURE_TIMEOUT_SEC)
        self.declare_parameter("camera_info_timeout_sec", CAMERA_INFO_TIMEOUT_SEC)
        self.declare_parameter("settle_sec_after_move", SETTLE_SEC_AFTER_MOVE)
        self.declare_parameter("pregrasp_offset_mm", PREGRASP_OFFSET_MM)
        self.declare_parameter("descend_extra_mm", DESCEND_EXTRA_MM)
        self.declare_parameter("lift_mm", LIFT_MM)
        self.declare_parameter("place_back_x_mm", PLACE_BACK_X_MM)
        self.declare_parameter("place_back_y_mm", PLACE_BACK_Y_MM)
        self.declare_parameter("place_back_z_mm", PLACE_BACK_Z_MM)
        self.declare_parameter("place_back_release_z_mm", PLACE_BACK_RELEASE_Z_MM)
        self.declare_parameter("debug_repeat_count", DEBUG_REPEAT_COUNT)
        self.declare_parameter("pickup_target_tag_id", PICKUP_TARGET_TAG_ID)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        calibration_file = str(self.get_parameter("calibration_file").value)
        tag_families = str(self.get_parameter("tag_families").value)
        tag_size = float(self.get_parameter("tag_size").value)
        self._camera_info_from_topic = not bool(calibration_file)
        self._camera_model_ready_logged = False

        self.nav_goal = {
            "x": float(self.get_parameter("nav_goal_x").value),
            "y": float(self.get_parameter("nav_goal_y").value),
            "yaw": float(self.get_parameter("nav_goal_yaw").value),
        }
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.target_slot_id = normalize_slot_id(str(self.get_parameter("target_slot_id").value))
        self.slot_world_path = Path(str(self.get_parameter("slot_world_path").value)).expanduser()
        self.slot_nav_x_offset_m = float(self.get_parameter("slot_nav_x_offset_m").value)
        self.slot_nav_yaw = float(self.get_parameter("slot_nav_yaw").value)
        self.home_pose = {
            "x": float(self.get_parameter("home_x").value),
            "y": float(self.get_parameter("home_y").value),
            "yaw": float(self.get_parameter("home_yaw").value),
        }
        self.slot_waypoints = self._load_slot_waypoints()

        self.base_time_move_duration = float(self.get_parameter("base_time_move_duration").value)
        self.base_time_move_vx = float(self.get_parameter("base_time_move_vx").value)
        self.base_distance_move_axis = str(self.get_parameter("base_distance_move_axis").value).strip().lower()
        self.base_distance_move_meters = float(self.get_parameter("base_distance_move_meters").value)
        self.base_distance_move_speed = float(self.get_parameter("base_distance_move_speed").value)

        self.arm_example_x_mm = float(self.get_parameter("arm_example_x_mm").value)
        self.arm_example_y_mm = float(self.get_parameter("arm_example_y_mm").value)
        self.arm_example_z_mm = float(self.get_parameter("arm_example_z_mm").value)

        self.capture_timeout_sec = float(self.get_parameter("capture_timeout_sec").value)
        self.camera_info_timeout_sec = float(self.get_parameter("camera_info_timeout_sec").value)
        self.settle_sec_after_move = float(self.get_parameter("settle_sec_after_move").value)
        self.pregrasp_offset_mm = float(self.get_parameter("pregrasp_offset_mm").value)
        self.descend_extra_mm = float(self.get_parameter("descend_extra_mm").value)
        self.lift_mm = float(self.get_parameter("lift_mm").value)
        self.place_back_x_mm = float(self.get_parameter("place_back_x_mm").value)
        self.place_back_y_mm = float(self.get_parameter("place_back_y_mm").value)
        self.place_back_z_mm = float(self.get_parameter("place_back_z_mm").value)
        self.place_back_release_z_mm = float(self.get_parameter("place_back_release_z_mm").value)
        self.debug_repeat_count = int(self.get_parameter("debug_repeat_count").value)
        self.pickup_target_tag_id = int(self.get_parameter("pickup_target_tag_id").value)

        self._observe_pose_mm = (
            self.arm_example_x_mm,
            self.arm_example_y_mm,
            self.arm_example_z_mm,
        )
        self._arm_command_xyz_mm: Optional[Tuple[float, float, float]] = None

        self._detector = AprilTagDetector(
            families=tag_families,
            tag_size=tag_size,
            calibration_file=calibration_file if calibration_file else None,
        )
        if self._detector.calibrated:
            self.get_logger().info("[sim_demo] 已加载相机模型，可直接做 3D AprilTag 位姿估计")
        else:
            self.get_logger().info("[sim_demo] 将从 /camera/camera_info 加载仿真相机内参")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_nav = self.create_publisher(PoseStamped, "/mission/nav_goal", qos)
        self.pub_gcode = self.create_publisher(String, "/mission/gcode_cmd", qos)
        self.pub_capture = self.create_publisher(Bool, "/mission/capture_trigger", qos)
        self.pub_detected = self.create_publisher(Image, "/mission/image_detected", qos_latched)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", qos)

        self.create_subscription(Bool, "/mission/nav_result", self._on_nav_result, qos)
        self.create_subscription(Bool, "/mission/gcode_result", self._on_gcode_result, qos)
        self.create_subscription(Image, "/mission/image_capture", self._on_image, qos)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_camera_info, qos)

        self._nav_event = threading.Event()
        self._nav_ok = False
        self._gcode_event = threading.Event()
        self._gcode_ok = False
        self._image_event = threading.Event()
        self._last_image: Optional[Image] = None
        self._camera_info_event = threading.Event()
        self._last_camera_info: Optional[CameraInfo] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        threading.Thread(target=self._run_mission, daemon=True).start()

    def _load_slot_waypoints(self) -> dict[str, SlotWaypoint]:
        try:
            cube_positions = load_cube_positions_from_world(self.slot_world_path)
            slot_waypoints = build_slot_waypoints(
                cube_positions,
                nav_x_offset_m=self.slot_nav_x_offset_m,
            )
        except Exception as exc:
            self.get_logger().error(f"[sim_demo] 加载货位导航点失败: {exc}")
            raise

        self.get_logger().info(
            "[sim_demo] 已加载货位导航点: "
            f"world={self.slot_world_path}, slots={', '.join(slot_waypoints.keys())}"
        )
        return slot_waypoints

    def get_slot_waypoint(self, slot_id: str, *, log_error: bool = True) -> Optional[SlotWaypoint]:
        normalized_slot_id = normalize_slot_id(slot_id)
        waypoint = resolve_slot_waypoint(self.slot_waypoints, normalized_slot_id)
        if waypoint is not None:
            return waypoint

        if log_error:
            self.get_logger().error(
                "[sim_demo] 未知货位编号: "
                f"{slot_id!r}，可用编号: {', '.join(self.slot_waypoints.keys())}"
            )
        return None

    def navigate_to_slot(self, slot_id: str, timeout: Optional[float] = None) -> bool:
        waypoint = self.get_slot_waypoint(slot_id)
        if waypoint is None:
            return False

        self.get_logger().info(
            "[sim_demo] 货位导航: "
            f"slot={waypoint.slot_id}, cube={waypoint.cube_name}, "
            f"cube=({waypoint.cube_x:.3f}, {waypoint.cube_y:.3f}), "
            f"nav=({waypoint.nav_x:.3f}, {waypoint.nav_y:.3f}), yaw={self.slot_nav_yaw:.3f}"
        )
        return self.nav_to(
            waypoint.nav_x,
            waypoint.nav_y,
            yaw=self.slot_nav_yaw,
            timeout=timeout,
        )

    def _on_nav_result(self, msg: Bool) -> None:
        self._nav_ok = msg.data
        self._nav_event.set()

    def _on_gcode_result(self, msg: Bool) -> None:
        self._gcode_ok = msg.data
        self._gcode_event.set()

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg
        self._image_event.set()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg
        self._camera_info_event.set()
        if not self._camera_info_from_topic:
            return
        if self._detector.load_camera_info_msg(msg) and not self._camera_model_ready_logged:
            fx = float(self._detector._cam_params[0]) if self._detector._cam_params else 0.0
            fy = float(self._detector._cam_params[1]) if self._detector._cam_params else 0.0
            cx = float(self._detector._cam_params[2]) if self._detector._cam_params else 0.0
            cy = float(self._detector._cam_params[3]) if self._detector._cam_params else 0.0
            self.get_logger().info(
                "[sim_demo] 已从 /camera/camera_info 加载仿真相机内参: "
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )
            self._camera_model_ready_logged = True

    def wait_for_camera_model(self, timeout: Optional[float] = None) -> bool:
        if self._detector.calibrated:
            return True

        effective_timeout = self.camera_info_timeout_sec if timeout is None else float(timeout)
        if self._last_camera_info is not None and self._detector.load_camera_info_msg(self._last_camera_info):
            return True

        self.get_logger().info("[sim_demo] 等待 /camera/camera_info 提供仿真相机内参")
        if not self._camera_info_event.wait(effective_timeout):
            self.get_logger().error(f"[sim_demo] 等待 /camera/camera_info 超时（>{effective_timeout:.1f}s）")
            return False
        if self._last_camera_info is None:
            self.get_logger().error("[sim_demo] 收到了 camera_info 事件，但消息为空")
            return False
        if not self._detector.load_camera_info_msg(self._last_camera_info):
            self.get_logger().error("[sim_demo] camera_info 中没有有效内参，无法做 3D Tag 位姿估计")
            return False
        return True

    def nav_to(self, x: float, y: float, yaw: float = 0.0, timeout: Optional[float] = None) -> bool:
        self._nav_event.clear()
        pose = make_pose(x, y, yaw)
        pose.header.stamp = self.get_clock().now().to_msg()
        self.pub_nav.publish(pose)
        self.get_logger().info(f"[sim_demo] 导航到点: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        effective_timeout = self.nav_timeout_sec if timeout is None else float(timeout)
        if effective_timeout <= 0.0:
            self.get_logger().info("[sim_demo] 导航等待模式：不设超时，等待 Nav2 最终结果")
            self._nav_event.wait()
        else:
            if not self._nav_event.wait(effective_timeout):
                self.get_logger().error(f"[sim_demo] 导航超时（>{effective_timeout:.1f}s）")
                return False
        return self._nav_ok

    def gcode(self, cmd: str, timeout: float = GCODE_TIMEOUT_SEC) -> bool:
        if self.count_subscribers("/mission/gcode_cmd") == 0:
            self.get_logger().error(
                "[sim_demo] 没有节点订阅 /mission/gcode_cmd；"
                "仿真请确认 e4_gcode_adapter 已启动，真机请确认 mission_executor 使用 backend=real"
            )
            return False
        if self.count_publishers("/mission/gcode_result") == 0:
            self.get_logger().error(
                "[sim_demo] 没有节点发布 /mission/gcode_result；"
                "仿真请确认 e4_gcode_adapter 已启动，真机请确认 mission_executor 使用 backend=real"
            )
            return False

        self._gcode_event.clear()
        msg = String()
        msg.data = cmd
        self.pub_gcode.publish(msg)
        self.get_logger().info(f"[sim_demo] G-code: {cmd}")
        if not self._gcode_event.wait(timeout):
            self.get_logger().error(f"[sim_demo] G-code 超时: {cmd}")
            return False
        if not self._gcode_ok:
            self.get_logger().error(
                "[sim_demo] G-code 执行失败（收到 /mission/gcode_result=false）: "
                f"{cmd}"
            )
            return False
        return self._gcode_ok

    def set_arm_command_state(self, x_mm: float, y_mm: float, z_mm: float) -> None:
        self._arm_command_xyz_mm = (float(x_mm), float(y_mm), float(z_mm))

    def arm_home(self) -> bool:
        if not self.gcode("$h"):
            return False
        self.set_arm_command_state(
            ARM_HOME_CARTESIAN_MM["x"],
            ARM_HOME_CARTESIAN_MM["y"],
            ARM_HOME_CARTESIAN_MM["z"],
        )
        return True

    def arm_move_absolute(self, x_mm: float, y_mm: float, z_mm: float, timeout: float = GCODE_TIMEOUT_SEC) -> bool:
        cmd = f"M20 G90 X{x_mm:.1f} Y{y_mm:.1f} Z{z_mm:.1f}"
        if not self.gcode(cmd, timeout=timeout):
            return False
        self.set_arm_command_state(x_mm, y_mm, z_mm)
        return True

    def arm_move_relative(
        self,
        dx_mm: float = 0.0,
        dy_mm: float = 0.0,
        dz_mm: float = 0.0,
        timeout: float = GCODE_TIMEOUT_SEC,
    ) -> bool:
        cmd = f"M20 G91 X{dx_mm:.1f} Y{dy_mm:.1f} Z{dz_mm:.1f}"
        if not self.gcode(cmd, timeout=timeout):
            return False
        if self._arm_command_xyz_mm is not None:
            self.set_arm_command_state(
                self._arm_command_xyz_mm[0] + dx_mm,
                self._arm_command_xyz_mm[1] + dy_mm,
                self._arm_command_xyz_mm[2] + dz_mm,
            )
        return True

    def set_suction(self, enabled: bool) -> bool:
        return self.gcode("M3 S1000" if enabled else "M3 S500")

    def stop_base(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.pub_cmd_vel.publish(stop)
            time.sleep(0.05)

    def move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> None:
        self.get_logger().info(
            "[sim_demo] 底盘按时间移动: "
            f"vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}, duration={duration:.2f}s"
        )
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz

        duration = max(0.0, float(duration))
        if duration <= 0.0:
            self.stop_base()
            return

        publish_period = 1.0 / BASE_CMD_STREAM_HZ
        deadline = time.monotonic() + duration
        while rclpy.ok():
            now = time.monotonic()
            if now >= deadline:
                break
            self.pub_cmd_vel.publish(twist)
            time.sleep(min(publish_period, max(0.0, deadline - now)))

        self.stop_base()

    def move_distance(self, axis: str, distance_m: float, speed_mps: float) -> None:
        if abs(speed_mps) < 1e-6:
            raise ValueError("speed_mps 不能为 0")

        signed_speed = abs(speed_mps) if distance_m >= 0.0 else -abs(speed_mps)
        duration = abs(distance_m / speed_mps)

        if axis == "x":
            self.move_for_time(vx=signed_speed, vy=0.0, wz=0.0, duration=duration)
        elif axis == "y":
            self.move_for_time(vx=0.0, vy=signed_speed, wz=0.0, duration=duration)
        else:
            raise ValueError(f"不支持的 axis={axis!r}，只能是 'x' 或 'y'")

    def capture_and_detect(self, timeout: Optional[float] = None) -> Tuple[Optional[Image], List[TargetInfo]]:
        effective_timeout = self.capture_timeout_sec if timeout is None else float(timeout)
        if not self.wait_for_camera_model():
            return None, []

        self._image_event.clear()
        self._last_image = None

        trigger = Bool()
        trigger.data = True
        self.pub_capture.publish(trigger)
        self.get_logger().info("[sim_demo] 触发抓图")

        if not self._image_event.wait(effective_timeout):
            self.get_logger().error(f"[sim_demo] 抓图超时（>{effective_timeout:.1f}s）")
            return None, []

        raw_image = self._last_image
        if raw_image is None:
            self.get_logger().error("[sim_demo] 抓图事件已触发，但没有拿到图像消息")
            return None, []

        annotated, targets = self._detector.detect(raw_image)
        self.pub_detected.publish(annotated)

        filtered_targets: List[TargetInfo] = [target for target in targets if target.tag_detected]
        if filtered_targets:
            for target in filtered_targets:
                pose_text = (
                    f", pose=({target.pos_x:.4f}, {target.pos_y:.4f}, {target.pos_z:.4f}) m"
                    if target.pose_valid
                    else ", pose=invalid"
                )
                self.get_logger().info(
                    f"[sim_demo] 检测到 Tag: id={target.tag_id}, "
                    f"center=({target.center_u:.1f}, {target.center_v:.1f}){pose_text}"
                )
        else:
            self.get_logger().info("[sim_demo] 当前帧没有检测到可用 Tag")

        return raw_image, filtered_targets

    def select_target(
        self,
        raw_image: Optional[Image],
        targets: List[TargetInfo],
    ) -> Optional[TargetInfo]:
        if raw_image is None:
            self.get_logger().error("[sim_demo] 没有原始图像，无法挑选目标")
            return None
        if not targets:
            self.get_logger().error("[sim_demo] 没有候选目标，无法挑选")
            return None

        candidates = [target for target in targets if target.pose_valid]
        if not candidates:
            self.get_logger().error("[sim_demo] 当前帧没有带有效 3D 位姿的目标，无法执行几何抓取")
            return None
        if self.pickup_target_tag_id >= 0:
            candidates = [target for target in candidates if target.tag_id == self.pickup_target_tag_id]
            if not candidates:
                self.get_logger().error(
                    "[sim_demo] 当前帧没有匹配 "
                    f"pickup_target_tag_id={self.pickup_target_tag_id} 且 pose_valid=True 的目标"
                )
                return None

        image_cx = raw_image.width / 2.0
        image_cy = raw_image.height / 2.0

        def score(target: TargetInfo) -> float:
            du = float(target.center_u) - image_cx
            dv = float(target.center_v) - image_cy
            return du * du + dv * dv

        best = min(candidates, key=score)
        pixel_error = math.sqrt(score(best))
        self.get_logger().info(
            "[sim_demo] 选定目标: "
            f"id={best.tag_id}, center=({best.center_u:.1f}, {best.center_v:.1f}), "
            f"image_center=({image_cx:.1f}, {image_cy:.1f}), pixel_error={pixel_error:.1f}"
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
                f"[sim_demo] TF 查询失败: {target_frame} <- {source_frame}: {exc}"
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
        target: TargetInfo,
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
            self.get_logger().error("[sim_demo] 当前吸盘位置与目标点几乎重合，无法确定接近方向")
            return None

        sign = 1.0 if vec_dot(vector_to_target, suction_z_axis_arm_base) >= 0.0 else -1.0
        approach_axis = vec_scale(suction_z_axis_arm_base, sign)
        return vec_normalize(approach_axis)

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
            "[sim_demo] 吸盘目标位换算（G91 相对运动）: "
            f"{label}, desired=({desired_suction_point_arm_base[0]:.4f}, "
            f"{desired_suction_point_arm_base[1]:.4f}, {desired_suction_point_arm_base[2]:.4f}) m, "
            f"delta_arm_base=({delta_arm_base_m[0] * 1000.0:.1f}, "
            f"{delta_arm_base_m[1] * 1000.0:.1f}, {delta_arm_base_m[2] * 1000.0:.1f}) mm, "
            f"delta_cmd=({delta_cmd_mm[0]:.1f}, {delta_cmd_mm[1]:.1f}, {delta_cmd_mm[2]:.1f}) mm"
        )

        if not self.arm_move_relative(
            dx_mm=delta_cmd_mm[0],
            dy_mm=delta_cmd_mm[1],
            dz_mm=delta_cmd_mm[2],
        ):
            return False
        time.sleep(self.settle_sec_after_move)
        return True

    def move_to_observe_pose(self) -> bool:
        if not self.arm_move_absolute(*self._observe_pose_mm):
            return False
        time.sleep(self.settle_sec_after_move)
        self.get_logger().info(
            "[sim_demo] 机械臂已到观察位姿 "
            f"({self._observe_pose_mm[0]:.1f}, {self._observe_pose_mm[1]:.1f}, {self._observe_pose_mm[2]:.1f}) mm"
        )
        return True

    def best_effort_release(self) -> None:
        self.get_logger().info("[sim_demo] 执行 best-effort 释放，确保吸盘关闭")
        self.set_suction(False)
        time.sleep(0.5)

    def run_prepare_sequence(self) -> bool:
        if self.target_slot_id:
            waypoint = self.get_slot_waypoint(self.target_slot_id)
            if waypoint is None:
                self.get_logger().error("[sim_demo] 准备阶段失败：货位编号无效")
                return False

            if not self.navigate_to_slot(waypoint.slot_id):
                self.get_logger().error("[sim_demo] 准备阶段失败：货位导航没有成功")
                return False

            self.get_logger().info(
                "[sim_demo] 准备阶段步骤 1 完成：已到达货位导航点 "
                f"{waypoint.slot_id} -> ({waypoint.nav_x:.2f}, {waypoint.nav_y:.2f})"
            )
            self.get_logger().info(
                "[sim_demo] 准备阶段步骤 2 跳过：货位导航已直接使用 x 偏移目标点"
            )
        else:
            if not self.nav_to(**self.nav_goal):
                self.get_logger().error("[sim_demo] 准备阶段失败：导航没有成功")
                return False

            self.get_logger().info(
                "[sim_demo] 准备阶段步骤 1 完成：已到达目标点 "
                f"({self.nav_goal['x']:.2f}, {self.nav_goal['y']:.2f})"
            )

            try:
                self.move_distance(
                    axis=self.base_distance_move_axis,
                    distance_m=self.base_distance_move_meters,
                    speed_mps=self.base_distance_move_speed,
                )
            except ValueError as exc:
                self.get_logger().error(f"[sim_demo] 准备阶段失败：底盘移动参数无效: {exc}")
                return False

            self.get_logger().info(
                "[sim_demo] 准备阶段步骤 2 完成：已沿车体 "
                f"{self.base_distance_move_axis} 方向移动 {self.base_distance_move_meters:.2f} m"
            )

        if not self.arm_home():
            self.get_logger().error("[sim_demo] 准备阶段失败：机械臂回零失败")
            return False
        time.sleep(0.5)
        self.get_logger().info("[sim_demo] 准备阶段步骤 3 完成：机械臂已执行回零")

        if not self.move_to_observe_pose():
            self.get_logger().error("[sim_demo] 准备阶段失败：机械臂移动到观察位姿失败")
            return False

        self.get_logger().info("[sim_demo] 准备阶段步骤 4 完成：观察位姿就绪")
        return True

    def execute_single_grasp_cycle(self, cycle_index: int) -> bool:
        raw_image, targets = self.capture_and_detect()
        if raw_image is None:
            self.get_logger().error("[sim_demo] 抓取循环失败：没有拿到图像")
            return False

        target = self.select_target(raw_image, targets)
        if target is None:
            self.get_logger().error("[sim_demo] 抓取循环失败：没有选到有效目标")
            return False
        if not target.pose_valid:
            self.get_logger().error("[sim_demo] 抓取循环失败：目标没有有效 3D 位姿")
            return False

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

        # AprilTag 给出的目标点就是当前场景里方块可见 Tag 面的中心。
        # 在修正了吸盘接触点定义后，默认策略进一步收紧为：
        #   1. 先停在表面前约 12 mm 的预抓位
        #   2. 最终停在表面前约 3 mm，而不是贴死或压入模型
        # Gazebo 吸附插件对 3 cm cube 的抓取窗口足够覆盖这个保守距离。
        pregrasp_point = vec_sub(target_point_arm_base, vec_scale(approach_axis, self.pregrasp_offset_mm / 1000.0))
        grasp_point = vec_add(target_point_arm_base, vec_scale(approach_axis, self.descend_extra_mm / 1000.0))

        self.get_logger().info(
            "[sim_demo] 抓取几何信息: "
            f"cycle={cycle_index}, tag_id={target.tag_id}, "
            f"target_in_arm_base=({target_point_arm_base[0]:.4f}, "
            f"{target_point_arm_base[1]:.4f}, {target_point_arm_base[2]:.4f}) m, "
            f"approach_axis=({approach_axis[0]:.4f}, {approach_axis[1]:.4f}, {approach_axis[2]:.4f})"
        )

        if not self.move_suction_to_arm_base_point(pregrasp_point, "预抓取位"):
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

        if not self.move_suction_to_arm_base_point(grasp_point, "抓取位"):
            return False
        if not self.set_suction(True):
            return False
        time.sleep(1.0)

        if not self.move_suction_to_arm_base_point(lift_point, "抬起位"):
            return False

        if not self.arm_move_absolute(self.place_back_x_mm, self.place_back_y_mm, self.place_back_z_mm):
            return False
        time.sleep(self.settle_sec_after_move)

        if not self.arm_move_absolute(
            self.place_back_x_mm,
            self.place_back_y_mm,
            self.place_back_release_z_mm,
        ):
            return False
        time.sleep(self.settle_sec_after_move)

        if not self.set_suction(False):
            return False
        time.sleep(1.0)

        if not self.arm_move_absolute(self.place_back_x_mm, self.place_back_y_mm, self.place_back_z_mm):
            return False
        time.sleep(self.settle_sec_after_move)

        self.get_logger().info(f"[sim_demo] 抓取循环 {cycle_index} 完成：已抓取并放回")
        return True

    def run_grasp_debug_loop(self, repeat_count: int, shutdown_when_done: bool) -> None:
        if not self.move_to_observe_pose():
            self.get_logger().error("[sim_demo] 抓取调试启动失败：无法到达观察位姿")
            if shutdown_when_done:
                rclpy.shutdown()
            return

        cycle_index = 0
        while rclpy.ok():
            if repeat_count > 0 and cycle_index >= repeat_count:
                break
            cycle_index += 1
            self.get_logger().info(f"===== sim_demo 抓取循环 {cycle_index} 开始 =====")

            cycle_ok = self.execute_single_grasp_cycle(cycle_index)
            if not cycle_ok:
                self.get_logger().error(f"[sim_demo] 抓取循环 {cycle_index} 失败")
                self.best_effort_release()

            if not self.move_to_observe_pose():
                self.get_logger().error("[sim_demo] 无法回到观察位姿，结束抓取调试")
                break

        self.get_logger().info("[sim_demo] 抓取调试循环结束")
        if shutdown_when_done:
            rclpy.shutdown()

    def _run_mission(self) -> None:
        time.sleep(2.0)
        self.get_logger().info(f"===== sim_demo 开始，mode={self.mode} =====")

        if self.mode == "full":
            if not self.run_prepare_sequence():
                rclpy.shutdown()
                return
            self.run_grasp_debug_loop(repeat_count=1, shutdown_when_done=True)
            return

        if self.mode == "prepare_only":
            if not self.run_prepare_sequence():
                rclpy.shutdown()
                return
            self.get_logger().info(
                "[sim_demo] prepare_only 已完成，机器人保持在观察位。"
                "现在可以保留当前仿真并单独启动 grasp_debug_loop。"
            )
            return

        if self.mode == "grasp_debug_loop":
            self.get_logger().info(
                "[sim_demo] grasp_debug_loop 假定底盘已在正确位置，不再重复导航和横移。"
            )
            self.run_grasp_debug_loop(
                repeat_count=self.debug_repeat_count,
                shutdown_when_done=True,
            )
            return

        if self.mode == "slot_nav_only":
            if not self.target_slot_id:
                self.get_logger().error("[sim_demo] slot_nav_only 需要提供 target_slot_id，例如 B1 / C3")
                rclpy.shutdown()
                return
            success = self.navigate_to_slot(self.target_slot_id)
            self.get_logger().info(f"[sim_demo] slot_nav_only 结束，success={success}")
            rclpy.shutdown()
            return

        self.get_logger().error(
            f"[sim_demo] 不支持的 mode={self.mode!r}，只能是 "
            "full / prepare_only / grasp_debug_loop / slot_nav_only"
        )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimMissionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
