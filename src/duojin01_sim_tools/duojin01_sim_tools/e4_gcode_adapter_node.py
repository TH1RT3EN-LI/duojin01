#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HOME_JOINTS = {
    "arm_joint1": 0.0,
    "arm_joint2": 0.2749,
    "arm_joint3": 0.5732,
    "arm_joint4": -0.8481,
}
HOME_CARTESIAN = [0.195, 0.0, 0.269]
JOINT_NAMES = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4"]

JOINT_LIMITS = {
    "arm_joint2": (-1.57, 0.78),
    "arm_joint3": (-0.3, 2.0),
    "arm_joint4": (-math.pi, math.pi),
}

L2 = 0.141
L3 = 0.142
X_BASE = 0.06
Z_BASE = 0.125
ANGLE_CORRECTION = 0.76


def parse_gcode(cmd_str: str) -> Dict[str, object]:
    parts = cmd_str.upper().replace("\t", " ").split()
    result: Dict[str, object] = {}

    for part in parts:
        key = "".join(char for char in part if char.isalpha())
        value = "".join(char for char in part if (char.isdigit() or char in ".-"))
        if key == "G":
            result.setdefault("G", [])
            try:
                result["G"].append(int(value))
            except ValueError:
                continue
        elif key == "M":
            try:
                result["M"] = int(value)
            except ValueError:
                continue
        elif key:
            try:
                result[key] = float(value)
            except ValueError:
                continue

    return result


def ease_out(ratio: float) -> float:
    clamped = max(0.0, min(1.0, float(ratio)))
    return 1.0 - pow(1.0 - clamped, 3)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_to_pi(angle: float) -> float:
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


class E4GcodeAdapter(Node):
    def __init__(self) -> None:
        super().__init__("e4_gcode_adapter")

        self.declare_parameter("command_topic", "/arm/gcode_cmd")
        self.declare_parameter("result_topic", "/arm/gcode_result")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/e4_joint_trajectory")
        self.declare_parameter("suction_topic", "/arm_suction_enabled")
        self.declare_parameter("end_effector_pose_topic", "/arm/end_effector_pose")
        self.declare_parameter("end_effector_query_service", "/arm/query_end_effector_pose")
        self.declare_parameter("end_effector_pose_frame", "arm_gcode_frame")
        self.declare_parameter("end_effector_publish_rate_hz", 10.0)
        self.declare_parameter("joint_names", JOINT_NAMES)
        self.declare_parameter("home_joint_positions", [HOME_JOINTS[name] for name in JOINT_NAMES])
        self.declare_parameter("home_cartesian_xyz", HOME_CARTESIAN)
        self.declare_parameter("interpolation_steps", 200)
        self.declare_parameter("default_feedrate", 2000.0)
        self.declare_parameter("min_duration_sec", 0.2)
        self.declare_parameter("speed_scale", 2.0)
        self.declare_parameter("result_mode", "always")

        self._command_topic = str(self.get_parameter("command_topic").value)
        self._result_topic = str(self.get_parameter("result_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self._suction_topic = str(self.get_parameter("suction_topic").value)
        self._end_effector_pose_topic = str(self.get_parameter("end_effector_pose_topic").value)
        self._end_effector_query_service = str(self.get_parameter("end_effector_query_service").value)
        self._end_effector_pose_frame = str(self.get_parameter("end_effector_pose_frame").value)
        self._end_effector_publish_rate_hz = max(
            0.1, float(self.get_parameter("end_effector_publish_rate_hz").value)
        )
        self._joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        home_joint_positions = [float(value) for value in self.get_parameter("home_joint_positions").value]
        self._home_joints = dict(zip(self._joint_names, home_joint_positions))
        self._home_cartesian = [float(value) for value in self.get_parameter("home_cartesian_xyz").value]
        self._interpolation_steps = max(2, int(self.get_parameter("interpolation_steps").value))
        self._default_feedrate = max(1.0, float(self.get_parameter("default_feedrate").value))
        self._min_duration_sec = max(0.05, float(self.get_parameter("min_duration_sec").value))
        self._speed_scale = max(0.1, float(self.get_parameter("speed_scale").value))
        result_mode_value = self.get_parameter("result_mode").value
        if isinstance(result_mode_value, bool):
            self._result_mode = "always" if result_mode_value else "off"
        else:
            self._result_mode = str(result_mode_value).strip().lower()

        self._joint_state_map = dict(self._home_joints)
        self._commanded_joints = dict(self._home_joints)
        self._cartesian_xyz = list(self._home_cartesian)
        self._state_initialized = False

        self._trajectory_pub = self.create_publisher(JointTrajectory, self._trajectory_topic, 10)
        self._result_pub = self.create_publisher(Bool, self._result_topic, 10)
        self._suction_pub = self.create_publisher(Bool, self._suction_topic, 10)
        self._end_effector_pose_pub = self.create_publisher(PoseStamped, self._end_effector_pose_topic, 10)

        self.create_subscription(String, self._command_topic, self._on_gcode_cmd, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self.create_service(Trigger, self._end_effector_query_service, self._on_query_end_effector_pose)
        self.create_timer(1.0 / self._end_effector_publish_rate_hz, self._publish_end_effector_pose)

        self.get_logger().info(
            "E4 G-code adapter started: "
            f"{self._command_topic} -> {self._trajectory_topic}; "
            f"result_topic={self._result_topic}; result_mode={self._result_mode}; "
            f"speed_scale={self._speed_scale:.2f}; "
            f"pose_topic={self._end_effector_pose_topic}; query_service={self._end_effector_query_service}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        updated = False
        for name, position in zip(msg.name, msg.position):
            if name in self._joint_names:
                self._joint_state_map[name] = float(position)
                updated = True

        if not updated:
            return

        if not self._state_initialized and self._have_complete_joint_state():
            self._commanded_joints = dict(self._joint_state_map)
            cartesian = self._forward_kinematics(self._joint_state_map)
            if cartesian is not None:
                self._cartesian_xyz = cartesian
            self._state_initialized = True
            self.get_logger().info(
                "E4 adapter initialized from /joint_states: "
                f"cartesian=({self._cartesian_xyz[0]:.3f}, {self._cartesian_xyz[1]:.3f}, {self._cartesian_xyz[2]:.3f})"
            )

    def _make_end_effector_pose_msg(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._end_effector_pose_frame
        msg.pose.position.x = self._cartesian_xyz[0]
        msg.pose.position.y = self._cartesian_xyz[1]
        msg.pose.position.z = self._cartesian_xyz[2]
        # The public G-code interface currently defines XYZ only.
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_end_effector_pose(self) -> None:
        self._end_effector_pose_pub.publish(self._make_end_effector_pose_msg())

    def _on_query_end_effector_pose(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        response.success = True
        response.message = json.dumps(
            {
                "frame_id": self._end_effector_pose_frame,
                "units": "m",
                "source": "e4_gcode_adapter_state",
                "coordinate_convention": "same as /arm/gcode_cmd G90/G91 XYZ",
                "position": {
                    "x": self._cartesian_xyz[0],
                    "y": self._cartesian_xyz[1],
                    "z": self._cartesian_xyz[2],
                },
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        return response

    def _have_complete_joint_state(self) -> bool:
        return all(name in self._joint_state_map for name in self._joint_names)

    def _ensure_initialized(self) -> None:
        if self._state_initialized:
            return

        if self._have_complete_joint_state():
            self._commanded_joints = dict(self._joint_state_map)
            cartesian = self._forward_kinematics(self._joint_state_map)
            if cartesian is not None:
                self._cartesian_xyz = cartesian
            self._state_initialized = True
            return

        self.get_logger().warn(
            "joint_states not ready; falling back to configured home pose for G-code state bootstrap"
        )
        self._joint_state_map = dict(self._home_joints)
        self._commanded_joints = dict(self._home_joints)
        self._cartesian_xyz = list(self._home_cartesian)
        self._state_initialized = True

    def _forward_kinematics(self, joints: Dict[str, float]) -> Optional[List[float]]:
        try:
            # Align the adapted E4 model with Gazebo's top-view frame:
            # +X forward, +Y left. The original reference simulation used the
            # opposite shoulder / elbow sign convention, but this workspace's
            # current E4 assembly rotates in the positive direction directly.
            theta0 = float(joints["arm_joint1"])
            theta1 = float(joints["arm_joint2"])
            theta2 = float(joints["arm_joint3"]) + ANGLE_CORRECTION
        except KeyError:
            return None

        planar_x = L2 * math.cos(theta1) + L3 * math.cos(theta1 + theta2)
        planar_z = L2 * math.sin(theta1) + L3 * math.sin(theta1 + theta2)
        radius = planar_x + X_BASE

        return [
            radius * math.cos(theta0),
            radius * math.sin(theta0),
            planar_z + Z_BASE,
        ]

    def _inverse_kinematics(self, x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        theta0 = math.atan2(y, x)

        radius = math.hypot(x, y)
        planar_x = radius - X_BASE
        planar_z = z - Z_BASE

        d_value = -1.0 * (planar_x ** 2 + planar_z ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
        if d_value < -1.000001 or d_value > 1.000001:
            self.get_logger().warn(f"IK failed: target out of reach (D={d_value:.4f})")
            return None
        d_value = clamp(d_value, -1.0, 1.0)

        theta2 = math.acos(d_value)
        theta1 = math.atan2(planar_z, planar_x) - math.atan2(L3 * math.sin(theta2), L2 + L3 * math.cos(theta2))

        joint1 = theta0
        joint2 = theta1
        joint3 = theta2 - ANGLE_CORRECTION

        joint2_lower, joint2_upper = JOINT_LIMITS["arm_joint2"]
        joint3_lower, joint3_upper = JOINT_LIMITS["arm_joint3"]
        if not (joint2_lower <= joint2 <= joint2_upper and joint3_lower <= joint3 <= joint3_upper):
            self.get_logger().warn(
                "IK solution violates joint limits: "
                f"arm_joint2={joint2:.3f}, arm_joint3={joint3:.3f}"
            )
            return None

        return joint1, joint2, joint3

    def _command_duration(self, feedrate: float) -> float:
        safe_feedrate = max(1.0, float(feedrate))
        return max(self._min_duration_sec, 2000.0 / (safe_feedrate * self._speed_scale))

    def _duration_msg(self, seconds: float) -> Duration:
        clamped = max(0.05, float(seconds))
        whole = int(clamped)
        nanos = int(round((clamped - whole) * 1_000_000_000))
        if nanos >= 1_000_000_000:
            whole += 1
            nanos -= 1_000_000_000
        return Duration(sec=whole, nanosec=nanos)

    def _make_smoothed_trajectory(
        self,
        start_joints: Sequence[float],
        target_joints: Sequence[float],
        duration_sec: float,
    ) -> JointTrajectory:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._joint_names)

        for index in range(1, self._interpolation_steps + 1):
            linear_ratio = index / float(self._interpolation_steps)
            ratio = ease_out(linear_ratio)

            joint1 = start_joints[0] + (target_joints[0] - start_joints[0]) * ratio
            joint2 = start_joints[1] + (target_joints[1] - start_joints[1]) * ratio
            joint3 = start_joints[2] + (target_joints[2] - start_joints[2]) * ratio

            # Keep the tool-side part aligned using the same compensation rule
            # as the reference simulation: joint4 offsets joint2 + joint3.
            delta_now = (joint2 - start_joints[1]) + (joint3 - start_joints[2])
            joint4 = wrap_to_pi(start_joints[3] - delta_now)

            point = JointTrajectoryPoint()
            point.positions = [joint1, joint2, joint3, joint4]
            point.time_from_start = self._duration_msg(duration_sec * linear_ratio)
            msg.points.append(point)

        return msg

    def _publish_result(self, success: bool) -> None:
        if self._result_mode == "off":
            return
        if self._result_mode not in {"auto", "always"}:
            self.get_logger().warn(
                f"unknown result_mode={self._result_mode!r}; falling back to always publish"
            )
            self._result_mode = "always"
        if self._result_mode == "auto" and self.count_publishers(self._result_topic) > 1:
            self.get_logger().warn(
                "multiple publishers detected on result_topic; "
                "auto mode now still publishes the adapter result to avoid hanging mission scripts"
            )

        msg = Bool()
        msg.data = bool(success)
        self._result_pub.publish(msg)

    def _publish_home(self, duration_sec: float) -> None:
        self._ensure_initialized()
        start_joints = [self._commanded_joints[name] for name in self._joint_names]
        target_joints = [self._home_joints[name] for name in self._joint_names]
        trajectory = self._make_smoothed_trajectory(start_joints, target_joints, duration_sec)
        self._trajectory_pub.publish(trajectory)
        self._commanded_joints = dict(self._home_joints)
        self._cartesian_xyz = list(self._home_cartesian)
        self.get_logger().info(
            "E4 home trajectory published: "
            + ", ".join(f"{name}={self._home_joints[name]:.3f}" for name in self._joint_names)
            + f"; xyz=({self._cartesian_xyz[0]:.3f}, {self._cartesian_xyz[1]:.3f}, {self._cartesian_xyz[2]:.3f}) m"
        )
        self._publish_result(True)

    def _publish_cartesian_target(self, target_xyz: Sequence[float], feedrate: float) -> bool:
        self._ensure_initialized()

        ik_solution = self._inverse_kinematics(target_xyz[0], target_xyz[1], target_xyz[2])
        if ik_solution is None:
            self._publish_result(False)
            return False

        start_joints = [self._commanded_joints[name] for name in self._joint_names]
        target_joint4 = wrap_to_pi(
            start_joints[3] - ((ik_solution[1] - start_joints[1]) + (ik_solution[2] - start_joints[2]))
        )

        joint4_lower, joint4_upper = JOINT_LIMITS["arm_joint4"]
        if not (joint4_lower <= target_joint4 <= joint4_upper):
            target_joint4 = clamp(target_joint4, joint4_lower, joint4_upper)

        target_joints = [ik_solution[0], ik_solution[1], ik_solution[2], target_joint4]
        duration_sec = self._command_duration(feedrate)
        trajectory = self._make_smoothed_trajectory(start_joints, target_joints, duration_sec)
        self._trajectory_pub.publish(trajectory)

        self._commanded_joints = dict(zip(self._joint_names, target_joints))
        self._cartesian_xyz = [float(value) for value in target_xyz]

        self.get_logger().info(
            "E4 G-code target published: "
            f"xyz=({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f}) m, "
            f"feedrate={feedrate:.1f}, duration={duration_sec:.2f}s, "
            + ", ".join(f"{name}={value:.3f}" for name, value in zip(self._joint_names, target_joints))
        )
        self._publish_result(True)
        return True

    def _handle_motion_command(self, parsed: Dict[str, object]) -> bool:
        g_codes = [code for code in parsed.get("G", []) if code in (90, 91)]
        if not g_codes:
            self.get_logger().warn(f"unsupported G-code motion mode: {parsed}")
            self._publish_result(False)
            return False

        self._ensure_initialized()

        target_xyz = list(self._cartesian_xyz)
        if g_codes[0] == 90:
            if "X" in parsed:
                target_xyz[0] = float(parsed["X"]) / 1000.0
            if "Y" in parsed:
                target_xyz[1] = float(parsed["Y"]) / 1000.0
            if "Z" in parsed:
                target_xyz[2] = float(parsed["Z"]) / 1000.0
        else:
            if "X" in parsed:
                target_xyz[0] += float(parsed["X"]) / 1000.0
            if "Y" in parsed:
                target_xyz[1] += float(parsed["Y"]) / 1000.0
            if "Z" in parsed:
                target_xyz[2] += float(parsed["Z"]) / 1000.0

        feedrate = float(parsed.get("F", self._default_feedrate))
        return self._publish_cartesian_target(target_xyz, feedrate)

    def _on_gcode_cmd(self, msg: String) -> None:
        raw_command = msg.data.strip()
        if not raw_command:
            return

        compact_command = "".join(raw_command.upper().split())

        if compact_command == "?":
            self._ensure_initialized()
            self.get_logger().info(
                "E4 status query: "
                f"xyz=({self._cartesian_xyz[0]:.3f}, {self._cartesian_xyz[1]:.3f}, {self._cartesian_xyz[2]:.3f}) m"
            )
            self._publish_result(True)
            return

        if compact_command == "$H":
            self._publish_home(duration_sec=self._command_duration(5000.0))
            return

        if compact_command == "M50":
            self._publish_home(duration_sec=self._command_duration(2000.0))
            return

        if compact_command in {"M3S1000", "M3S500", "M3S0"}:
            suction_enabled = compact_command == "M3S1000"
            suction_msg = Bool()
            suction_msg.data = suction_enabled
            self._suction_pub.publish(suction_msg)
            self.get_logger().info(
                f"E4 suction command published: {compact_command} -> {self._suction_topic}={suction_enabled}"
            )
            self._publish_result(True)
            return

        parsed = parse_gcode(raw_command)
        if "G" not in parsed:
            self.get_logger().warn(f"failed to parse or unsupported G-code: {raw_command}")
            self._publish_result(False)
            return

        self._handle_motion_command(parsed)


def main() -> None:
    rclpy.init()
    node = E4GcodeAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
