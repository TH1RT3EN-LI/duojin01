#!/usr/bin/env python3
"""Real-robot A/B test for static vs dynamic odom covariance in a small arena.

Design goals:
1) Work in tight space (default guard box: +/-0.75 m from start in both x/y).
2) Compare static and dynamic covariance with the same command sequence.
3) Keep safety first: watchdog lock detection + boundary guard + guaranteed stop.
4) Optional rosbag recording per run.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclasses.dataclass(frozen=True)
class Phase:
    name: str
    vx: float
    vy: float
    wz: float
    duration: float


@dataclasses.dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclasses.dataclass
class RunResult:
    label: str
    aborted: bool
    abort_reason: str
    max_abs_dx: float
    max_abs_dy: float
    odom_end_dx: Optional[float]
    odom_end_dy: Optional[float]
    odom_end_dyaw_deg: Optional[float]
    filtered_end_dx: Optional[float]
    filtered_end_dy: Optional[float]
    filtered_end_dyaw_deg: Optional[float]
    final_stop_vx_std: Optional[float]
    final_stop_vy_std: Optional[float]
    final_stop_wz_std: Optional[float]
    bag_path: Optional[str]


def normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CovarianceABRunner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("covariance_ab_real_test")
        self.args = args
        self._abort_reason: str = ""
        self._watchdog_locked = False
        self._current_phase_name = ""
        self._run_start_odom: Optional[Pose2D] = None
        self._run_start_filtered: Optional[Pose2D] = None
        self._max_abs_dx = 0.0
        self._max_abs_dy = 0.0
        self._final_stop_twists: List[Tuple[float, float, float]] = []
        self._last_odom: Optional[Odometry] = None
        self._last_filtered: Optional[Odometry] = None

        self._cmd_pub = self.create_publisher(Twist, args.cmd_topic, 20)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 30)
        self.create_subscription(Odometry, args.filtered_topic, self._filtered_cb, 30)
        self.create_subscription(Bool, args.watchdog_lock_topic, self._watchdog_cb, 10)

        self.get_logger().info(
            "A/B runner ready: cmd_topic=%s odom=%s filtered=%s base_driver_node=%s",
            args.cmd_topic,
            args.odom_topic,
            args.filtered_topic,
            args.base_driver_node,
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg
        if self._run_start_odom is None:
            return

        pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(quat_to_yaw(msg.pose.pose.orientation)),
        )
        dx = pose.x - self._run_start_odom.x
        dy = pose.y - self._run_start_odom.y
        self._max_abs_dx = max(self._max_abs_dx, abs(dx))
        self._max_abs_dy = max(self._max_abs_dy, abs(dy))

        if abs(dx) > self.args.guard_abs_xy or abs(dy) > self.args.guard_abs_xy:
            self._abort_reason = (
                f"boundary exceeded: |dx|={abs(dx):.3f} |dy|={abs(dy):.3f} "
                f"> guard_abs_xy={self.args.guard_abs_xy:.3f}"
            )

        if self._current_phase_name == "final_stop":
            t = msg.twist.twist
            self._final_stop_twists.append((float(t.linear.x), float(t.linear.y), float(t.angular.z)))

    def _filtered_cb(self, msg: Odometry) -> None:
        self._last_filtered = msg

    def _watchdog_cb(self, msg: Bool) -> None:
        self._watchdog_locked = bool(msg.data)
        if self._watchdog_locked:
            self._abort_reason = "watchdog lock is true"

    def run_loop_once(self, timeout_sec: float = 0.0) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def has_abort(self) -> bool:
        return bool(self._abort_reason)

    def abort_reason(self) -> str:
        return self._abort_reason

    def clear_abort(self) -> None:
        self._abort_reason = ""

    def publish_twist(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def send_stop(self, hold_sec: float = 0.8) -> None:
        end = time.monotonic() + max(0.0, hold_sec)
        while time.monotonic() < end:
            self.publish_twist(0.0, 0.0, 0.0)
            self.run_loop_once(timeout_sec=0.02)

    def wait_for_topics(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.run_loop_once(timeout_sec=0.1)
            if self._last_odom is not None:
                return True
        return False

    def wait_for_base_driver_node(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for name, namespace in self.get_node_names_and_namespaces():
                full = f"{namespace.rstrip('/')}/{name}".replace("//", "/")
                if full == self.args.base_driver_node:
                    return True
                if self.args.base_driver_node.startswith("/") and f"/{name}" == self.args.base_driver_node:
                    return True
                if not self.args.base_driver_node.startswith("/") and name == self.args.base_driver_node:
                    return True
            if self._last_odom is not None:
                # If odom already streams, the node is effectively alive; param set can still fail explicitly later.
                return True
            self.run_loop_once(timeout_sec=0.1)
        return False

    def set_base_driver_params(self, params: Dict[str, object], timeout_sec: float = 5.0) -> None:
        _ = timeout_sec
        for name, value in params.items():
            if isinstance(value, bool):
                value_str = "true" if value else "false"
            elif isinstance(value, float):
                value_str = f"{value:.10g}"
            else:
                value_str = str(value)

            cmd = ["ros2", "param", "set", self.args.base_driver_node, name, value_str]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            out = (proc.stdout or "") + (proc.stderr or "")
            if proc.returncode != 0 or "Set parameter successful" not in out:
                raise RuntimeError(f"set param failed: {name}={value_str} | {out.strip()}")

    def start_run_reference(self) -> None:
        if self._last_odom is None:
            raise RuntimeError("no odom received")

        self._run_start_odom = Pose2D(
            x=float(self._last_odom.pose.pose.position.x),
            y=float(self._last_odom.pose.pose.position.y),
            yaw=float(quat_to_yaw(self._last_odom.pose.pose.orientation)),
        )
        if self._last_filtered is not None:
            self._run_start_filtered = Pose2D(
                x=float(self._last_filtered.pose.pose.position.x),
                y=float(self._last_filtered.pose.pose.position.y),
                yaw=float(quat_to_yaw(self._last_filtered.pose.pose.orientation)),
            )
        else:
            self._run_start_filtered = None

        self._max_abs_dx = 0.0
        self._max_abs_dy = 0.0
        self._final_stop_twists.clear()
        self.clear_abort()

    def finalize_result(self, label: str, bag_path: Optional[str]) -> RunResult:
        odom_end_dx = None
        odom_end_dy = None
        odom_end_dyaw_deg = None
        filtered_end_dx = None
        filtered_end_dy = None
        filtered_end_dyaw_deg = None

        if self._run_start_odom is not None and self._last_odom is not None:
            end = Pose2D(
                x=float(self._last_odom.pose.pose.position.x),
                y=float(self._last_odom.pose.pose.position.y),
                yaw=float(quat_to_yaw(self._last_odom.pose.pose.orientation)),
            )
            odom_end_dx = end.x - self._run_start_odom.x
            odom_end_dy = end.y - self._run_start_odom.y
            odom_end_dyaw_deg = math.degrees(normalize_angle(end.yaw - self._run_start_odom.yaw))

        if self._run_start_filtered is not None and self._last_filtered is not None:
            endf = Pose2D(
                x=float(self._last_filtered.pose.pose.position.x),
                y=float(self._last_filtered.pose.pose.position.y),
                yaw=float(quat_to_yaw(self._last_filtered.pose.pose.orientation)),
            )
            filtered_end_dx = endf.x - self._run_start_filtered.x
            filtered_end_dy = endf.y - self._run_start_filtered.y
            filtered_end_dyaw_deg = math.degrees(normalize_angle(endf.yaw - self._run_start_filtered.yaw))

        vx_std = vy_std = wz_std = None
        if len(self._final_stop_twists) >= 3:
            vx = [v[0] for v in self._final_stop_twists]
            vy = [v[1] for v in self._final_stop_twists]
            wz = [v[2] for v in self._final_stop_twists]
            vx_std = statistics.pstdev(vx)
            vy_std = statistics.pstdev(vy)
            wz_std = statistics.pstdev(wz)

        return RunResult(
            label=label,
            aborted=self.has_abort(),
            abort_reason=self.abort_reason(),
            max_abs_dx=self._max_abs_dx,
            max_abs_dy=self._max_abs_dy,
            odom_end_dx=odom_end_dx,
            odom_end_dy=odom_end_dy,
            odom_end_dyaw_deg=odom_end_dyaw_deg,
            filtered_end_dx=filtered_end_dx,
            filtered_end_dy=filtered_end_dy,
            filtered_end_dyaw_deg=filtered_end_dyaw_deg,
            final_stop_vx_std=vx_std,
            final_stop_vy_std=vy_std,
            final_stop_wz_std=wz_std,
            bag_path=bag_path,
        )


def default_phases(args: argparse.Namespace) -> List[Phase]:
    v = float(args.v_lin)
    w = float(args.wz)
    return [
        Phase("settle", 0.0, 0.0, 0.0, 1.2),
        Phase("forward", +v, 0.0, 0.0, args.seg_t),
        Phase("backward", -v, 0.0, 0.0, args.seg_t),
        Phase("left", 0.0, +v, 0.0, args.seg_t),
        Phase("right", 0.0, -v, 0.0, args.seg_t),
        Phase("yaw_ccw", 0.0, 0.0, +w, args.turn_t),
        Phase("yaw_cw", 0.0, 0.0, -w, args.turn_t),
        Phase("diag_fwd_left", +0.8 * v, +0.8 * v, 0.0, args.diag_t),
        Phase("diag_back_right", -0.8 * v, -0.8 * v, 0.0, args.diag_t),
        Phase("final_stop", 0.0, 0.0, 0.0, args.final_stop_t),
    ]


def run_phase(node: CovarianceABRunner, phase: Phase, publish_hz: float) -> None:
    node._current_phase_name = phase.name
    dt_sec = 1.0 / max(1e-3, publish_hz)
    end_t = time.monotonic() + max(0.0, phase.duration)
    while time.monotonic() < end_t:
        if node.has_abort():
            return
        node.publish_twist(phase.vx, phase.vy, phase.wz)
        node.run_loop_once(timeout_sec=dt_sec)


def start_bag_record(bag_dir: Path, run_label: str, topics: List[str]) -> Tuple[subprocess.Popen, str]:
    bag_path = bag_dir / run_label
    cmd = ["ros2", "bag", "record", "-o", str(bag_path)] + topics
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    return proc, str(bag_path)


def stop_bag_record(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


def print_result(res: RunResult) -> None:
    print(f"\n[{res.label}]")
    print(f"  aborted         : {res.aborted}")
    if res.abort_reason:
        print(f"  abort_reason    : {res.abort_reason}")
    print(f"  max |dx|, |dy|  : {res.max_abs_dx:.3f}, {res.max_abs_dy:.3f} m")
    if res.odom_end_dx is not None:
        print(
            "  odom end        : "
            f"dx={res.odom_end_dx:+.3f} dy={res.odom_end_dy:+.3f} "
            f"dyaw={res.odom_end_dyaw_deg:+.2f} deg"
        )
    if res.filtered_end_dx is not None:
        print(
            "  filtered end    : "
            f"dx={res.filtered_end_dx:+.3f} dy={res.filtered_end_dy:+.3f} "
            f"dyaw={res.filtered_end_dyaw_deg:+.2f} deg"
        )
    if res.final_stop_vx_std is not None:
        print(
            "  final stop std  : "
            f"vx={res.final_stop_vx_std:.4f} vy={res.final_stop_vy_std:.4f} wz={res.final_stop_wz_std:.4f}"
        )
    if res.bag_path:
        print(f"  bag             : {res.bag_path}")


def summarize(results: List[RunResult]) -> None:
    if not results:
        return
    print("\n=== Summary ===")
    for res in results:
        print_result(res)

    grouped: Dict[str, List[RunResult]] = {"static": [], "dynamic": []}
    for r in results:
        key = "dynamic" if "dynamic" in r.label else "static"
        grouped.setdefault(key, []).append(r)

    print("\n=== Aggregate (non-aborted only) ===")
    for key in ("static", "dynamic"):
        vals = [r for r in grouped.get(key, []) if not r.aborted and r.filtered_end_dx is not None]
        if not vals:
            print(f"  {key:7s}: no valid run")
            continue
        pos_errors = [
            math.hypot(float(r.filtered_end_dx), float(r.filtered_end_dy))
            for r in vals
            if r.filtered_end_dx is not None and r.filtered_end_dy is not None
        ]
        yaw_errors = [abs(float(r.filtered_end_dyaw_deg)) for r in vals if r.filtered_end_dyaw_deg is not None]
        stop_wz_std = [float(r.final_stop_wz_std) for r in vals if r.final_stop_wz_std is not None]
        mean_pos = sum(pos_errors) / len(pos_errors) if pos_errors else float("nan")
        mean_yaw = sum(yaw_errors) / len(yaw_errors) if yaw_errors else float("nan")
        mean_wz = sum(stop_wz_std) / len(stop_wz_std) if stop_wz_std else float("nan")
        print(f"  {key:7s}: mean filtered_end_pos_err={mean_pos:.4f} m, mean |dyaw|={mean_yaw:.3f} deg, mean stop_wz_std={mean_wz:.5f}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-robot A/B test for odom covariance configs.")
    p.add_argument("--base-driver-node", default="/base_driver", help="Base driver node name for param setting.")
    p.add_argument("--cmd-topic", default="/cmd_vel", help="Command topic into twist_mux.")
    p.add_argument("--odom-topic", default="/odom", help="Odom topic for boundary guard.")
    p.add_argument("--filtered-topic", default="/odometry/filtered", help="Filtered odom for evaluation.")
    p.add_argument("--watchdog-lock-topic", default="/watchdog/lock", help="Watchdog lock topic (Bool).")
    p.add_argument("--publish-hz", type=float, default=30.0, help="Command publish rate.")
    p.add_argument("--guard-abs-xy", type=float, default=0.75, help="Abort if |x-x0| or |y-y0| exceeds this value (m).")
    p.add_argument("--startup-timeout", type=float, default=8.0, help="Wait time for odom and services (s).")
    p.add_argument("--inter-run-wait", type=float, default=2.0, help="Stop hold between runs (s).")
    p.add_argument("--cycles", type=int, default=2, help="How many static+dynamic cycles.")

    p.add_argument("--v-lin", type=float, default=0.16, help="Linear speed for short segments (m/s).")
    p.add_argument("--wz", type=float, default=0.45, help="Angular speed for short turns (rad/s).")
    p.add_argument("--seg-t", type=float, default=1.7, help="Duration for x/y translation phases (s).")
    p.add_argument("--turn-t", type=float, default=1.4, help="Duration for yaw phases (s).")
    p.add_argument("--diag-t", type=float, default=1.4, help="Duration for diagonal phases (s).")
    p.add_argument("--final-stop-t", type=float, default=2.5, help="Final stop phase duration (s).")

    p.add_argument("--dyn-linear-gain", type=float, default=1.5, help="dynamic_covariance_linear_gain")
    p.add_argument("--dyn-angular-gain", type=float, default=0.8, help="dynamic_covariance_angular_gain")
    p.add_argument("--dyn-scale-min", type=float, default=1.0, help="dynamic_covariance_scale_min")
    p.add_argument("--dyn-scale-max", type=float, default=12.0, help="dynamic_covariance_scale_max")

    p.add_argument("--record-bag", action="store_true", help="Record rosbag for each run.")
    p.add_argument("--bag-dir", default="bags/covariance_ab_real", help="Bag output directory.")
    p.add_argument(
        "--bag-topics",
        default="/odom /odometry/filtered /imu /cmd_vel /cmd_vel_safe /battery_voltage /watchdog/lock",
        help="Space separated topic list for bag recording.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.cycles <= 0:
        print("cycles must be > 0", file=sys.stderr)
        return 2
    if args.guard_abs_xy <= 0.0:
        print("guard-abs-xy must be > 0", file=sys.stderr)
        return 2

    bag_dir = Path(args.bag_dir).expanduser()
    if args.record_bag:
        bag_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CovarianceABRunner(args)
    phases = default_phases(args)
    results: List[RunResult] = []

    try:
        if not node.wait_for_topics(args.startup_timeout):
            raise RuntimeError(f"timeout waiting odom on {args.odom_topic}")
        if not node.wait_for_base_driver_node(args.startup_timeout):
            raise RuntimeError(f"timeout waiting base driver node {args.base_driver_node}")

        node.get_logger().info("warmup stop for 1.0s")
        node.send_stop(hold_sec=1.0)

        run_specs = []
        for idx in range(args.cycles):
            run_specs.append(("static", idx + 1))
            run_specs.append(("dynamic", idx + 1))

        for mode, cycle_id in run_specs:
            run_label = f"{mode}_c{cycle_id}_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
            node.get_logger().info("==== run %s start ====", run_label)

            if mode == "static":
                node.set_base_driver_params(
                    {
                        "dynamic_covariance_enable": False,
                    }
                )
            else:
                node.set_base_driver_params(
                    {
                        "dynamic_covariance_enable": True,
                        "dynamic_covariance_linear_gain": args.dyn_linear_gain,
                        "dynamic_covariance_angular_gain": args.dyn_angular_gain,
                        "dynamic_covariance_scale_min": args.dyn_scale_min,
                        "dynamic_covariance_scale_max": args.dyn_scale_max,
                    }
                )

            node.start_run_reference()
            bag_proc: Optional[subprocess.Popen] = None
            bag_path: Optional[str] = None
            if args.record_bag:
                topics = [t.strip() for t in args.bag_topics.split() if t.strip()]
                bag_proc, bag_path = start_bag_record(bag_dir, run_label, topics)
                time.sleep(0.4)

            for ph in phases:
                if node.has_abort():
                    break
                run_phase(node, ph, args.publish_hz)

            node.send_stop(hold_sec=max(0.6, args.inter_run_wait))
            stop_bag_record(bag_proc)

            result = node.finalize_result(run_label, bag_path)
            results.append(result)
            print_result(result)

            if result.aborted:
                node.get_logger().error("run aborted, stop remaining runs")
                break

        node.send_stop(hold_sec=1.0)
        summarize(results)
        return 0

    except KeyboardInterrupt:
        node.send_stop(hold_sec=1.0)
        print("\nInterrupted by user.")
        return 130
    except Exception as exc:
        try:
            node.send_stop(hold_sec=1.0)
        except Exception:
            pass
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
