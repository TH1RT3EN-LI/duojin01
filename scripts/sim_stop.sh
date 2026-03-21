#!/usr/bin/env bash
set -euo pipefail

# Stop all simulation-related processes that may survive after Ctrl+Z / forced kill.
# This prevents multiple /clock publishers causing "Detected jump back in time" spam.

patterns=(
  "ros2 launch duojin01_bringup sim.launch.py"
  "ros2 launch duojin01_bringup sim_navigation.launch.py"
  "ros2 launch duojin01_bringup sim_mapping.launch.py"
  "gz sim"
  "gz -g"
  "ros_gz_sim/create"
  "ros_gz_bridge/parameter_bridge"
  "robot_state_publisher/robot_state_publisher"
  "robot_localization/ekf_node"
  "twist_mux/twist_mux"
  "duojin01_sim_tools/clock_guard_node"
  "duojin01_sim_tools/joint_state_stamp_fix_node"
  "duojin01_controller_emulator_node"
  "duojin01_base_driver_node"
  "duojin01_safety_watchdog/safety_watchdog_node"
  "joy_node"
  "teleop_node"
)

kill_by_patterns() {
  local sig="$1"
  local killed=0
  local self_pid="$$"
  local parent_pid="${PPID:-0}"
  for pat in "${patterns[@]}"; do
    while IFS= read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "${self_pid}" ]] && continue
      [[ "${pid}" == "${parent_pid}" ]] && continue
      kill "-${sig}" "${pid}" 2>/dev/null || true
      killed=1
    done < <(pgrep -f "${pat}" || true)
  done
  return "${killed}"
}

kill_by_patterns TERM || true
sleep 1
kill_by_patterns KILL || true

# Clean up stale FastDDS shared-memory locks that may break next launch.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo "Simulation processes stopped and FastDDS SHM locks cleaned."
