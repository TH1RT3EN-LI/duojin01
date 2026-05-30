#!/usr/bin/env bash
set -euo pipefail

# Stop common ROS 1/2 processes and clean up common DDS shared-memory artifacts.

SELF_PID="$$"
PARENT_PID="${PPID:-0}"

PROCESS_PATTERNS=(
  "roscore"
  "rosmaster"
  "roslaunch"
  "rosout"
  "_ros2_daemon"
  "ros2 launch"
  "ros2 run"
  "ros2 bag"
  "ros2 topic"
  "ros2 service"
  "ros2 action"
  "ros2 param"
  "ros2 node"
  "ros2 lifecycle"
  "--ros-args"
  "__name:="
  "__ns:="
  "__log:="
  "component_container"
  "component_container_mt"
  "component_container_isolated"
  "rviz2"
  "rviz "
  "rqt"
  "foxglove_bridge"
  "ros_gz_bridge"
  "ros_gz_sim"
  "gz sim"
  "gzserver"
  "gzclient"
  "ign gazebo"
  "ignition gazebo"
  "iox-roudi"
)

DDS_GLOBS=(
  "/dev/shm/fastrtps_*"
  "/dev/shm/fastdds_*"
  "/dev/shm/sem.fastrtps_*"
  "/dev/shm/sem.fastdds_*"
  "/dev/shm/cyclonedds*"
  "/dev/shm/sem.cyclonedds*"
  "/dev/shm/iox_*"
  "/dev/shm/sem.iox_*"
)

collect_matching_pids() {
  local pattern pid
  local -a pids=()

  for pattern in "${PROCESS_PATTERNS[@]}"; do
    while IFS= read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "${SELF_PID}" ]] && continue
      [[ "${pid}" == "${PARENT_PID}" ]] && continue
      pids+=("${pid}")
    done < <(pgrep -f -- "${pattern}" || true)
  done

  if ((${#pids[@]} > 0)); then
    printf '%s\n' "${pids[@]}" | sort -u
  fi
}

kill_matching_processes() {
  local signal="$1"
  local -a pids=()

  mapfile -t pids < <(collect_matching_pids)
  if ((${#pids[@]} == 0)); then
    return 1
  fi

  echo "Sending SIG${signal} to PIDs: ${pids[*]}"
  kill "-${signal}" "${pids[@]}" 2>/dev/null || true
  return 0
}

cleanup_dds_artifacts() {
  local glob
  local -a matches=()
  local removed_any=0

  shopt -s nullglob
  for glob in "${DDS_GLOBS[@]}"; do
    matches=(${glob})
    if ((${#matches[@]} == 0)); then
      continue
    fi
    rm -rf -- "${matches[@]}" 2>/dev/null || true
    echo "Removed DDS artifacts: ${matches[*]}"
    removed_any=1
  done
  shopt -u nullglob

  if ((removed_any == 0)); then
    echo "No DDS shared-memory artifacts found."
  fi
}

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

if kill_matching_processes TERM; then
  sleep 2
  kill_matching_processes KILL || true
else
  echo "No matching ROS processes found."
fi

cleanup_dds_artifacts
echo "ROS process shutdown and DDS cleanup complete."
