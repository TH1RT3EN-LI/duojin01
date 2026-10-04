#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/source.sh"
cd "$hardware_root"
python3 -m pytest -q tests/test_hardware_boundary.py src/duojin01_teleop/test/test_keyboard_teleop_core.py
for entry in base mapping navigation nav2 slam save_map joy_teleop camera mission; do
  ros2 launch duojin01_bringup "$entry.launch.py" --show-args >/dev/null
  echo "Launch arguments resolved: $entry"
done
