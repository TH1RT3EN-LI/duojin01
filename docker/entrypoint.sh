#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/duojin01/install/setup.bash
if [[ -r /ws/install/hardware/setup.bash ]]; then
  source /ws/install/hardware/setup.bash
fi
exec "$@"
