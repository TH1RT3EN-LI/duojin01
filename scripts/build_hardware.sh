#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$hardware_root"
exec colcon --log-base log/hardware build --base-paths src \
  --build-base build/hardware --install-base install/hardware \
  --symlink-install --parallel-workers "${BUILD_WORKERS:-2}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
