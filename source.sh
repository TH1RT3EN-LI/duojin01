#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source install/setup.bash

# FastDDS shared-memory transport may spam:
# "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed"
# in container + host IPC setups. Default to UDP-only for stability.
# Set DUOJIN01_FASTDDS_SHM=1 before sourcing to re-enable SHM transport.
if [[ "${DUOJIN01_FASTDDS_SHM:-0}" != "1" ]]; then
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

if [[ -z "${DUOJIN01_SECURITY_ROOT:-}" ]]; then
  export DUOJIN01_SECURITY_ROOT="${SCRIPT_DIR}/security/keystore"
fi

if [[ ! -r "${DUOJIN01_SECURITY_ROOT}/policy.version" ]]; then
  cat >&2 <<EOF
[duojin01] security keystore is missing or unreadable:
  ${DUOJIN01_SECURITY_ROOT}
Run:
  ./setup_security_keystore.sh
EOF
  return 1 2>/dev/null || exit 1
fi

# Keep the runtime domain fixed so launches do not depend on host/container
# environment drift or keystore metadata.
export ROS_DOMAIN_ID=0
