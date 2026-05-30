#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"
STAMP="$(date +%F-%H%M%S)"

APT_CMD=()
ROOT_CMD=()

UBUNTU_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
ROS_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
ROSDEP_INDEX_URL="https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"
PIP_INDEX_URL="https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple"
ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
OSRF_KEY_URL="https://packages.osrfoundation.org/gazebo.gpg"

log() {
  printf '\n[%s] %s\n' "$(date '+%F %T')" "$*"
}

die() {
  printf '\n[ERROR] %s\n' "$*" >&2
  exit 1
}

backup_file() {
  local path="$1"
  if [[ -f "$path" ]]; then
    "${ROOT_CMD[@]}" cp "$path" "${path}.bak.${STAMP}"
  fi
}

download_to() {
  local url="$1"
  local dest="$2"

  if command -v curl >/dev/null 2>&1; then
    curl -fsSL --retry 5 --retry-delay 2 --retry-all-errors "$url" -o "$dest"
    return
  fi

  python3 - "$url" "$dest" <<'PY'
import pathlib
import ssl
import sys
import urllib.request

url, dest = sys.argv[1:3]
ctx = ssl.create_default_context()
with urllib.request.urlopen(url, context=ctx, timeout=30) as resp:
    pathlib.Path(dest).write_bytes(resp.read())
PY
}

apt_update() {
  "${APT_CMD[@]}" -o Acquire::Retries=3 update
}

apt_install() {
  "${APT_CMD[@]}" install -y "$@"
}

run_root() {
  "${ROOT_CMD[@]}" "$@"
}

source_ros_setup() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    # ROS setup scripts may reference optional variables that are not set in a
    # strict `set -u` shell, so temporarily relax nounset while sourcing.
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

init_privilege_mode() {
  if [[ "${EUID}" -eq 0 ]]; then
    APT_CMD=(apt-get)
    ROOT_CMD=()
  else
    command -v sudo >/dev/null 2>&1 || die "当前不是 root，但系统里没有 sudo。请先安装 sudo，或者直接用 root 运行。"
    APT_CMD=(sudo apt-get)
    ROOT_CMD=(sudo)
  fi
}

ensure_ubuntu_2204() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    [[ "${ID:-}" == "ubuntu" && "${VERSION_CODENAME:-}" == "jammy" ]] || \
      die "这个脚本只面向 Ubuntu 22.04 (jammy)。当前系统是 ${ID:-unknown} ${VERSION_CODENAME:-unknown}。"
  fi
}

purge_unneeded_conflicting_packages() {
  # `v4l2loopback-dkms` is unrelated to this workspace. On some hosts it blocks
  # apt transactions by trying to rebuild against the current kernel.
  if dpkg -s v4l2loopback-dkms >/dev/null 2>&1; then
    log "移除不需要的冲突包: v4l2loopback-dkms"
    run_root dpkg --purge --force-all v4l2loopback-dkms || \
      run_root apt-get purge -y v4l2loopback-dkms || true
  fi
}

bootstrap_minimum_tools() {
  log "安装最小引导依赖"
  apt_update
  apt_install ca-certificates curl python3 lsb-release gnupg
}

configure_ubuntu_sources() {
  if [[ -f /etc/apt/sources.list.d/ubuntu.sources ]]; then
    backup_file /etc/apt/sources.list.d/ubuntu.sources
    log "写入 /etc/apt/sources.list.d/ubuntu.sources"
    run_root tee /etc/apt/sources.list.d/ubuntu.sources >/dev/null <<EOF
Types: deb
URIs: ${UBUNTU_MIRROR}
Suites: jammy jammy-updates jammy-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: http://security.ubuntu.com/ubuntu/
Suites: jammy-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
  else
    backup_file /etc/apt/sources.list
    log "写入 /etc/apt/sources.list"
    run_root tee /etc/apt/sources.list >/dev/null <<EOF
deb ${UBUNTU_MIRROR} jammy main restricted universe multiverse
deb ${UBUNTU_MIRROR} jammy-updates main restricted universe multiverse
deb ${UBUNTU_MIRROR} jammy-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse
EOF
  fi
}

configure_ros_sources() {
  run_root mkdir -p /usr/share/keyrings /etc/apt/sources.list.d

  local ros_key_tmp
  ros_key_tmp="$(mktemp)"
  download_to "$ROS_KEY_URL" "$ros_key_tmp"
  run_root install -m 0644 "$ros_key_tmp" /usr/share/keyrings/ros-archive-keyring.gpg
  rm -f "$ros_key_tmp"

  backup_file /etc/apt/sources.list.d/ros2.list
  log "写入 /etc/apt/sources.list.d/ros2.list"
  run_root tee /etc/apt/sources.list.d/ros2.list >/dev/null <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} jammy main
EOF
}

configure_rosdep_source() {
  run_root mkdir -p /etc/ros/rosdep/sources.list.d
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run_root rosdep init || true
  fi

  local rosdep_tmp
  rosdep_tmp="$(mktemp)"
  download_to "https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list" "$rosdep_tmp"
  run_root install -m 0644 "$rosdep_tmp" /etc/ros/rosdep/sources.list.d/20-default.list
  rm -f "$rosdep_tmp"

  touch "$HOME/.bashrc"
  grep -qxF "export ROSDISTRO_INDEX_URL=${ROSDEP_INDEX_URL}" "$HOME/.bashrc" || \
    echo "export ROSDISTRO_INDEX_URL=${ROSDEP_INDEX_URL}" >> "$HOME/.bashrc"
  export ROSDISTRO_INDEX_URL="${ROSDEP_INDEX_URL}"
}

install_base_tools() {
  apt_install \
    git \
    curl \
    ffmpeg \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    python3-serial \
    python3-rtree
}

install_repo_apt_deps() {
  apt_install \
    ros-humble-asio-cmake-module \
    ros-humble-laser-filters \
    ros-humble-pcl-msgs \
    ros-humble-sros2 \
    ros-humble-xacro \
    ros-humble-twist-mux \
    ros-humble-serial-driver \
    ros-humble-foxglove-bridge \
    libboost-thread-dev \
    libyaml-cpp-dev \
    libdw-dev \
    libpcl-dev
}

install_gazebo_deps() {
  apt_install curl lsb-release gnupg

  local osrf_key_tmp
  osrf_key_tmp="$(mktemp)"
  download_to "$OSRF_KEY_URL" "$osrf_key_tmp"
  run_root install -m 0644 "$osrf_key_tmp" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  rm -f "$osrf_key_tmp"

  backup_file /etc/apt/sources.list.d/gazebo-stable.list
  log "写入 /etc/apt/sources.list.d/gazebo-stable.list"
  run_root tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main
EOF

  apt_update
  apt_install gz-harmonic ros-humble-ros-gzharmonic
}

install_ros_workspace_deps() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source_ros_setup /opt/ros/humble/setup.bash
  else
    log "警告: 未检测到 /opt/ros/humble/setup.bash，这通常表示 ROS 2 Humble 本体还没装。"
  fi

  cd "$WS_DIR"

  local attempt
  for attempt in 1 2 3; do
    if rosdep update; then
      break
    fi
    [[ "$attempt" -lt 3 ]] || die "rosdep update 失败，请检查网络和镜像配置。"
    log "rosdep update 失败，正在重试 (${attempt}/3)..."
    sleep 5
  done

  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
    --skip-keys "ros_gz_sim ros_gz_bridge"
}

install_python_deps() {
  python3 -m pip config set global.index-url "$PIP_INDEX_URL"
  python3 -m pip install --user -U pip
  python3 -m pip install --user pupil-apriltags
}

verify_install() {
  command -v rosdep >/dev/null 2>&1 || die "rosdep 没有安装成功。"
  command -v gz >/dev/null 2>&1 || die "Gazebo Harmonic 没有安装成功。"
  python3 -c "import pupil_apriltags" || die "pupil-apriltags 导入失败。"
}

main() {
  init_privilege_mode
  ensure_ubuntu_2204
  log "工作区: ${WS_DIR}"
  purge_unneeded_conflicting_packages
  bootstrap_minimum_tools
  configure_ubuntu_sources
  apt_update
  configure_ros_sources
  apt_update

  install_base_tools
  install_repo_apt_deps

  configure_rosdep_source
  install_gazebo_deps
  install_ros_workspace_deps
  install_python_deps
  verify_install

  log "全部步骤已完成。"
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    log "提示: 你还需要先安装 ROS 2 Humble 本体，然后再执行 source /opt/ros/humble/setup.bash。"
  fi
}

main "$@"
