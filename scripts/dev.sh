#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-duojin01:humble-harmonic}"
NAME="${NAME:-duojin01_dev}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

GUI="${GUI:-1}"
GUI_BACKEND="${GUI_BACKEND:-auto}"  # auto | wayland | x11
NET_HOST="${NET_HOST:-1}"
DETACH="${DETACH:-1}"
IPC_HOST="${IPC_HOST:-1}"
SHM_SIZE="${SHM_SIZE:-}"

GPU="${GPU:-1}"
INPUT="${INPUT:-1}"
USB_BUS="${USB_BUS:-0}"
DEV="${DEV:-}"
DEV_LIST="${DEV_LIST:-}"
DEV_GLOB="${DEV_GLOB:-}"

DOCKER_DEV_ARGS=()
DOCKER_GROUP_ARGS=()
DOCKER_ENV_ARGS=()
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}"
HAVE_X11=0
HAVE_WAYLAND=0
GUI_BACKEND_SELECTED=""

if [[ -n "${DISPLAY:-}" ]] && [[ -d /tmp/.X11-unix ]]; then
  HAVE_X11=1
fi

if [[ -n "${WAYLAND_DISPLAY:-}" ]] && [[ -n "$HOST_XDG_RUNTIME_DIR" ]] && [[ -S "${HOST_XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]]; then
  HAVE_WAYLAND=1
fi

if [[ "$GUI" == "1" ]]; then
  case "$GUI_BACKEND" in
    auto)
      # Prefer X11/XWayland first: Gazebo GUI (Qt5) is typically more stable
      # there than native Wayland inside containers.
      if [[ "$HAVE_X11" == "1" ]]; then
        GUI_BACKEND_SELECTED="x11"
      elif [[ "$HAVE_WAYLAND" == "1" ]]; then
        GUI_BACKEND_SELECTED="wayland"
      fi
      ;;
    wayland)
      if [[ "$HAVE_WAYLAND" == "1" ]]; then
        GUI_BACKEND_SELECTED="wayland"
      elif [[ "$HAVE_X11" == "1" ]]; then
        echo "Requested Wayland GUI, but Wayland socket is unavailable. Falling back to X11." >&2
        GUI_BACKEND_SELECTED="x11"
      fi
      ;;
    x11)
      if [[ "$HAVE_X11" == "1" ]]; then
        GUI_BACKEND_SELECTED="x11"
      elif [[ "$HAVE_WAYLAND" == "1" ]]; then
        echo "Requested X11 GUI, but DISPLAY/X11 socket is unavailable. Falling back to Wayland." >&2
        GUI_BACKEND_SELECTED="wayland"
      fi
      ;;
    *)
      echo "Invalid GUI_BACKEND='$GUI_BACKEND' (expected: auto, wayland, x11)." >&2
      exit 1
      ;;
  esac

  if [[ -z "$GUI_BACKEND_SELECTED" ]]; then
    echo "No usable display socket found (DISPLAY/WAYLAND_DISPLAY). Disable GUI and run headless." >&2
    GUI="0"
  else
    echo "GUI backend: ${GUI_BACKEND_SELECTED}" >&2
  fi
fi


if [[ "$INPUT" == "1" ]] && [[ -d /dev/input ]]; then
  DOCKER_DEV_ARGS+=(--device=/dev/input)
fi

if [[ "$GPU" == "1" ]]; then
  if [[ -d /dev/dri ]]; then
    DOCKER_DEV_ARGS+=(--device=/dev/dri)

    for grp in render video; do
      gid="$(getent group "$grp" | cut -d: -f3 || true)"
      [[ -n "$gid" ]] && DOCKER_GROUP_ARGS+=(--group-add "$gid")
    done

    # Avoid frequent AMD/Mesa DRM fd warnings in containerized GUI apps.
    if [[ "${AMD_DRI3_FIX:-1}" == "1" ]]; then
      DOCKER_ENV_ARGS+=(-e LIBGL_DRI3_DISABLE=1)
    fi
  else
    echo "GPU=1 but /dev/dri not found on host, falling back to software rendering." >&2
  fi
fi

if [[ "$USB_BUS" == "1" ]] && [[ -d /dev/bus/usb ]]; then
  DOCKER_DEV_ARGS+=(-v /dev/bus/usb:/dev/bus/usb)
fi

if [[ -n "$DEV_LIST" ]]; then
  IFS=',' read -r -a _devs <<< "$DEV_LIST"
  for d in "${_devs[@]}"; do
    d="$(echo "$d" | xargs)"
    [[ -z "$d" ]] && continue
    [[ -e "$d" ]] && DOCKER_DEV_ARGS+=(--device="$d")
  done
fi

if [[ -n "$DEV_GLOB" ]]; then
  IFS=',' read -r -a _globs <<< "$DEV_GLOB"
  for g in "${_globs[@]}"; do
    g="$(echo "$g" | xargs)"
    [[ -z "$g" ]] && continue
    shopt -s nullglob
    matches=( $g )
    shopt -u nullglob
    for m in "${matches[@]}"; do
      [[ -e "$m" ]] && DOCKER_DEV_ARGS+=(--device="$m")
    done
  done
fi

if [[ -n "$DEV" ]] && [[ -e "$DEV" ]]; then
  DOCKER_DEV_ARGS+=(--device="$DEV")
fi

args=(
  --name "$NAME"
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp
  -v "$WS_DIR":/ws
  -w /ws
  -v /etc/passwd:/etc/passwd:ro
  -v /etc/group:/etc/group:ro
)

args+=( "${DOCKER_DEV_ARGS[@]}" )
args+=( "${DOCKER_GROUP_ARGS[@]}" )
args+=( "${DOCKER_ENV_ARGS[@]}" )

if [[ "$NET_HOST" == "1" ]]; then
  args+=(--net=host)
fi

if [[ "$IPC_HOST" == "1" ]]; then
  args+=(--ipc=host)
elif [[ -n "$SHM_SIZE" ]]; then
  args+=(--shm-size="$SHM_SIZE")
fi

if [[ "$GUI" == "1" ]]; then
  if [[ "$GUI_BACKEND_SELECTED" == "x11" ]]; then
    xhost +si:localuser:"$USER" >/dev/null 2>&1 || true
    args+=(-e QT_X11_NO_MITSHM=1)
    args+=(-e QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}")
    args+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    args+=(-e XDG_RUNTIME_DIR=/tmp/runtime-root)
  fi

  if [[ "$GUI_BACKEND_SELECTED" == "wayland" ]]; then
    WAYLAND_SOFTWARE_GL="${WAYLAND_SOFTWARE_GL:-0}"

    args+=(-e WAYLAND_DISPLAY="$WAYLAND_DISPLAY")
    args+=(-e XDG_RUNTIME_DIR="$HOST_XDG_RUNTIME_DIR" -v "$HOST_XDG_RUNTIME_DIR:$HOST_XDG_RUNTIME_DIR:rw")
    args+=(-e QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland;xcb}")

    # Keep X11 fallback available for tools that only support xcb.
    if [[ "$HAVE_X11" == "1" ]]; then
      xhost +si:localuser:"$USER" >/dev/null 2>&1 || true
      args+=(-e QT_X11_NO_MITSHM=1)
      args+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    fi

    # More stable defaults for Qt Quick Gazebo GUI under Wayland containers.
    if [[ "$WAYLAND_SOFTWARE_GL" == "1" ]]; then
      args+=(-e LIBGL_ALWAYS_SOFTWARE=1)
      args+=(-e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe)
      args+=(-e QT_OPENGL=software)
      args+=(-e QT_QUICK_BACKEND=software)
      args+=(-e QT_XCB_GL_INTEGRATION=none)
    fi
  fi

  if [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]]; then
    args+=(-e DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS")
    [[ -n "$HOST_XDG_RUNTIME_DIR" ]] && [[ -S "${HOST_XDG_RUNTIME_DIR}/bus" ]] && args+=(-v "${HOST_XDG_RUNTIME_DIR}/bus:${HOST_XDG_RUNTIME_DIR}/bus")
  fi
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "Container '$NAME' already exists."
  echo "Use: ./scripts/enter.sh"
  echo "Or : docker rm -f $NAME"
  exit 0
fi

if [[ "$DETACH" == "1" ]]; then
  exec docker run -d "${args[@]}" "$IMAGE" tail -f /dev/null
else
  exec docker run --rm -it "${args[@]}" "$IMAGE" bash
fi
