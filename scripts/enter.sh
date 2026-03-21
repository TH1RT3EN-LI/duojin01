#!/usr/bin/env bash
set -euo pipefail

NAME="${NAME:-duojin01_dev}"
ROOT="${ROOT:-1}"
DOCKER_EXEC_USER="${DOCKER_EXEC_USER:-$(id -u):$(id -g)}"
EXEC_HOME="/tmp"

if [[ "$ROOT" == "1" ]]; then
  DOCKER_EXEC_USER="0"
  EXEC_HOME="/root"
  if [[ -n "${DISPLAY:-}" ]]; then
    xhost +si:localuser:root >/dev/null 2>&1 || true
  fi
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  DETACH=1 ./scripts/dev.sh >/dev/null
fi


TERM_IN="${TERM:-xterm-256color}"
if [[ -n "${TMUX:-}" ]]; then
  TERM_IN="tmux-256color"
fi

exec docker exec -it \
  -u "$DOCKER_EXEC_USER" \
  -w /ws \
  -e "HOME=${EXEC_HOME}" \
  -e "TERM=${TERM_IN}" \
  -e "COLORTERM=${COLORTERM:-truecolor}" \
  ${TERMINFO:+-e "TERMINFO=${TERMINFO}"} \
  ${TERMINFO_DIRS:+-e "TERMINFO_DIRS=${TERMINFO_DIRS}"} \
  "$NAME" bash
