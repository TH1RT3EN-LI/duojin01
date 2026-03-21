#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_NAME="duojin01_bringup"

INSTALL_DIR="$WS_DIR/install/$PKG_NAME/share/$PKG_NAME/maps"
SRC_DIR="$WS_DIR/src/$PKG_NAME/maps"
NUMERIC_ONLY=1
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage:
  scripts/sync_saved_maps.sh [options]

Options:
  --install-dir <path>   Source map directory from install.
  --src-dir <path>       Destination map directory in src.
  --all-names            Sync all map names (default only numeric names).
  --dry-run              Show what would be copied.
  -h, --help             Show this help.

Defaults:
  install-dir: install/duojin01_bringup/share/duojin01_bringup/maps
  src-dir:     src/duojin01_bringup/maps
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-dir)
      INSTALL_DIR="${2:-}"
      shift 2
      ;;
    --src-dir)
      SRC_DIR="${2:-}"
      shift 2
      ;;
    --all-names)
      NUMERIC_ONLY=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[sync_saved_maps] Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$INSTALL_DIR" || -z "$SRC_DIR" ]]; then
  echo "[sync_saved_maps] install-dir/src-dir must not be empty." >&2
  exit 1
fi

if [[ ! -d "$INSTALL_DIR" ]]; then
  echo "[sync_saved_maps] Install map dir not found: $INSTALL_DIR" >&2
  exit 1
fi

mkdir -p "$SRC_DIR"

copied=0
skipped=0
matched=0

shopt -s nullglob
for yaml in "$INSTALL_DIR"/*.yaml; do
  stem="$(basename "${yaml%.yaml}")"

  if [[ "$NUMERIC_ONLY" -eq 1 ]] && [[ ! "$stem" =~ ^[0-9]+$ ]]; then
    continue
  fi

  ((matched+=1))

  for ext in yaml pgm png; do
    src_file="$INSTALL_DIR/$stem.$ext"
    dst_file="$SRC_DIR/$stem.$ext"

    if [[ ! -e "$src_file" ]]; then
      continue
    fi

    if [[ -e "$dst_file" ]] && cmp -s "$src_file" "$dst_file"; then
      ((skipped+=1))
      continue
    fi

    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "[dry-run] $src_file -> $dst_file"
    else
      cp -Lf "$src_file" "$dst_file"
      echo "[copied] $src_file -> $dst_file"
    fi
    ((copied+=1))
  done
done
shopt -u nullglob

if [[ "$matched" -eq 0 ]]; then
  if [[ "$NUMERIC_ONLY" -eq 1 ]]; then
    echo "[sync_saved_maps] No numeric map yaml found in: $INSTALL_DIR"
  else
    echo "[sync_saved_maps] No map yaml found in: $INSTALL_DIR"
  fi
  exit 0
fi

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "[sync_saved_maps] dry-run done. matched=$matched, pending_copy=$copied, skipped_same=$skipped"
else
  echo "[sync_saved_maps] done. matched=$matched, copied=$copied, skipped_same=$skipped"
fi
