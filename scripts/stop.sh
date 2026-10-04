#!/usr/bin/env bash
set -euo pipefail
NAME="${NAME:-duojin01_hardware}"
docker rm -f "$NAME" 2>/dev/null || true
echo "Stopped & removed: $NAME"
