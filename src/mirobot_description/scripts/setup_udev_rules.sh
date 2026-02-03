#!/bin/bash
# 一次性安装 udev 规则，之后无需再运行 sudo chmod 777 /dev/ttyUSB0
# 用法: sudo ./setup_udev_rules.sh  或  bash setup_udev_rules.sh (会请求 sudo)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 源码: scripts/ 与 config/ 同级；安装后: lib/mirobot_description/ 与 share/mirobot_description/ 同级
RULES_NAME="99-mirobot-usb.rules"
if [ -f "$SCRIPT_DIR/../config/$RULES_NAME" ]; then
  RULES_SRC="$SCRIPT_DIR/../config/$RULES_NAME"
elif [ -f "$SCRIPT_DIR/../share/mirobot_description/config/$RULES_NAME" ]; then
  RULES_SRC="$SCRIPT_DIR/../share/mirobot_description/config/$RULES_NAME"
elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  PKG_SHARE="$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -1)"
  if [ -f "$PKG_SHARE/share/mirobot_description/config/$RULES_NAME" ]; then
    RULES_SRC="$PKG_SHARE/share/mirobot_description/config/$RULES_NAME"
  else
    echo "未找到 $RULES_NAME"
    exit 1
  fi
else
  echo "未找到 $RULES_NAME，请先 source install/setup.sh 或从 mirobot_description 源码/install 下执行"
  exit 1
fi

echo "安装 udev 规则: $RULES_SRC -> /etc/udev/rules.d/"
sudo cp "$RULES_SRC" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "完成。请重新插拔机械臂 USB，或下次启动即可直接使用 /dev/ttyUSB0。"
