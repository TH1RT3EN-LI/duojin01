#!/usr/bin/env python3
"""
Launch 启动时调用：若串口无写权限则尝试 chmod（需 pkexec），否则提示安装 udev 或加入 dialout 组。
从 config/rviz_params.yaml 读取 port_name，若无则用 /dev/ttyUSB0。
"""

import os
import subprocess
import sys


def main():
    port = "/dev/ttyUSB0"
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("mirobot_description")
        yaml_path = os.path.join(pkg_share, "config", "rviz_params.yaml")
        if os.path.isfile(yaml_path):
            with open(yaml_path) as f:
                for line in f:
                    if "port_name" in line and ":" in line:
                        port = line.split(":", 1)[1].strip().strip('"').strip("'")
                        break
    except Exception:
        pass

    if not os.path.exists(port):
        print(f"[ensure_serial_permissions] 端口 {port} 不存在，请连接机械臂后再 launch。", file=sys.stderr)
        return 0
    if os.access(port, os.R_OK | os.W_OK):
        return 0

    # 尝试用 pkexec chmod（会弹窗要密码）
    try:
        r = subprocess.run(
            ["pkexec", "chmod", "666", port],
            capture_output=True,
            text=True,
            timeout=30,
        )
        if r.returncode == 0:
            print(f"[ensure_serial_permissions] 已设置 {port} 权限。")
            return 0
    except FileNotFoundError:
        pass
    except subprocess.TimeoutExpired:
        print("[ensure_serial_permissions] 权限请求超时。", file=sys.stderr)

    print(
        f"[ensure_serial_permissions] 无法写入 {port}。请任选其一：\n"
        "  1) 一次性安装 udev 规则: sudo bash $(ros2 pkg prefix mirobot_description)/lib/mirobot_description/setup_udev_rules.sh\n"
        "  2) 或将当前用户加入 dialout 组: sudo usermod -aG dialout $USER  然后注销再登录",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
