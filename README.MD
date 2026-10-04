# Duojin01 真机工作区

`hardware-only` 分支面向 **Ubuntu 22.04（Jammy）+ ROS 2 Humble**。保留麦克纳姆底盘串口驱动、雷达、USB/Orbbec 相机、安全看门狗、建图、Nav2 导航，以及机械臂串口 G-code 和 AprilTag 任务工具。

## Docker：构建和运行

在仓库根目录执行：

```bash
./scripts/build_image.sh
```

生成 `duojin01:hardware-humble`。镜像从 `ros:humble-ros-base-jammy` 构建，包含已编译的全部 14 个真机 ROS 包；进入容器后即可使用 ROS 命令。镜像安装所需的 ROS 和硬件依赖，不添加 OSRF 软件源。

构建脚本会传递当前终端的代理变量；Linux 上的本地回环代理会自动使用构建主机网络，也可用 `DOCKER_BUILD_NETWORK` 指定网络。基础镜像拉取由 Docker 服务负责，如这一阶段超时，应配置 Docker 服务的网络代理，或先通过 `docker load` 导入官方基础镜像。代理配置不会写入生成镜像的运行环境。

按真机实际端口映射设备；下面假设底盘和雷达已创建稳定设备名，机械臂在 `/dev/ttyUSB0`，USB 相机在 `/dev/video0`：

```bash
DEV_LIST=/dev/duojin01_controller,/dev/lslidar,/dev/ttyUSB0,/dev/video0 \
  ./scripts/dev.sh
./scripts/enter.sh
```

容器名默认 `duojin01_hardware`，工作区挂载到 `/ws`，地图保存到宿主机仓库的 `maps/`。启动容器只打开开发环境，不会自动启动机器人节点或发送运动命令。

需要 Orbbec 深度相机时，创建容器时增加 `USB_BUS=1`；需要本机 RViz 窗口时增加 `GUI=1 GPU=1`。`DEV_LIST` 支持逗号分隔的设备列表，`DEV` 支持单设备，`DEV_GLOB` 支持设备通配符。明确指定的设备不存在时会报错，需按实际连接情况调整列表。

```bash
# 示例：启用 USB 总线和本机图形界面
USB_BUS=1 GUI=1 GPU=1 \
  DEV_LIST=/dev/duojin01_controller,/dev/lslidar,/dev/ttyUSB0,/dev/video0 \
  ./scripts/dev.sh

# 停止并删除本分支的开发容器
./scripts/stop.sh

# 转移镜像到另一台真机
# docker save -o duojin01-hardware-humble.tar duojin01:hardware-humble
# docker load -i duojin01-hardware-humble.tar
```

修改设备映射后应先停止旧容器，再重新创建。`IMAGE` 和 `NAME` 可以覆盖默认镜像和容器名。默认 `ROS_DOMAIN_ID=20`，参与同一真机系统的所有终端应使用相同值；可在创建容器前设置其他值。

镜像中的工作区位于 `/opt/duojin01/install`。如需修改源码并在开发容器中重编译：

```bash
./scripts/build_hardware.sh
source ./source.sh
```

开发构建使用独立的 `build/hardware`、`install/hardware`、`log/hardware`，仅扫描 `src/`。再次进入容器会优先加载此硬件构建覆盖层。

## Ubuntu 22.04 原生构建

先安装 ROS 2 Humble，然后运行：

```bash
./setup_ubuntu22.sh
./scripts/build_hardware.sh
source ./source.sh
```

ROS 安装说明：[官方 Ubuntu Debian 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。

串口设备需要当前用户具备 `dialout` 组权限。稳定设备名可通过 `scripts/set_dev.sh` 为实际设备建立，例如：

```bash
sudo ./scripts/set_dev.sh /dev/ttyACM0 duojin01_controller Controller
sudo ./scripts/set_dev.sh /dev/ttyACM1 lslidar Lslidar
```

## 真机启动入口

```bash
# 底盘联调：保留 twist_mux 和安全看门狗
ros2 launch duojin01_bringup base.launch.py use_lidar:=false

# 建图
ros2 launch duojin01_bringup mapping.launch.py

# 保存地图到工作区 maps/，需要建图节点已启动
ros2 launch duojin01_bringup save_map.launch.py

# 导航，默认从 maps/ 选择最大的数字编号地图
ros2 launch duojin01_bringup navigation.launch.py
# 或显式指定地图；相对路径从工作区根目录解析
ros2 launch duojin01_bringup navigation.launch.py map:=maps/0.yaml

# 独立 USB 相机
ros2 launch duojin01_bringup camera.launch.py device_id:=0
# 使用真实相机标定文件
ros2 launch duojin01_bringup camera.launch.py \
  device_id:=0 calibration_file:=/ws/calibration.yaml

# 机械臂串口任务 API；此入口不会自动执行任务序列
ros2 launch duojin01_bringup mission.launch.py \
  serial_port:=/dev/ttyUSB0 serial_baudrate:=115200

# 键盘控制
ros2 run duojin01_teleop keyboard_teleop_node
```

所有启动入口固定使用系统时钟。导航启动后需在 RViz 中用 **2D Pose Estimate** 指定真实初始位置，然后发送目标点。地图不是随镜像附带的预设场景，需要从真实环境采集或指定已有地图。

常用启动参数：

| 参数 | 含义 |
| --- | --- |
| `base_serial_port` | 底盘串口，默认 `/dev/duojin01_controller` |
| `base_serial_baudrate` | 底盘波特率，默认 `115200` |
| `use_teleop` | 手柄链路开关，与雷达开关独立 |
| `use_foxglove` | Foxglove Bridge 开关 |
| `use_rviz` | 建图/导航的 RViz 开关 |
| `use_depth_camera` | 导航中按需启动 Orbbec 硬件相机，默认 `false` |
| `depth_camera_launch_file` | 默认 `gemini_330_series.launch.py`，应与实际型号一致 |
| `map` / `params_file` | 导航地图及 Nav2 参数文件 |

无显示器时可设置 `use_rviz:=false`；不使用 Foxglove 时设置 `use_foxglove:=false`。可通过 `DUOJIN01_WORKSPACE_ROOT` 指定数据工作区的绝对路径，导航选图和保存地图使用相同根目录。

## 验证

在硬件 Docker 镜像或已构建的原生环境执行：

```bash
./scripts/validate_hardware.sh
```

验证包依赖边界、真实时钟配置、URDF/网格完整性、可选相机依赖、地图路径以及全部真机启动入口的参数解析；不会启动硬件节点。真实串口、相机、雷达与机械臂动作仍需连接设备进行联调。

本分支已在 `linux/amd64` 的 Ubuntu 22.04 Docker 镜像中完成全部 14 个包的编译，24 项测试和 9 个启动入口解析通过。相机 NumPy/OpenCV/AprilTag 兼容性、驱动动态库、容器重新进入和宿主机地图持久化也已验证。

接口和任务工具见 [README_DEVELOPMENT.md](README_DEVELOPMENT.md)。自研 `duojin01_*` 包使用 GPL-3.0-only；第三方驱动遵循各自目录的许可证。
