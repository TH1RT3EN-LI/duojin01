<div align="center">
  <img src="docs/images/hero.png" alt="duojin01_hero" width="600px" style="border-radius: 10px;">

  # Duojin01 UGV ROS2

  [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
  [![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
  [![Gazebo: Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)

  **夺锦01 麦克纳姆轮 UGV 功能框架**
</div>

---

## License

本仓库中 `duojin01_*` 自研包采用 GNU General Public License v3.0（`GPL-3.0-only`）许可，完整文本见仓库根目录 `LICENSE`。

`src/OrbbecSDK` 与 `src/lslidar_driver` 为第三方代码，继续遵循其目录内声明的原始许可证。

---

## 0. 运行前准备

```bash
# 加载环境 1
source /opt/ros/humble/setup.bash

# 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境 2 （接下来的操作在执行了下面这个命令的终端内进行）
source install/setup.bash
```

---

## 1. 常用命令（先看这里）

### 1.1 真机

底盘联调（仅启动底盘，可以遥操，但没有其他用）：

```bash
ros2 launch duojin01_bringup base.launch.py use_lidar:=false use_foxglove:=true
```

建图：

```bash
ros2 launch duojin01_bringup mapping.launch.py
```

键盘操作（见下文说明 ## 4）：
```bash
ros2 run duojin01_teleop keyboard_teleop_node
```
  
保存地图（会有successfu、done的提示）：

```bash
ros2 launch duojin01_bringup save_map.launch.py
```

导航（已有地图，默认加载最新的）：

```bash
ros2 launch duojin01_bringup navigation.launch.py
```


### 1.2 仿真

基础启动（默认 GPU 档）：

```bash
ros2 launch duojin01_bringup sim.launch.py
```

如需开启仿真深度相机：

```bash
ros2 launch duojin01_bringup sim.launch.py use_sim_camera:=true
```

仿真建图：

```bash
ros2 launch duojin01_bringup sim_mapping.launch.py sim_profile:=gpu
```

`sim_mapping.launch.py` / `sim_navigation.launch.py` 同样支持 `use_sim_camera:=true`。

默认行为统一：
- `sim.launch.py`：默认启动手柄链路（`use_teleop:=true`）
- `sim_mapping.launch.py`：固定启动手柄链路（用于建图手动控车）
- `sim_navigation.launch.py`：固定关闭手柄链路（避免与导航控制冲突）

仿真导航（已有地图）：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py sim_profile:=gpu
```

仿真保存地图：

```bash
ros2 launch duojin01_bringup save_map.launch.py use_sim_time:=true
```

---

## 2. Launch 文件说明（duojin01_bringup）

### 2.1 真机相关

- `base.launch.py`：底盘基础链路（`base_driver`、`twist_mux`、`safety_watchdog`、`ekf`、`robot_state_publisher`）。
- `mapping.launch.py`：真机建图（`base + slam_toolbox + RViz`）。
- `navigation.launch.py`：真机导航（`base + nav2 + RViz`）。
- `nav2.launch.py`：只启动 Nav2（可选 RViz）。
- `slam.launch.py`：只启动 `slam_toolbox`。
- `joy_teleop.launch.py`：手柄链路。
- `save_map.launch.py`：等待 `/map` 后调用 `map_saver_cli`。

### 2.2 仿真相关

- `sim.launch.py`：仿真基础链路（Gazebo、桥接、控制器、底盘节点等）。
- `sim_mapping.launch.py`：仿真建图（`sim + slam_toolbox + scan_frame_rewriter + RViz`）。
- `sim_navigation.launch.py`：仿真导航（`sim + nav2 + scan_frame_rewriter`）。

---

## 3. 仿真渲染档位

待整理

```bash
# GPU 档（默认）
ros2 launch duojin01_bringup sim.launch.py sim_profile:=gpu

# CPU 档（兜底）
ros2 launch duojin01_bringup sim.launch.py sim_profile:=cpu

# 无界面（正常情况下推荐）
ros2 launch duojin01_bringup sim.launch.py headless:=true
```


---

## 4. 键盘说明

- 键盘的node启动后会在终端打印有关信息
- 速度调节可能暂时无效，不建议使用
- 键盘节点是检测按键状态，不是终端字符输入，是故窗口不一定要聚焦在node的终端、启动键盘操作后可能不便于进行其他造作
- 结束时不必聚焦node的终端，直接ctrl + c就结束了
- 蓝牙键盘在部分环境下可能不兼容，无法正常操作

```bash
ros2 run duojin01_teleop keyboard_teleop_node
```

---

## 5. 手柄控制

需要 PS2 布局手柄。高层 launch 通常会自动启动手柄链路，也可以单独启动：

```bash
ros2 launch duojin01_bringup joy_teleop.launch.py
```

默认逻辑：

- `RB(5)`：正常速度通道（`/cmd_vel_normal`）
- `LB(4)`：慢速通道（`/cmd_vel_slow`）
- `A(0)`：急停通道（`/cmd_vel_estop`）
- `Y(3)` 长按 2 秒：触发保存地图
- 左右遥感自行体会含义，后续补充操作说明

---

## 6. 参数速查

- `use_sim_time`：真机通常 `false`，仿真通常 `true`
- `use_rviz`：`mapping/navigation/sim_mapping/sim_navigation` 可开关
- `map`：`navigation` / `sim_navigation` 指定地图 yaml
- `headless`：仿真可无界面运行
- `sim_profile`：
  - `gpu`：默认，`ogre + 硬件渲染`
  - `cpu`：软件渲染，兼容优先
- `use_teleop`：`sim.launch.py` 可开关，默认 `true`
- `sim_mapping.launch.py`：手柄链路固定开启
- `sim_navigation.launch.py`：手柄链路固定关闭
- `use_sim_camera`：仅仿真有效，默认 `false`，可按需开启深度相机
- `use_foxglove`：`base` / `mapping` / `navigation` / `sim*` 可开关；`sim*` 默认 `false`
