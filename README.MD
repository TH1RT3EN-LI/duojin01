<div align="center">
  <img src="docs/images/hero.png" alt="duojin01_hero" width="600px" style="border-radius: 10px;">

  # Duojin01 UGV ROS2

  [![License: Proprietary](https://img.shields.io/badge/License-Proprietary-red.svg)](https://opensource.org/licenses/alphabetical)
  [![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
  [![Gazebo: Ignition](https://img.shields.io/badge/Gazebo-Ignition-orange.svg)](https://gazebosim.org/)

  **夺锦01 麦克纳姆轮UGV 功能框架**
</div>

---

## 0. 运行前准备

```bash
# 1) 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2) 加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 1. Launch 功能总览（duojin01_bringup）

### 1.1 真机相关

- `base.launch.py`
  - 底盘基础 bringup：`base_driver`、`twist_mux`、`safety_watchdog`、`ekf`、`robot_state_publisher`。
  - `use_lidar:=true` 时会同时拉起 `lslidar_launch.py` 和 `joy_teleop.launch.py`。
- `mapping.launch.py`
  - 真机建图一键启动：`base + slam_toolbox + RViz`。
- `navigation.launch.py`
  - 真机导航一键启动：`base + nav2 + RViz`。
  - 地图默认自动选择 `maps/` 下“最大数字文件名”或最新 yaml。
- `nav2.launch.py`
  - 只启动导航栈（Nav2 + 可选 RViz + 初始位姿发布器）。
- `slam.launch.py`
  - 只启动 `slam_toolbox`（`online_async_launch.py`）。
- `joy_teleop.launch.py`
  - 手柄链路：`joy_node + joy_axis_selector + teleop_twist_joy(慢/正常/急停) + joy_launcher`。
- `save_map.launch.py`
  - 等待 `/map` 后调用 `map_saver_cli` 保存地图；默认自动命名为下一个数字编号。

### 1.2 仿真相关

- `sim.launch.py`
  - 仿真基础 bringup：Ignition Gazebo、`ros_gz_bridge`、仿真控制器仿真器、`base_driver`、`twist_mux`、`safety_watchdog`、可选手柄。
  - 默认世界：`worlds/empty_world.sdf`。
- `sim_mapping.launch.py`
  - 仿真建图一键启动：`sim + slam_toolbox + scan_frame_rewriter + RViz`。
- `sim_navigation.launch.py`
  - 仿真导航一键启动：`sim + nav2 + scan_frame_rewriter`（Nav2 延时启动，避免仿真初期 TF 未稳定）。

---

## 2. 真机运行说明（Run）

### 2.1 真机底盘联调（仅底盘）

```bash
ros2 launch duojin01_bringup base.launch.py \
  use_lidar:=false \
  use_foxglove:=true
```

### 2.2 真机建图

```bash
ros2 launch duojin01_bringup mapping.launch.py

# 指定是否启动 rviz / foxglove
ros2 launch duojin01_bringup mapping.launch.py use_rviz:=false use_foxglove:=false
```

保存地图：

```bash
ros2 launch duojin01_bringup save_map.launch.py
```

可选参数：

```bash
# 指定地图名(默认为递增序号)
ros2 launch duojin01_bringup save_map.launch.py map_name:=my_lab

# 指定输出目录
ros2 launch duojin01_bringup save_map.launch.py output_dir:=/tmp/maps
```

### 2.3 真机导航（已有地图）

```bash
ros2 launch duojin01_bringup navigation.launch.py
```

指定地图：

```bash
ros2 launch duojin01_bringup navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

---

## 3. 仿真运行说明（Run）

### 3.1 仿真基础启动

```bash
ros2 launch duojin01_bringup sim.launch.py
```

常用参数：

```bash
# 无界面（headless）
ros2 launch duojin01_bringup sim.launch.py headless:=true

# 关闭手柄链路
ros2 launch duojin01_bringup sim.launch.py use_teleop:=false
```

### 3.2 仿真建图

```bash
ros2 launch duojin01_bringup sim_mapping.launch.py
```

在仿真中保存地图（注意 `use_sim_time:=true`）：

```bash
ros2 launch duojin01_bringup save_map.launch.py use_sim_time:=true
```

### 3.3 仿真导航（已有地图）

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py
```

指定地图：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py \
  map:=/absolute/path/to/map.yaml 
```

---

## 4. 键盘控制（单独说明）

当前项目的键盘控制节点是 `duojin01_teleop/keyboard_teleop_node.py`，不是 launch 文件，单独 `ros2 run`。
*注意当前的键盘控制不是在终端读取输入，是直接检测键盘是否被按下，测试中发现蓝牙键盘可能无法使用该功能*

### 4.1 启动命令

```bash
ros2 run duojin01_teleop keyboard_teleop_node 
```

### 4.2 按键定义

- `W/S`：前进/后退（x）
- `A/D`：左移/右移（y）
- `Q/E`：左转/右转（yaw）
- `X`：急停（速度归零）
- `I/K`：线速度 `+10% / -10%`
- `O/L`：角速度 `+10% / -10%`

---

## 5. 手柄控制

需要一个 PS2 布局的手柄。
无须单独启动，更高层次的 launch 文件大多会默认启动手柄控制
```bash
ros2 launch duojin01_bringup joy_teleop.launch.py
```

### 5.1 当前手柄逻辑（按默认配置）

- `RB(5)` 按住：正常速度模式（`/cmd_vel_normal`，约 `x/y=0.4`，`yaw=1.0`），左右遥感/十字方向键操作
- `LB(4)` 按住：慢速模式（`/cmd_vel_slow`，约 `x/y=0.15`，`yaw=0.5`），左右遥感/十字方向键操作
- `A(0)` 按住：急停通道（`/cmd_vel_estop`，高优先级）
- `Y(3)` 长按 2 秒：触发 `save_map.launch.py`（并有震动反馈）

具体操作的覆盖逻辑可以参考 config 里对应的优先级配置 / 自己摸索，细节的处理可能不太符合直觉

---

## 6. 常见参数速查

- `use_sim_time`
  - 真机一般 `false`，仿真一般 `true`
- `use_rviz`
  - `mapping/navigation/sim_mapping/sim_navigation` 可开关 RViz
- `map`
  - `navigation` / `sim_navigation` 指定地图 yaml
- `headless`
  - `sim` / `sim_mapping` / `sim_navigation` 可无界面运行
- `use_foxglove`
  - `base` / `mapping` / `navigation` / `sim*` 可开关 foxglove bridge

