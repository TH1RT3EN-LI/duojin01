# 真机开发接口

运行平台为 Ubuntu 22.04 + ROS 2 Humble。底盘由真实串口驱动提供 `/odom`、`/imu`、`/battery_voltage`，经 EKF 输出 `/odometry/filtered`。导航保留 Nav2 规划和避障。

## 底盘与安全链路

`/cmd_vel`（导航）、`/cmd_vel_normal`、`/cmd_vel_slow`、`/cmd_vel_estop` 进入 `twist_mux`，其 `/cmd_vel_safe` 输出连接底盘驱动。安全看门狗通过 `/watchdog/lock` 锁定速度输出，监测真实里程计、IMU、电池电压和倾斜。现有优先级与保护阈值保留在对应配置文件中。

可用 `/watchdog/status` 查看状态，用 `/watchdog/reset`（`std_srvs/srv/Trigger`）解除已消除的故障。`base.launch.py` 的 `base_serial_port` 和 `base_serial_baudrate` 对应底盘驱动参数 `usart_port_name` 和 `serial_baud_rate`。

## 导航

标准入口是 `/navigate_to_pose`，Action 类型为 `nav2_msgs/action/NavigateToPose`，目标使用 `map` 坐标系。`mission_executor` 也保留了包装接口：

| 输入/输出 | 类型 | 用途 |
| --- | --- | --- |
| `/mission/nav_goal` | `geometry_msgs/msg/PoseStamped` | 提交导航目标 |
| `/mission/nav_result` | `std_msgs/msg/Bool` | 导航结果 |

真机启动不自动发布预设初始位姿；应先通过 RViz 的 2D Pose Estimate 或 `/initialpose` 定位。当前任务执行器在导航中会忽略新的导航请求，沿用原有接口行为。

## 相机与识别

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 真实 USB 相机图像 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参和畸变 |
| `/mission/capture_trigger` | `std_msgs/msg/Bool` | `true` 请求一帧触发后的图像 |
| `/mission/image_capture` | `sensor_msgs/msg/Image` | 捕获结果 |

相机标定：

```bash
ros2 run duojin01_camera calibrate_camera \
  --device 0 --cols 9 --rows 6 --square 0.025 --output /ws/calibration.yaml
```

AprilTag 工具位于 `duojin01_mission.april_tag_detector`，输出 `duojin01_msgs/TargetInfo`。Tag 尺寸和相机标定应按实际物体与硬件设置。

## 机械臂串口

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/mission/gcode_cmd` | `std_msgs/msg/String` | 将控制器支持的 G-code 发送到机械臂串口 |
| `/mission/gcode_result` | `std_msgs/msg/Bool` | 串口调用结果 |

启动 `mission.launch.py` 时显式指定机械臂的 `serial_port`；底盘、雷达与机械臂应各自使用对应设备。任务执行器只提供导航、摄像与串口 API，不自动运行任务序列。

现有真机工具入口为 `demo`、`grab_calibration_demo`、`nav_demo`、`pick_demo`、`place_demo` 和 `mission_executor`，可用 `ros2 pkg executables duojin01_mission` 查看。使用前应根据实际相机、Tag、机械臂尺寸与串口控制器核对各工具参数。

## 工作区与 Docker 数据

Docker 默认将源码和地图挂载到 `/ws`，安装好的 ROS 包在 `/opt/duojin01/install`。开发构建覆盖层仅使用 `/ws/install/hardware`。`DUOJIN01_WORKSPACE_ROOT` 同时控制 Python 导航选图和 C++ 地图保存的根目录；原生 `source.sh` 默认使用当前仓库根目录，也保留用户显式设置的绝对路径。

依赖边界和离线启动描述检查位于 `tests/test_hardware_boundary.py`；运行 `scripts/validate_hardware.sh` 不会打开串口或发出运动命令。
