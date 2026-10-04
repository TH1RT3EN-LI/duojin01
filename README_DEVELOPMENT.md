# 真机开发接口

运行平台为 Ubuntu 22.04 + ROS 2 Humble。底盘由真实串口驱动提供 `/odom`、`/imu`、`/battery_voltage`，经 EKF 输出 `/odometry/filtered`。导航保留 Nav2 规划和避障。

## 当前模块

工作区包含以下 10 个 ROS 包：

| 包 | 职责 |
| --- | --- |
| `duojin01_base_driver` | 麦克纳姆底盘串口通信、速度命令、里程计/IMU/电池反馈 |
| `lslidar_driver` | 官方统一驱动 V5.1.3，默认 X10/N10Plus 串口接入，发布 `/scan` 和点云 |
| `lslidar_msgs` | 统一驱动的两种消息及 11 种服务定义 |
| `duojin01_camera` | USB 彩色相机、图像/内参发布和相机标定 |
| `duojin01_description` | 真机 URDF、底盘/雷达/IMU 模型与 TF |
| `duojin01_bringup` | 统一启动和配置，集成 EKF、速度仲裁、建图、定位与 Nav2 |
| `duojin01_slam_tools` | 完整地图快照、校验清单、离线建图和回放输入过滤 |
| `duojin01_teleop` | 键盘控制；手柄使用 bringup 中的 Joy 配置 |
| `duojin01_mission` | 导航任务、相机捕获、AprilTag 识别与机械臂串口 G-code 工具 |
| `duojin01_msgs` | 任务识别结果等自定义接口 |

`robot_localization`、`twist_mux`、`slam_toolbox`、Nav2、Joy 和 Foxglove 是外部 ROS 依赖，由 Docker/rosdep 安装；SLAM Toolbox 另外从固定源码构建并应用项目补丁。深度相机 SDK、深度相机模型和启动入口、安全看门狗、手柄急停输入及锁定配置已删除。

## 底盘与速度链路

`/cmd_vel`（导航和键盘）、`/cmd_vel_normal`、`/cmd_vel_slow` 进入 `twist_mux`，其 `/cmd_vel_base` 输出连接底盘驱动。速度仲裁按配置中的优先级选择有效输入，底盘驱动继续发布真实里程计、IMU 和电池电压。

`base.launch.py` 的 `base_serial_port` 和 `base_serial_baudrate` 对应底盘驱动参数 `usart_port_name` 和 `serial_baud_rate`。

## 雷达

来源固定在官方 `master` 提交 `ecd9a836a922db714236aba7a0f54508adcb7e0e`，本地两个雷达包统一标为 `5.1.3`。配置文件是 `lslidar_driver/config/duojin01_n10plus.yaml`，独立入口为 `lslidar_x10_launch.py`；`base`、`mapping`、`navigation` 的 `lidar_serial_port` 和 `lidar_model` 会转发至该入口。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | `laser` 坐标系、540 个角度格，供建图和导航使用 |
| `/lslidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 雷达点云 |
| `/motor_control` | `std_msgs/msg/Int8` | 新版电机控制输入，替代旧 `/lslidar_order`；沿用官方命令语义 |

默认配置使用 `/dev/lslidar`、460800 波特率、108 字节双回波协议和 10 Hz 格数计算。`invert_azimuth: true` 保留旧驱动的角度方向；扫描数组按 −π 至 π 排序，取每格最近有效距离。雷达子 launch 使用独立参数作用域，Nav2 的 `params_file` 与雷达参数互不覆盖。具体适配和验证边界见 [驱动来源](src/lslidar_driver/UPSTREAM.md) 与 [升级对比](docs/lidar-upstream-review.md)。

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

Docker 默认将源码和地图挂载到 `/ws`，安装好的 ROS 包在 `/opt/duojin01/install`。开发构建覆盖层使用 `/ws/install/slam_backend` 与 `/ws/install/hardware`，均检查目标架构。`DUOJIN01_WORKSPACE_ROOT` 同时控制导航选图和 Python 地图保存的根目录；原生 `source.sh` 默认使用当前仓库根目录，也保留用户显式设置的绝对路径。

依赖边界和离线启动描述检查位于 `tests/test_hardware_boundary.py`；运行 `scripts/validate_hardware.sh` 不会打开串口或发出运动命令。

雷达测试位于 `lslidar_driver/test/test_x10_driver.cpp`，运行 `scripts/test_lidar.sh` 构建到独立的 `build/lidar-tests`，不会加入日常 `install/hardware` 覆盖层。测试只使用构造的协议数据与容器内伪终端。

在线 / 离线 SLAM 的接口、录包 QoS、地图快照和升级分析见 [SLAM 升级与验证](docs/slam-upgrade-review.md)。
