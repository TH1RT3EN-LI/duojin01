# 雷达驱动来源与项目适配

- 官方仓库：<https://github.com/Lslidar/Lslidar_ROS2_driver>
- 分支：`master`
- 固定提交：`ecd9a836a922db714236aba7a0f54508adcb7e0e`
- 上游发布标识：`LSLIDAR_ROS2_V5.1.3_250822`，见本目录官方 README。
- 本地包版本：`lslidar_driver` 和 `lslidar_msgs` 均为 `5.1.3`。上游的 CMake/package.xml 仍写 `5.1.1`；本地将打包版本与其 README 发布标识对齐。
- 核对日期：2026-10-04。

旧 M10P/N10P 驱动及其旧消息文件已从当前工作树移除。新版本保留官方统一驱动的 X10、CX、CH、LS 实现；Duojin01 的默认入口是 X10/N10Plus。源文件和本目录 LICENSE 使用 Apache-2.0。

## 本机默认接入

`lslidar_x10_launch.py` 使用 `config/duojin01_n10plus.yaml`，不自动启动 RViz：

| 项目 | 默认值 |
| --- | --- |
| 连接 | `/dev/lslidar`，460800 波特率 |
| 型号 | `N10Plus`，108 字节、16 组双回波 |
| 坐标系 / 扫描话题 | `laser` / `/scan` |
| 点云话题 | `/lslidar_point_cloud` |
| 频率 / 扫描格数 | `N10Plus_hz: 10` / 540；由 5400 点/秒计算，不使用旧预热参数 |
| 距离过滤 | 0.30–100.0 m |
| 角度方向 | `invert_azimuth: true`，保留原 N10_P 驱动的顺时针协议角度转换 |

`base`、`mapping`、`navigation` 通过 `lidar_serial_port`、`lidar_model` 转发配置。雷达 include 使用独立的 launch 配置作用域，避免与 Nav2 的 `params_file` 混用。

## 对官方代码的适配

1. 加入上述真机配置和 launch；CMake/package.xml 补齐 yaml-cpp、Eigen、测试依赖并统一版本标识。
2. X10 加入可配置的 `invert_azimuth`；其他调用默认保持官方角度方向。
3. LaserScan 格数使用整数计算，保证高精度模式恰为十倍；修正 `angle_max`，合并最近有效回波，并将 ±π 边界归入同一个角度格。
4. 每次发布捕获独立的帧、时间戳和帧间隔；X10 采用单个发布工作线程，避免共享备份帧被下一帧覆盖和发布顺序变化。析构时先完成发布队列，再释放 X10 成员。MultiEchoLaserScan 同样捕获对应帧，修正秒/纳秒转换。
5. 初始化高精度默认值和变换矩阵；检查变换长度、N10Plus 频率以及数据包长度。
6. 串口读取累计分段数据，区分暂时无字节与实际断开；坏包丢弃后继续采集，传输失败时节点以失败状态退出。读取库使用 `rclcpp::ok()` 判断退出，移除对节点可执行程序全局标志的依赖，使驱动库可独立链接和测试。

默认沿用扫描结束时间，`time_increment` 为 0，因为输出按角度格重排，不能将等间隔采样时间直接套到该顺序。`use_first_point_time` 可选择帧开始时间，但角度重排后的第一束光时间与运动补偿仍需实机数据核对。

## 验证

在 Ubuntu 22.04/Humble 环境运行 `scripts/test_lidar.sh`。测试使用构造的协议包和容器内伪终端，覆盖校验、双回波、跨零角度、扫描几何、完整帧发布、时间戳顺序与串口分段读取；不会连接物理设备。

N10Plus 的波特率、长度、角度/距离偏移及校验方式与本项目旧 N10_P 解码一致，因此作为本机接入配置。上述证据来自源码与离线测试，尚未确认实际设备的铭牌、固件和完整实机采集表现。
