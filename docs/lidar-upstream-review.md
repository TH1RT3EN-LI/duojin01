# 雷达驱动升级与官方仓库对比

核对及升级日期：2026-10-04。项目分支：`hardware-only`，升级前基线：`3a1b64792de34b1899a84e7ebb18e01cb43b873e`。

**当前已用官方统一驱动 V5.1.3_250822 替换旧 `M10P/N10P` 定制驱动。来源固定在 `master` 的 `ecd9a836`；本机默认接入为 X10/N10Plus，已适配真机参数、角度方向、双回波合并和发布队列。旧驱动及其旧消息文件已删除。**

来源与补丁清单见 [UPSTREAM.md](../src/lslidar_driver/UPSTREAM.md)。本文保留升级前差异，便于核对迁移过程中保留或调整的行为。

## 1. 来源和更新情况

本地驱动在 `src/lslidar_driver/`，上游为 [Lslidar/Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver)。按型号分支比较，不能只看默认分支更新时间。

| 上游分支 | 核对时最新提交 | 提交日期（UTC+8） | 含义 |
| --- | --- | --- | --- |
| `M10P/N10P` | [`8bb760c`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa) | 2023-05-29 | 与本地代码匹配的旧型号分支，仅一个提交 |
| `master` | [`ecd9a83`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/ecd9a836a922db714236aba7a0f54508adcb7e0e) | 2025-11-03 | 统一驱动；这个提交只改英文 README |
| `main` | [`a8f0b2c`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/a8f0b2c7c3c32c4c7cfe1f0a23437c910c29af35) | 2026-06-25 | 此分支只有型号索引 README，更新不是驱动代码更新 |

`master` 最后一次涉及驱动代码/配置的提交为 [`215e064`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/215e0646baa94bea09664e11c9de266675ec313b)，日期为 **2025-08-22**：扩展 CX1S3 转速设置，并纠正 CH16X1 名称。这两项不是当前 N10_P 的专门修复。2025-11-03 的前一个提交主要调整文件位置。

旧型号分支和 `master` 没有共同 Git 祖先，不能把统一版当作旧分支直接快进升级。当前统一版 [README](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/README.md) 标为 `V5.1.3_250822`，但 [package.xml](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/package.xml) 仍写 `5.1.1`；记录提交 SHA 比只记版本号准确。上游 README 明确列出 Ubuntu 22.04 + ROS 2 Humble。

## 2. 升级前本地与旧型号分支的差异

以下均指升级前基线。按文件内容逐字节比较，排除 `.git`：旧分支 24 个文件，本地 20 个文件；**14 个相同、6 个修改、4 个未保留**。

| 文件或组件 | 对比结果 |
| --- | --- |
| `lslidar_msgs/` 全部 7 个文件 | 完全相同，消息接口没有本地改动 |
| 驱动 `CMakeLists.txt`、`input.h`、`lsiosr.h` | 完全相同 |
| `input.cc`、`lsiosr.cpp`、`lslidar_driver_node.cc`、RViz 配置 | 完全相同 |
| `lslidar_driver.cc`、`lslidar_driver.h` | 本地修改了扫描数据发布与状态同步 |
| `package.xml`、`lslidar_launch.py`、`params/lsx10.yaml`、根 README | 本地修改了依赖声明、启动方式、硬件参数及说明 |
| 双雷达示例 launch、两个双雷达参数文件、`version.txt` | 本地未保留 |

旧分支和本地 `package.xml` 都是 `1.2.0`，版本号没有体现定制改动。旧 `version.txt` 写的是 `LSLIDAR_M10_N10_V2.5.2_230110_ROS2`。驱动包目录的差异统计为 8 个文件、299 行新增、273 行删除；此统计不含根 README 和根 `version.txt`。

本地主要改动：

- **N10_P 双回波合并**：同一角度取较近的有效回波，输出每个角度一个距离值；原版的数组长度为 `2N`，角度步长却按 `N` 计算。本地还在多个点落入同一角度格时保留较近值。
- **扫描点数稳定**：增加 `fixed_scan_count` 和 `scan_count_warmup`。升级前参数为 `0`、`10`，先观察 10 帧，取最大点数作为后续输出长度，预热完成前不发布扫描。显式点数小于当前点数时，代码会扩大输出，因此显式设置不是绝对固定长度保证。
- **角度一致性和边界检查**：`angle_max = angle_min + angle_increment × (N−1)`，补充有效点数、回波索引和角度索引检查。
- **跨线程快照**：点数与扫描点数组在同一互斥锁下保存、读取，避免使用另一帧的点数。
- **时间处理**：初始化首帧时间，普通扫描时间戳从发布时的 `now()` 改为 `start_time + scan_time`。
- **项目接入**：取消雷达 launch 自动启动 RViz；清理依赖声明，使用真实串口、项目 `laser` 坐标系和 `/scan`。

这些差异集中在 LaserScan 后处理和项目接入。底层串口实现、网络输入实现均与旧上游相同；旧上游已经包含 N10_P 解码，不能从本地 README 的“原版本似乎不适用”推断官方完全不支持这个型号。该 README 也沿用了 Catkin/Foxy 说明，实际构建使用 `ament_cmake` 和 `colcon`。

旧 `lslidar_driver.cc` 最后修改于项目提交 `f30003e`（2026-02-10）；升级后该文件已移除。

## 3. 接口迁移与当前配置

统一版将型号实现拆为 X10、CX、CH、LS 等驱动，当前单线雷达对应 X10 路径。以下对比基于上游 [`lslidar_x10.yaml`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/config/lslidar_x10.yaml) 与 [`lslidar_x10_launch.py`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/launch/lslidar_x10_launch.py)。

| 项目 | 升级前 | 官方统一版默认值或接口 | 当前项目适配 |
| --- | --- | --- | --- |
| 启动文件 | `lslidar_launch.py` | `lslidar_x10_launch.py` | 普通 ROS Node，不自动启动 RViz |
| 参数文件 | `params/lsx10.yaml` | `config/lslidar_x10.yaml` | `config/duojin01_n10plus.yaml` |
| 型号参数 | `lidar_name: N10_P` | `lidar_type` + `lidar_model` | `X10` + `N10Plus` |
| 串口 | `serial_port_: /dev/lslidar` | `serial_port`；空字符串走网络 | `/dev/lslidar`、460800 波特率 |
| 坐标系 | `laser` | `laser_link` | `laser` |
| 扫描话题 | `/scan` | `/x10/scan` | `/scan`，无 `x10` 命名空间 |
| 点云输出 | 默认关闭 | 始终发布点云 | `/lslidar_point_cloud` |
| 电机输入话题 | `/lslidar_order`，Int8 | `/x10/motor_control`，Int8 | `/motor_control`，Int8 |
| 距离过滤 | 0.30–100.0 m | 0.15–50.0 m | 0.30–100.0 m |
| 扫描格数 | 预热观察或显式指定 | 型号脉冲频率 / 转速 | 5400 / 10 Hz = 540；高精度模式十倍格数 |
| 扫描排序 | 0 至 2π | −π 至 π | 沿用新数组排序，`invert_azimuth: true` 保留旧角度方向 |
| 双回波 | 同格取最近有效距离 | 点云转换为扫描 | 保留最近有效距离合并及边界检查 |
| 逐点时间间隔 | `scan_time / 输出点数` | `0` | `0`，角度排序后不假定等间隔采集时间 |
| 消息包 | 5 个旧消息 | 2 个消息、11 个服务 | 旧接口删除，完整替换为新消息包 |

`base`、`mapping`、`navigation` 统一提供 `lidar_serial_port` 和 `lidar_model`。雷达子 launch 使用独立参数作用域，避免其 `params_file` 与 Nav2 的配置相互覆盖。上游 X10 launch 使用 LifecycleNode，但可执行程序实际创建普通 `rclcpp::Node`；项目入口使用普通 Node。

统一版有孤立点滤波开关，但上游明确注明 **N10Plus 不生效**。不能把它算作当前设备升级后必然可用的功能。

### N10_P 与 N10Plus 的协议核对

升级前 `lslidar_driver.cc` 和上游 [`lslidar_x10_driver.cpp`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/src/lslidar_x10_driver.cpp) 中的配置及双回波解码结构一致：

| 协议特征 | 升级前 N10_P | 上游 N10Plus |
| --- | --- | --- |
| 串口波特率 | 460800 | 460800 |
| 数据包长度 | 108 字节 | 108 字节 |
| 每包角度组数 | 16 | 16 |
| 起始角度/距离数据/结束角度偏移 | 5 / 7 / 105 | 5 / 7 / 105 |
| 每个角度的回波数 | 2 | 2 |
| 校验 | 前 107 字节累加、低 8 位存于第 108 字节 | 相同 |

**根据上述源码和离线协议验证，当前默认使用 N10Plus。尚未读取铭牌、固件或真实原始数据，不能认定所有 N10_P 都与 N10Plus 兼容。** 普通 N10 是 230400 波特率、58 字节包，与本项目旧协议不同。

## 4. 项目补丁与验证边界

新版源码已替换并完成接入适配；同时修正了离线检查中发现的以下问题：

- X10 发布队列原先读取共享备份帧和时间变量，可能被下一帧覆盖。现在捕获独立帧、对应时间戳与帧间隔，单线程按序发布；析构时先完成队列。
- 修正 LaserScan 角度上限、整数格数、±π 边界及 MultiEchoLaserScan 的秒/纳秒转换；初始化高精度默认值和变换矩阵。
- 校验变换矩阵长度、N10Plus 频率和数据包长度；丢弃校验失败的数据包后继续采集。
- 串口累计读取分段数据，区分暂时无数据与断开。传输失败时节点非零退出；驱动库移除对主程序全局退出标志的依赖，可独立链接和测试。
- 驱动 CMake 和两个包的 package.xml 版本均改为 `5.1.3`，对齐官方 README 发布标识；补齐 yaml-cpp、Eigen 和测试依赖。

默认时间戳沿用旧配置的扫描结束时刻，而 [ROS 2 Humble 的 LaserScan 定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) 要求头时间戳表示第一束光的采集时刻。新版保留 `use_first_point_time` 选项，但角度重排后的第一束光时间仍需实机数据核对。此次验证确认帧与时间戳对应及发布顺序，未验证移动中的 TF/运动补偿精度。

Ubuntu 22.04/Humble Docker 构建覆盖全部 10 个真机包。`scripts/validate_hardware.sh` 验证 26 项 Python 测试及 10 个启动入口解析；`scripts/test_lidar.sh` 验证 10 项 C++ 测试，使用构造协议包及容器内伪终端，覆盖双回波、损坏包、跨零角度、最近距离、540/5400 格数、无效配置、不同帧的数据与时间戳、分段读取及断开。没有连接物理雷达，也没有发送电机命令。

原工作区的 release 分支仍保留旧驱动和旧消息。本次替换只发生在 `hardware-only`；原工作区的未提交修改保持原样。

## 5. 可复核记录

完整上游提交：

```text
M10P/N10P  8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa
master     ecd9a836a922db714236aba7a0f54508adcb7e0e
main       a8f0b2c7c3c32c4c7cfe1f0a23437c910c29af35
```

升级前基线的旧驱动文件 SHA-256（当前已删除）：

```text
lslidar_driver/src/lslidar_driver.cc
b2978bfdd4572ad97b9eae78e37dbe5ddbdd3acb279ec274c157ceaefd2a6cdf
lslidar_driver/include/lslidar_driver/lslidar_driver.h
829dae45fab288cb944aba3ddeb26f222c08a0c73600a7c5c90851c771e21a65
lslidar_driver/params/lsx10.yaml
0228210f666f37c3295ec1f11ac82363167eb20482394897f5166dcd42f7ee0b
```
