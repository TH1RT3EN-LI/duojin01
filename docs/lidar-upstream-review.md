# 雷达驱动与官方仓库对比

核对日期：2026-10-04。项目分支：`hardware-only`，对比前提交：`340e1fff5f06f48dc72d0393a2b4c42f119a2e29`。

**官方有较新的统一驱动；当前项目是旧 `M10P/N10P` 驱动的定制版本，并不与最新统一版相同。旧型号分支本身没有新的提交。此次核对保留现有雷达源码及参数。**

## 1. 来源和更新情况

本地驱动在 `src/lslidar_driver/`，上游为 [Lslidar/Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver)。按型号分支比较，不能只看默认分支更新时间。

| 上游分支 | 核对时最新提交 | 提交日期（UTC+8） | 含义 |
| --- | --- | --- | --- |
| `M10P/N10P` | [`8bb760c`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa) | 2023-05-29 | 与本地代码匹配的旧型号分支，仅一个提交 |
| `master` | [`ecd9a83`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/ecd9a836a922db714236aba7a0f54508adcb7e0e) | 2025-11-03 | 统一驱动；这个提交只改英文 README |
| `main` | [`a8f0b2c`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/a8f0b2c7c3c32c4c7cfe1f0a23437c910c29af35) | 2026-06-25 | 此分支只有型号索引 README，更新不是驱动代码更新 |

`master` 最后一次涉及驱动代码/配置的提交为 [`215e064`](https://github.com/Lslidar/Lslidar_ROS2_driver/commit/215e0646baa94bea09664e11c9de266675ec313b)，日期为 **2025-08-22**：扩展 CX1S3 转速设置，并纠正 CH16X1 名称。这两项不是当前 N10_P 的专门修复。2025-11-03 的前一个提交主要调整文件位置。

旧型号分支和 `master` 没有共同 Git 祖先，不能把统一版当作旧分支直接快进升级。当前统一版 [README](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/README.md) 标为 `V5.1.3_250822`，但 [package.xml](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/package.xml) 仍写 `5.1.1`；记录提交 SHA 比只记版本号准确。上游 README 明确列出 Ubuntu 22.04 + ROS 2 Humble。

## 2. 本地与旧型号分支的差异

按文件内容逐字节比较，排除 `.git`：旧分支 24 个文件，本地 20 个文件；**14 个相同、6 个修改、4 个未保留**。

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
- **扫描点数稳定**：增加 `fixed_scan_count` 和 `scan_count_warmup`。当前参数为 `0`、`10`，先观察 10 帧，取最大点数作为后续输出长度，预热完成前不发布扫描。显式点数小于当前点数时，代码会扩大输出，因此显式设置不是绝对固定长度保证。
- **角度一致性和边界检查**：`angle_max = angle_min + angle_increment × (N−1)`，补充有效点数、回波索引和角度索引检查。
- **跨线程快照**：点数与扫描点数组在同一互斥锁下保存、读取，避免使用另一帧的点数。
- **时间处理**：初始化首帧时间，普通扫描时间戳从发布时的 `now()` 改为 `start_time + scan_time`。
- **项目接入**：取消雷达 launch 自动启动 RViz；清理依赖声明，使用真实串口、项目 `laser` 坐标系和 `/scan`。

这些差异集中在 LaserScan 后处理和项目接入。底层串口实现、网络输入实现均与旧上游相同；旧上游已经包含 N10_P 解码，不能从本地 README 的“原版本似乎不适用”推断官方完全不支持这个型号。该 README 也沿用了 Catkin/Foxy 说明，实际构建使用 `ament_cmake` 和 `colcon`。

本地 `lslidar_driver.cc` 最后修改于项目提交 `f30003e`（2026-02-10）。本次仅核对，不改变它的运行行为。

## 3. 与新统一驱动的接入差异

统一版将型号实现拆为 X10、CX、CH、LS 等驱动，当前单线雷达对应 X10 路径。以下对比基于上游 [`lslidar_x10.yaml`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/config/lslidar_x10.yaml) 与 [`lslidar_x10_launch.py`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/launch/lslidar_x10_launch.py)。

| 项目 | 当前项目 | 新统一版默认值或接口 |
| --- | --- | --- |
| 启动文件 | `lslidar_launch.py` | `lslidar_x10_launch.py` |
| 参数文件 | `params/lsx10.yaml` | `config/lslidar_x10.yaml` |
| 型号参数 | `lidar_name: N10_P` | `lidar_type: X10` + `lidar_model`；接受 `N10Plus`，不接受 `N10_P` |
| 串口选择 | `interface_selection: serial`、`serial_port_: /dev/lslidar` | `serial_port`；默认空字符串走网络 |
| 坐标系 | `laser` | `laser_link` |
| 扫描话题 | `scan_topic: /scan` | `laserscan_topic: scan`，默认 `x10` 命名空间后为 `/x10/scan` |
| 输出开关 | `pubScan`、`pubPointCloud2` | `publish_scan`；另外提供 MultiEchoLaserScan 开关 |
| 电机命令话题 | `/lslidar_order`，Int8 | `/x10/motor_control`，Int8 |
| 距离过滤默认值 | `0.30–100.0 m` | `0.15–50.0 m` |
| 扫描点数 | 预热观察或显式指定 | 由型号脉冲频率与转速计算；没有本地两个点数参数 |
| 扫描角度范围 | `0` 到 `2π−角度步长` | `−π` 到 `π`；角度索引顺序不同 |
| 逐点时间间隔 | `scan_time / 输出点数` | 点云转换为 LaserScan 时设为 `0` |
| 新功能 | 定制双回波合并与扫描稳定 | 多型号统一实现、高精度扫描、N10Plus 多回波消息等 |

统一版有孤立点滤波开关，但上游明确注明 **N10Plus 不生效**。不能把它算作当前设备升级后必然可用的功能。

### N10_P 与 N10Plus 的协议核对

本地 `lslidar_driver.cc` 和上游 [`lslidar_x10_driver.cpp`](https://github.com/Lslidar/Lslidar_ROS2_driver/blob/ecd9a836a922db714236aba7a0f54508adcb7e0e/lslidar_driver/src/lslidar_x10_driver.cpp) 中的配置及双回波解码结构一致：

| 协议特征 | 本地 N10_P | 上游 N10Plus |
| --- | --- | --- |
| 串口波特率 | 460800 | 460800 |
| 数据包长度 | 108 字节 | 108 字节 |
| 每包角度组数 | 16 | 16 |
| 起始角度/距离数据/结束角度偏移 | 5 / 7 / 105 | 5 / 7 / 105 |
| 每个角度的回波数 | 2 | 2 |

**由源码推断，N10Plus 是本机迁移时应核对的候选型号。尚未读取铭牌、固件或真实原始数据，不能认定所有 N10_P 都与 N10Plus 兼容。** 普通 N10 是 230400 波特率、58 字节包，不能因为名称相近就选择它。

## 4. 当前判断及验证边界

当前驱动已经针对项目扫描输出做过定制，新版不能直接覆盖目录后原样使用。若以后迁移，需要同时适配型号、串口参数、`laser` 坐标系、`/scan` 话题和启动文件，并保留或重新验证扫描点数、双回波选择与时间处理。新版的点数计算与本地预热机制不同；角度区间和逐点时间来自上游 `pointcloudToLaserscan()` 的静态检查，尚未测量它们对本机建图与导航的影响。

时间戳需要单独核对：本地使用扫描结束时刻，而 [ROS 2 Humble 的 LaserScan 定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) 要求头时间戳表示第一束光的采集时刻。结合本地角度重排，需用真实数据检查首束光对应关系和 `time_increment`；仅凭改用采集时间不能宣布 TF/运动补偿问题已经解决。

本次完成远端 Git 引用核对、提交历史检查、源码逐文件比较及参数/协议静态审查；没有启动雷达，没有验证新版实机表现。Ubuntu 22.04/Humble 容器验证针对保留的本地驱动。

另外，原工作区的 release 分支与 `hardware-only` 的雷达 C++ 和消息代码相同。release 中 `min_range` 是 `0.05 m`，本分支是 `0.30 m`；release 的雷达 launch 允许覆盖 `params_file`、`scan_topic`，本分支直接加载包内 YAML。这些是项目分支之间的配置区别，不属于上游新增更新。

## 5. 可复核记录

完整上游提交：

```text
M10P/N10P  8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa
master     ecd9a836a922db714236aba7a0f54508adcb7e0e
main       a8f0b2c7c3c32c4c7cfe1f0a23437c910c29af35
```

本地文件 SHA-256：

```text
lslidar_driver/src/lslidar_driver.cc
b2978bfdd4572ad97b9eae78e37dbe5ddbdd3acb279ec274c157ceaefd2a6cdf
lslidar_driver/include/lslidar_driver/lslidar_driver.h
829dae45fab288cb944aba3ddeb26f222c08a0c73600a7c5c90851c771e21a65
lslidar_driver/params/lsx10.yaml
0228210f666f37c3295ec1f11ac82363167eb20482394897f5166dcd42f7ee0b
```
