该包clone自镭神的[github仓库](https://github.com/Lslidar/Lslidar_ROS2_driver/tree/main?tab=readme-ov-file)的N10分支（last commit大概在23年），以下内容为仓库README.MD的中文翻译
**注意，其注明自己是在ROS2 Foxy下完成的测试并推荐使用此版本的ROS2，但是有部分社区中的 blog 表现出其在 humble 一样具备下的工作能力，这里先拿过来做测试**
# lslidar

## 描述

`lslidar` 软件包是一个基于 Linux 的 ROS2 激光雷达驱动，适用于 lslidar 的 **M10、M10_GPS、M10_P、M10_PLUS 以及 N10** 型号。

该软件包已在 **Ubuntu 20.04 + ROS2 Foxy** 环境下完成测试。

## 编译

这是一个 Catkin 软件包。在将该软件包克隆到你的工作空间后，请确保该软件包已经位于 `ROS_PACKAGE_PATH` 中。随后，按照 Catkin 软件包的常规流程进行编译即可。

```bash
cd your_work_space
colcon build
source install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py
```

打开一个新的终端窗口：

```bash
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1\    # 打开雷达
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0\    # 关闭雷达
```

```bash
ros2 launch lslidar_driver lslidar_launch.py
```

注意：
该 launch 文件会同时启动驱动节点，这是**唯一需要使用的启动文件**。

## 常见问题（FAQ）

## Bug 反馈

建议通过提交 Issue 的方式进行反馈。
你也可以通过电子邮件联系：[honghangli@lslidar.com](mailto:honghangli@lslidar.com)

## 版本
