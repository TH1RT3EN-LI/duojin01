**下面为对 (github)[https://github.com/Lslidar/Lslidar_ROS2_driver/tree/main] 仓库下的README.md的中文翻译**
**原始版本功能包可见仓库源码**
**当前版本是在原有基础上对 N10_P 支线的优化，原版本似乎不适用于该雷达**
# lslidar

## 说明

`lslidar` 包是一个 Linux 下的 ROS2 驱动，用于 lslidar 的 M10、M10_GPS、M10_P、M10_PLUS 和 N10。
该包已在 Ubuntu 20.04 + ROS2 Foxy 上测试。

## 编译

这是一个 Catkin 包。将包克隆到你的工作空间后，请确保该包位于 `ROS_PACKAGE_PATH` 中。然后按 Catkin 包的常规流程编译即可。

```bash
cd your_work_space
colcon build
source install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py

# 打开新终端
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1\        (打开雷达)
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0\        (关闭雷达)

ros2 launch lslidar_driver lslidar_launch.py
```

注意：该 launch 文件会同时启动驱动程序，这是唯一需要使用的 launch 文件。

## 常见问题（FAQ）

## Bug 反馈（Bug Report）

建议优先提交 issue。你也可以发送邮件到 [honghangli@lslidar.com](mailto:honghangli@lslidar.com)。


