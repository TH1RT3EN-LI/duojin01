# 当前开发接口说明（部分）

- 底盘导航走 Nav2 `NavigateToPose` action。
- 机械臂只走 G-code。
- 吸取和放下也按 G-code 处理。
- 相机直接订阅图像话题。


## 1. Nav2 Action 小 Demo

Nav2 的目标点通过 `NavigateToPose` action 发送。Action 名称是 `/navigate_to_pose`，目标类型是 `geometry_msgs/msg/PoseStamped`。

下面是一个最小 Python demo：给机器人发一个 `map` 坐标系下的目标点，等导航完成后退出。

```python
#!/usr/bin/env python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Nav2ActionDemo(Node):
    def __init__(self):
        super().__init__("nav2_action_demo")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def go(self, x: float, y: float, yaw: float):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 action server /navigate_to_pose not available")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"navigation result: {'success' if ok else 'failed'}")
        return ok


def main():
    rclpy.init()
    node = Nav2ActionDemo()
    try:
        node.go(x=1.0, y=0.0, yaw=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```


仿真启动：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py sim_profile:=gpu
```

## 2. 机械臂移动
给用户只暴露一个机械臂入口：

| 用途 | Topic | 消息类型 |
| --- | --- | --- |
| 发送机械臂 G-code | `/arm/gcode_cmd` | `std_msgs/msg/String` |
| 监听执行结果 | `/arm/gcode_result` | `std_msgs/msg/Bool` |
| 当前末端位姿 | `/arm/end_effector_pose` | `geometry_msgs/msg/PoseStamped` |

另有一个按需查询服务：

| 用途 | Service | 类型 |
| --- | --- | --- |
| 查询当前末端位姿 | `/arm/query_end_effector_pose` | `std_srvs/srv/Trigger` |

仿真里由 `e4_gcode_adapter_node` 把 G-code 转成轨迹；真机里由任务执行节点把 G-code 发到串口控制器
仿真里的末端位置默认按机械臂 G-code 坐标输出，`x/y/z` 和 `/arm/gcode_cmd` 里的 `G90/G91` 使用同一套坐标约定：`+X` 向前、`+Y` 向左、`+Z` 向上，单位为米。
`/arm/end_effector_pose` 中只有 `pose.position.x/y/z` 表示当前末端坐标；`pose.orientation` 目前固定为中性四元数，不表示真实末端姿态。

命令示例：

```bash
# 回零
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String '{data: "$H"}'

# 绝对坐标移动，单位按 G-code 习惯使用 mm
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G90 G1 X190 Y0 Z280 F2000'}"

# 相对当前位置向下移动 20 mm （***建议多使用相对移动的方法驱动！）
# X Y Z后直接连接一个整数，表示了在对应方向的正方向上移动的距离（单位:mm）
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G91 G1 X10 Y10 Z-20 F800'}"

# 查询执行结果
ros2 topic echo /arm/gcode_result

# 持续监听当前末端位姿
ros2 topic echo /arm/end_effector_pose

# 按需查询当前末端位姿，返回 JSON 字符串
ros2 service call /arm/query_end_effector_pose std_srvs/srv/Trigger '{}'
```


## 3. 机械臂吸取和放下也是 G-code

吸取、关闭吸取、放下，都不要作为独立接口写给用户。用户仍然只发 `/arm/gcode_cmd`。

建议约定：

```bash
# 开启吸取
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String "{data: 'M3 S1000'}"

# 关闭吸取，也就是放下
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String "{data: 'M3 S0'}"
```

一次完整抓取可以写成：

```bash
# 到物体上方大致位置
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G90 G1 X190 Y0 Z300 F2000'}"

# 下探
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G91 G1 Z-30 F800'}"

# 吸取
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String "{data: 'M3 S1000'}"

# 抬起
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G91 G1 Z30 F1200'}"
```

一次完整放下可以写成：

```bash
# 到放置点上方
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G90 G1 X160 Y80 Z300 F2000'}"

# 下探
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G91 G1 Z-25 F800'}"

# 关闭吸取，物体放下
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String "{data: 'M3 S0'}"

# 抬起
ros2 topic pub --once /arm/gcode_cmd std_msgs/msg/String \
  "{data: 'G91 G1 Z25 F1200'}"
```

## 4. 相机 Demo


| 用途 | Topic | 消息类型 |
| --- | --- | --- |
| 原始图像 | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| 相机内参 | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

`use_arm:=true` 时，机械臂上的 mono 相机在仿真里会直接输出这两个 ROS 话题，`/camera/camera_info` 默认按零畸变处理。

默认仿真参数如下：

| 参数 | 默认值 |
| --- | --- |
| 宽度 | `1280` |
| 高度 | `720` |
| 水平视场角 HFOV | `1.5707 rad` |
| 输出 `frame_id` | `camera_optical_frame` |
| 畸变模型 | `plumb_bob` |
| 畸变系数 `D` | `[0.0, 0.0, 0.0, 0.0, 0.0]` |

如果上游没有给出可用的 `camera_info`，这里对外仍按标准 `CameraInfo` 话题处理，并保持零畸变。当前默认值对应：

```text
fx = 640.061652118157
fy = 640.061652118157
cx = 639.5
cy = 359.5
```

对应的 `CameraInfo` 关键字段可直接按下面理解：

```python
K = [
    640.061652118157, 0.0, 639.5,
    0.0, 640.061652118157, 359.5,
    0.0, 0.0, 1.0,
]

R = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
]

P = [
    640.061652118157, 0.0, 639.5, 0.0,
    0.0, 640.061652118157, 359.5, 0.0,
    0.0, 0.0, 1.0, 0.0,
]
```

如果你改了环境变量 `DUOJIN01_SIM_ARM_MONO_CAMERA_WIDTH`、`DUOJIN01_SIM_ARM_MONO_CAMERA_HEIGHT`、`DUOJIN01_SIM_ARM_MONO_CAMERA_HFOV`，或者上游 Gazebo 已经开始稳定发布有效 `camera_info`，这里的数值就不再适用，应以运行时 `/camera/camera_info` 为准。


仿真相机可以在仿真启动时打开：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py use_arm:=true
```

开发侧直接订阅 ROS 话题即可，不再走 `/dev/videoX` 这条旁路。下面是一个最小的 Python 例子，订阅 `/camera/image_raw` 并保存一帧：

```python
#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraDump(Node):
    def __init__(self):
        super().__init__("camera_dump")
        self._bridge = CvBridge()
        self._sub = self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite("/tmp/camera_demo.jpg", frame)
        self.get_logger().info("saved /tmp/camera_demo.jpg")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraDump()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

如果只想先确认话题通不通，也可以直接看：

```bash
ros2 topic echo /camera/image_raw
ros2 topic echo /camera/camera_info
```

