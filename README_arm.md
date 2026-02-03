# 提前编译 serial_ros2 与 mirobot_description（可选）
# 源码在 src/serial_ros2、src/mirobot_description；serial_ros2 包名为 serial，mirobot_description 依赖 serial，需先装 serial
cd /home/doloris/ROS2/duojin01-arm
colcon build --packages-select serial
source install/setup.bash
colcon build --packages-select mirobot_description
source install/setup.bash

# 终端 1：编译并启动带机械臂的 bringup（底盘 + 机械臂模型 + G 代码桥接）
cd /home/doloris/ROS2/duojin01-arm
colcon build --packages-select serial mirobot_description duojin01_sim_tools lslidar_msgs lslidar_driver duojin01_bringup
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch duojin01_bringup bringup_with_arm.launch.py

# 终端 2：运行 global.py 向机械臂发送回零与运动指令（需先 source install/setup.bash）
cd /home/doloris/ROS2/duojin01-arm
source install/setup.bash
python3 global.py
