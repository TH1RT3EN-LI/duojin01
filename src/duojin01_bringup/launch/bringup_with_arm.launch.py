"""
带机械臂的整车启动：底盘 bringup_base + mirobot 模型与 arm_gcode 桥接。
运行后可在另一终端执行: python3 global.py 向话题 arm_gcode_command 发送 G 代码控制机械臂。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_urdf(urdf_path):
    with open(urdf_path, "r") as f:
        return f.read()


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    mirobot_share = get_package_share_directory("mirobot_description")

    urdf_path = os.path.join(mirobot_share, "urdf", "mirobot_urdf_2.urdf")
    arm_bridge_config = os.path.join(bringup_share, "config", "arm_gcode_bridge.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # 1) 底盘基础 bringup
    bringup_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup_base.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 2) 机械臂模型：robot_state_publisher（不启动 joint_state_publisher_gui，由真实机械臂/串口反馈）
    robot_description = load_urdf(urdf_path)
    arm_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_arm",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    # 3) G 代码桥接：订阅 arm_gcode_command，写入机械臂串口
    arm_gcode_bridge = Node(
        package="duojin01_bringup",
        executable="arm_gcode_bridge",
        name="arm_gcode_bridge",
        output="screen",
        parameters=[arm_bridge_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
        bringup_base,
        arm_robot_state_publisher,
        arm_gcode_bridge,
    ])
