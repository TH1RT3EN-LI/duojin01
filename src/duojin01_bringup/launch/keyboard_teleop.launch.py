import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_share = get_package_share_directory("duojin01_teleop")
    use_sim_time = LaunchConfiguration("use_sim_time")
    keyboard_backend = LaunchConfiguration("keyboard_backend")
    tty_device_path = LaunchConfiguration("tty_device_path")
    event_device_path = LaunchConfiguration("event_device_path")
    save_map_command = LaunchConfiguration("save_map_command")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    keyboard_teleop_config_path = os.path.join(teleop_share, "config", "keyboard_teleop.yaml")

    keyboard_teleop_node = Node(
        package="duojin01_teleop",
        executable="keyboard_teleop_node",
        name="keyboard_teleop_node",
        output="screen",
        parameters=[
            keyboard_teleop_config_path,
            {
                "cmd_vel_topic": cmd_vel_topic,
                "keyboard_backend": keyboard_backend,
                "tty_device_path": tty_device_path,
                "event_device_path": event_device_path,
                "save_map_command": save_map_command,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            DeclareLaunchArgument("keyboard_backend", default_value="tty"),
            DeclareLaunchArgument("tty_device_path", default_value=""),
            DeclareLaunchArgument("event_device_path", default_value=""),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_keyboard"),
            DeclareLaunchArgument("save_map_command", default_value="ros2 launch duojin01_bringup save_map.launch.py"),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            keyboard_teleop_node,
        ]
    )
