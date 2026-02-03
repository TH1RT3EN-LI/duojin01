import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")

    joy_teleop_normal_config_path = os.path.join(bringup_share, "config", "joy_teleop_normal.yaml")
    joy_teleop_slow_config_path = os.path.join(bringup_share, "config", "joy_teleop_slow.yaml")
    joy_teleop_estop_config_path = os.path.join(bringup_share, "config", "joy_teleop_estop.yaml")
    twist_config_path = os.path.join(bringup_share, "config", "twist_mux.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "device_id": 0,
                "deadzone": 0.12,
                "autorepeat_rate": 20.0,
            }
        ],
    )
    joy_teleop_slow = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_slow",
        parameters=[joy_teleop_slow_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("/cmd_vel", "/cmd_vel_slow")],
    )

    joy_teleop_normal = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_normal",
        parameters=[joy_teleop_normal_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("/cmd_vel", "/cmd_vel_normal")],
    )

    joy_teleop_estop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_estop",
        parameters=[joy_teleop_estop_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("/cmd_vel", "/cmd_vel_estop")],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            joy,
            joy_teleop_slow,
            joy_teleop_normal,
            joy_teleop_estop,
            twist_mux_node,
        ]
    )
