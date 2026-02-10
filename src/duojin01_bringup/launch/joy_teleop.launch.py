import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    teleop_share = get_package_share_directory("duojin01_teleop")
    
    joy_launcher_config_path = os.path.join(teleop_share, "config", "joy_launcher.yaml")
    joy_axis_selector_config_path = os.path.join(teleop_share, "config", "joy_axis_selector.yaml")
    joy_teleop_normal_config_path = os.path.join(bringup_share, "config", "joy_teleop_normal.yaml")
    joy_teleop_slow_config_path = os.path.join(bringup_share, "config", "joy_teleop_slow.yaml")
    joy_teleop_estop_config_path = os.path.join(bringup_share, "config", "joy_teleop_estop.yaml")

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
        remappings=[("/joy", "/joy_raw")],  
    )
    
    joy_axis_selector = Node(
        package="duojin01_teleop",
        executable="joy_axis_selector_node",
        name="joy_axis_selector_node",
        parameters=[joy_axis_selector_config_path, {"use_sim_time": use_sim_time_param}],
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

    joy_launcher = Node(
        package="duojin01_teleop",
        executable="joy_launcher_node",
        name="joy_launcher_node",
        parameters=[joy_launcher_config_path, {"use_sim_time": use_sim_time_param}],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            joy,
            joy_axis_selector,
            joy_teleop_slow,
            joy_teleop_normal,
            joy_teleop_estop,
            joy_launcher,
        ]
    )
