import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    rviz = LaunchConfiguration("rviz")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup_sim.launch.py")),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
            "use_sim_tf": "true",
            "use_teleop": use_teleop,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup_nav.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": rviz,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    nav_launch_delayed = TimerAction(period=8.0, actions=[nav_launch])
    
    scan_rewriter = Node(
        package="duojin01_sim_tools",
        executable="scan_frame_rewriter",
        name="scan_frame_rewriter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
                "output_frame_id": "laser",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # For navigation, teleop is off by default to avoid multiple /cmd_vel publishers.
            DeclareLaunchArgument("use_teleop", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(bringup_share, "maps", "my_map.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            sim_launch,
            scan_rewriter,
            nav_launch_delayed,
        ]
    )
