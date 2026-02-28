import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    default_output_dir = os.path.join(bringup_share, "maps")
    map_name = LaunchConfiguration("map_name")
    output_dir = LaunchConfiguration("output_dir")
    wait_timeout = LaunchConfiguration("wait_timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")

    save_map_client = Node(
        package="duojin01_slam_tools",
        executable="save_map_client_node",
        name="save_map_client_node",
        output="screen",
        parameters=[
            {
                "map_name": map_name,
                "output_dir": output_dir,
            },
            {
                "wait_timeout": ParameterValue(wait_timeout, value_type=float),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="auto"),
            DeclareLaunchArgument("output_dir", default_value=default_output_dir),
            DeclareLaunchArgument("wait_timeout", default_value="30"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            save_map_client,
        ]
    )
