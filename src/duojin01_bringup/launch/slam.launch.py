import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_config_path = os.path.join(
        get_package_share_directory("duojin01_bringup"),
        "config",
        "slam_toolbox.yaml",
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )


    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    "slam_params_file": slam_config_path,
                    "use_sim_time": "false",
                }.items(),
            ),
        ]
    )
