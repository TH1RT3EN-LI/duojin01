import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup_base.launch.py")
        ),
        launch_arguments={
            "use_camera": "true",
            "use_foxglove": "true",
            "use_lidar": "true",
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup_nav.launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(bringup_share, "maps", "my_map.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            base_launch,
            nav_launch,
        ]
    )
