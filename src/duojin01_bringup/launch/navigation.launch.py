import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    workspace_nav2_params_file = os.path.abspath(
        os.path.join(bringup_share, "..", "..", "..", "..", "src", "duojin01_bringup", "config", "nav2.yaml")
    )
    default_nav2_params_file = (
        workspace_nav2_params_file
        if os.path.exists(workspace_nav2_params_file)
        else os.path.join(bringup_share, "config", "nav2.yaml")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    nav_log_level = LaunchConfiguration("nav_log_level")
    use_composition = LaunchConfiguration("use_composition")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "base.launch.py")),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_rviz": use_rviz,
            "autostart": autostart,
            "use_composition": use_composition,
            "map": map_yaml,
            "params_file": params_file,
            "log_level": nav_log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            ),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("nav_log_level", default_value="info"),
            DeclareLaunchArgument(
                "use_composition",
                default_value=EnvironmentVariable("DUOJIN01_NAV2_USE_COMPOSITION", default_value="False"),
            ),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_nav2_params_file,
            ),
            base_launch,
            nav_launch,
        ]
    )
