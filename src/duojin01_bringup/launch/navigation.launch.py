import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _resolve_default_map_yaml(bringup_share: str) -> str:
    maps_dir = os.path.join(bringup_share, "maps")
    numeric_maps = []
    newest_map = None
    newest_mtime = -1.0

    if os.path.isdir(maps_dir):
        for filename in os.listdir(maps_dir):
            if not filename.endswith(".yaml"):
                continue
            path = os.path.join(maps_dir, filename)
            if not os.path.isfile(path):
                continue
            stem = os.path.splitext(filename)[0]
            if stem.isdigit():
                numeric_maps.append((int(stem), path))
            mtime = os.path.getmtime(path)
            if mtime > newest_mtime:
                newest_mtime = mtime
                newest_map = path

    if numeric_maps:
        numeric_maps.sort(key=lambda item: item[0])
        return numeric_maps[-1][1]
    if newest_map is not None:
        return newest_map
    return os.path.join(maps_dir, "my_map.yaml")


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    default_map_yaml = _resolve_default_map_yaml(bringup_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "base.launch.py")
        ),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "autostart": autostart,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_yaml,
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            base_launch,
            nav_launch,
        ]
    )
