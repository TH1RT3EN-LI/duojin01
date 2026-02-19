import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_profile = LaunchConfiguration("sim_profile")
    use_sim_camera = LaunchConfiguration("use_sim_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])
    sim_nav_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "odom_topic": "/odom",
        },
        convert_types=True,
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py")),
        launch_arguments={
            "headless": headless,
            "sim_profile": sim_profile,
            "use_sim_camera": use_sim_camera,
            "use_sim_tf": "true",
            "use_teleop": "false",
            "use_foxglove": use_foxglove,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
            "rviz_software_gl": rviz_software_gl,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": "false",
            "map": map_yaml,
            "params_file": sim_nav_params,
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
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
            DeclareLaunchArgument("sim_profile", default_value=EnvironmentVariable("DUOJIN01_SIM_PROFILE", default_value="gpu")),
            DeclareLaunchArgument(
                "use_sim_camera",
                default_value=EnvironmentVariable("DUOJIN01_SIM_CAMERA_ENABLED", default_value="false"),
            ),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_yaml,
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
