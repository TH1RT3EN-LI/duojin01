import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.map_paths import resolve_map_yaml
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "duojin01_bringup"


def _to_python_bool_literal(value: str) -> str:
    return "True" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "False"


def _create_nav_actions(context, nav2_share: str, bringup_share: str):
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")

    use_sim_time_value = use_sim_time.perform(context)
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    params_file_value = params_file.perform(context)
    autostart_value = autostart.perform(context)
    log_level_value = log_level.perform(context)
    rviz_config_value = rviz_config.perform(context)
    use_rviz_value = use_rviz.perform(context)
    use_composition_value = use_composition.perform(context)
    use_composition_literal = _to_python_bool_literal(use_composition_value)
    lattice_candidates = [
        "duojin01_omni_2cm_rich_lattice.json",
        "duojin01_omni_2cm_lattice.json",
    ]
    lattice_filepath = ""
    for lattice_filename in lattice_candidates:
        candidate = os.path.join(
            bringup_share,
            "config",
            "lattice_primitives",
            lattice_filename,
        )
        if os.path.exists(candidate):
            lattice_filepath = candidate
            break

    try:
        map_yaml = resolve_map_yaml(LaunchConfiguration("map").perform(context), PACKAGE_NAME)
    except (FileNotFoundError, RuntimeError) as exc:
        raise RuntimeError(f"[nav2] {exc}") from exc

    if not lattice_filepath:
        raise RuntimeError(
            "[nav2] lattice file not found under "
            f"{os.path.join(bringup_share, 'config', 'lattice_primitives')}"
        )

    configured_params = RewrittenYaml(
        source_file=params_file_value,
        param_rewrites={
            "lattice_filepath": lattice_filepath,
        },
        convert_types=True,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": use_sim_time_value,
            "params_file": configured_params,
            "autostart": autostart_value,
            "use_composition": use_composition_literal,
            "log_level": log_level_value,
        }.items(),
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_value],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_rviz_value),
    )

    initial_pose_pub = Node(
        package="duojin01_sim_tools",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"frame_id": "map"},
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"delay_sec": 2.0, "publish_count": 10, "publish_period_sec": 0.2},
        ],
    )

    return [
        LogInfo(msg=f"[nav2] using map: {map_yaml}"),
        LogInfo(msg=f"[nav2] using params: {params_file_value}"),
        LogInfo(msg=f"[nav2] using lattice: {lattice_filepath}"),
        LogInfo(msg=f"[nav2] use_composition: {use_composition_literal}"),
        GroupAction(scoped=True, actions=[nav2_launch]),
        GroupAction(scoped=True, actions=[rviz_launch, initial_pose_pub]),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory(PACKAGE_NAME)
    nav2_share = get_package_share_directory("nav2_bringup")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_composition",
                default_value=EnvironmentVariable("DUOJIN01_NAV2_USE_COMPOSITION", default_value="False"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            OpaqueFunction(function=_create_nav_actions, args=[nav2_share, bringup_share]),
        ]
    )
