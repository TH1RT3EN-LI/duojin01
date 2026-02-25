import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
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
    return os.path.join(maps_dir, "default.yaml")


def generate_launch_description():
    bringup_share = get_package_share_directory('duojin01_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    default_map_yaml = _resolve_default_map_yaml(bringup_share)

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])

    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'log_level': log_level,
            }.items()
        )

    rviz_launch = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}],
            condition=IfCondition(use_rviz),
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'nav2.yaml')
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config
        ),
        nav2_launch,
        rviz_launch,
        initial_pose_pub,
    ])
