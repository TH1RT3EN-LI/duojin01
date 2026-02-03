import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_pkg = get_package_share_directory('duojin01_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    foxglove_bridge_config_path = os.path.join(bringup_pkg, "config", "foxglove_bridge.yaml")

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz = LaunchConfiguration('rviz')
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart
            }.items()
        )

    rviz_launch = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}],
            condition=IfCondition(rviz),
        )

    # When RViz is disabled (e.g. headless simulation), Nav2/AMCL will not publish map->odom
    # until it receives an /initialpose. Publish a reasonable default so the "map" TF frame exists.
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
        condition=UnlessCondition(rviz),
    )
    
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
        condition=UnlessCondition(rviz),
    )    
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_pkg, 'config', 'nav2.yaml')
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_pkg, 'maps', 'my_map.yaml')
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
        ),
        nav2_launch,
        rviz_launch,
        initial_pose_pub,
        foxglove_bridge,
        
    ])
