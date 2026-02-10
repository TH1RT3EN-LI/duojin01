import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_lidar = LaunchConfiguration("use_lidar")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")


    bringup_share = get_package_share_directory("duojin01_bringup")
    description_share = get_package_share_directory("duojin01_description")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove", "bridge.yaml")
    watchdog_share = get_package_share_directory("duojin01_safety_watchdog")
    watchdog_config_path = os.path.join(watchdog_share, "config", "safety_watchdog.yaml")
    twist_mux_config_path = os.path.join(bringup_share, "config", "twist_mux.yaml")

    urdf_file = os.path.join(description_share, "urdf", "duojin01.xacro")
    robot_description = ParameterValue(Command(["xacro", " ", urdf_file]), value_type=str)


    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        condition=IfCondition(publish_joint_states),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        condition=IfCondition(publish_robot_model),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_path,
            {"use_sim_time": use_sim_time_param},
            {"odom0": odom0},
            {"imu0": imu0},
        ],
    )


    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_foxglove),
    )
    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "joy_teleop.launch.py",
            )
        ),
        condition=IfCondition(use_lidar),
    )
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lslidar_driver"),
                "launch",
                "lslidar_launch.py",
            )
        ),
        condition=IfCondition(use_lidar),
    )
    
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[twist_mux_config_path, {"use_sim_time": use_sim_time_param}],
            remappings=[("/cmd_vel_out", "/cmd_vel_safe")],
        )

    base_driver_node = Node(
            package="duojin01_base_driver",
            executable="duojin01_base_driver_node",  
            name="base_driver",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time_param}],
            remappings=[("/cmd_vel", "/cmd_vel_safe")],
        )

    safety_watchdog_node = Node(
            package="duojin01_safety_watchdog",
            executable="safety_watchdog_node",
            name="safety_watchdog",
            output="screen",
            parameters=[
                watchdog_config_path,
                {"use_sim_time": use_sim_time_param},
            ],
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument("publish_robot_model", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            twist_mux_node,
            base_driver_node,
            safety_watchdog_node,
            joint_state_publisher,
            robot_state_publisher,
            ekf_node,
            foxglove_bridge,
            joy_teleop_launch,
            lslidar_launch,
        ]
    )
