import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_lidar = LaunchConfiguration("use_lidar")

    bringup_share = get_package_share_directory("duojin01_bringup")

    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")
    usb_cam_config_path = os.path.join(bringup_share, "config", "usb_cam.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove_bridge.yaml")
    joy_teleop_normal_config_path = os.path.join(bringup_share, "config", "joy_teleop_normal.yaml")
    joy_teleop_slow_config_path = os.path.join(bringup_share, "config", "joy_teleop_slow.yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
    )
    usb_cam_node_exe = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen",
        parameters=[usb_cam_config_path],
        condition=IfCondition(use_camera),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path],
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
        launch_arguments={"use_sim_time": "false"}.items(),
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
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=IfCondition(use_lidar),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            ekf_node,
            usb_cam_node_exe,
            foxglove_bridge,
            joy_teleop_launch,
            lslidar_launch,
        ]
    )
