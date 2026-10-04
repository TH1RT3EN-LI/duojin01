import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "mapping.rviz"])
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")

    use_foxglove = LaunchConfiguration("use_foxglove")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "base.launch.py")
        ),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_teleop": LaunchConfiguration("use_teleop"),
            "base_serial_port": LaunchConfiguration("base_serial_port"),
            "base_serial_baudrate": LaunchConfiguration("base_serial_baudrate"),
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "slam.launch.py")
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("base_serial_port", default_value="/dev/duojin01_controller"),
            DeclareLaunchArgument("base_serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            base_launch,
            slam_launch,
            rviz_node,
        ]
    )
