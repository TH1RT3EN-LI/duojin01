import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove_bridge.yaml")

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    # Keep these overridable at the top-level so sim_mapping can run isolated from other ROS graphs.
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    localhost_only = LaunchConfiguration("localhost_only")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup_sim.launch.py")),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
            # In mapping mode, prefer Gazebo ground-truth TF to avoid EKF/odom drift and TF flicker.
            "use_sim_tf": "true",
            "use_teleop": "true",
            "ros_domain_id": ros_domain_id,
            "localhost_only": localhost_only,
        }.items(),
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup_slam.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
    )
    
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
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("ros_domain_id", default_value="13"),
            DeclareLaunchArgument("localhost_only", default_value="0"),
            sim_launch,
            slam_launch,
            scan_rewriter,
            foxglove_bridge,
        ]
    )
