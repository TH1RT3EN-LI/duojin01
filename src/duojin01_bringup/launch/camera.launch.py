from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="3"),
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("frame_id", default_value="camera_optical_frame"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        Node(
            package="duojin01_camera",
            executable="usb_camera_node",
            name="usb_camera_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                **{key: ParameterValue(LaunchConfiguration(key), value_type=int)
                   for key in ("device_id", "width", "height", "fps")},
                "frame_id": LaunchConfiguration("frame_id"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }],
        ),
    ])
