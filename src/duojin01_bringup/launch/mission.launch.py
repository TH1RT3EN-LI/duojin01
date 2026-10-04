from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("gcode_response_timeout", default_value="5.0"),
        Node(
            package="duojin01_mission",
            executable="mission_executor",
            name="mission_executor_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": ParameterValue(
                    LaunchConfiguration("serial_baudrate"), value_type=int),
                "gcode_response_timeout": ParameterValue(
                    LaunchConfiguration("gcode_response_timeout"), value_type=float),
            }],
        ),
    ])
