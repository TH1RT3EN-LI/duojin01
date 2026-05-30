import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "headless": LaunchConfiguration("headless"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sim_profile": LaunchConfiguration("sim_profile"),
            "render_engine": LaunchConfiguration("render_engine"),
            "software_gl": LaunchConfiguration("software_gl"),
            "separate_gui": LaunchConfiguration("separate_gui"),
            "gz_partition": LaunchConfiguration("gz_partition"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_software_gl": LaunchConfiguration("rviz_software_gl"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "use_sim_tf": LaunchConfiguration("use_sim_tf"),
            "controller_port": LaunchConfiguration("controller_port"),
            "use_teleop": LaunchConfiguration("use_teleop"),
            "use_foxglove": LaunchConfiguration("use_foxglove"),
            "base_driver_start_delay": LaunchConfiguration("base_driver_start_delay"),
            "e4_use_low_mesh": LaunchConfiguration("e4_use_low_mesh"),
        }.items(),
    )

    sim_nodes = GroupAction(
        scoped=True,
        actions=[
            Node(
                package="duojin01_sim_tools",
                executable="e4_joint_test_bridge_node",
                name="e4_joint_test_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "command_topic": LaunchConfiguration("e4_joint_test_cmd_topic"),
                        "trajectory_topic": "/e4_joint_trajectory",
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="e4_joint_cmd_bridge",
                output="screen",
                arguments=[
                    "/e4_joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="duojin01_sim_tools",
                executable="e4_gcode_adapter_node",
                name="e4_gcode_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "command_topic": LaunchConfiguration("e4_gcode_cmd_topic"),
                        "result_topic": LaunchConfiguration("e4_gcode_result_topic"),
                        "result_mode": LaunchConfiguration("e4_gcode_result_mode"),
                        "trajectory_topic": "/e4_joint_trajectory",
                        "joint_state_topic": "/joint_states",
                        "end_effector_pose_frame": LaunchConfiguration("e4_end_effector_pose_frame"),
                        "end_effector_pose_topic": LaunchConfiguration("e4_end_effector_pose_topic"),
                        "end_effector_query_service": LaunchConfiguration("e4_end_effector_query_service"),
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=PathJoinSubstitution([bringup_share, "worlds", "test_field.sdf"])),
            DeclareLaunchArgument(
                "world_name",
                default_value=PythonExpression(['"', LaunchConfiguration("world"), '".split("/")[-1].rsplit(".", 1)[0]']),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("DUOJIN01_GZ_PARTITION", default_value=f"duojin01_e4_kinematic_{os.getpid()}"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
            DeclareLaunchArgument(
                "sim_profile",
                default_value=EnvironmentVariable("DUOJIN01_SIM_PROFILE", default_value="gpu"),
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            DeclareLaunchArgument(
                "software_gl",
                default_value=PythonExpression(['"true" if "', LaunchConfiguration("sim_profile"), '" == "cpu" else "false"']),
            ),
            DeclareLaunchArgument("separate_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"]),
            ),
            DeclareLaunchArgument("use_sim_tf", default_value="false"),
            DeclareLaunchArgument(
                "controller_port",
                default_value=EnvironmentVariable(
                    "DUOJIN01_CONTROLLER_PORT",
                    default_value=f"/tmp/duojin01_controller_{os.getuid()}",
                ),
            ),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("base_driver_start_delay", default_value="6.0"),
            DeclareLaunchArgument(
                "e4_use_low_mesh",
                default_value=EnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", default_value="true"),
            ),
            DeclareLaunchArgument("with_arm", default_value="true"),
            DeclareLaunchArgument(
                "arm_variant",
                default_value=EnvironmentVariable("DUOJIN01_ARM_VARIANT", default_value="e4_kinematic"),
            ),
            DeclareLaunchArgument(
                "arm_mount_xyz",
                default_value=EnvironmentVariable("DUOJIN01_ARM_MOUNT_XYZ", default_value="-0.13 -0.04 0.155"),
            ),
            DeclareLaunchArgument(
                "arm_mount_rpy",
                default_value=EnvironmentVariable("DUOJIN01_ARM_MOUNT_RPY", default_value="1.5708 0 0"),
            ),
            DeclareLaunchArgument("e4_joint_test_cmd_topic", default_value="/e4_joint_test_cmd"),
            DeclareLaunchArgument("e4_gcode_cmd_topic", default_value="/arm/gcode_cmd"),
            DeclareLaunchArgument("e4_gcode_result_topic", default_value="/arm/gcode_result"),
            DeclareLaunchArgument("e4_gcode_result_mode", default_value="auto"),
            DeclareLaunchArgument("e4_end_effector_pose_frame", default_value="arm_gcode_frame"),
            DeclareLaunchArgument("e4_end_effector_pose_topic", default_value="/arm/end_effector_pose"),
            DeclareLaunchArgument("e4_end_effector_query_service", default_value="/arm/query_end_effector_pose"),
            SetEnvironmentVariable("DUOJIN01_WITH_ARM", LaunchConfiguration("with_arm")),
            SetEnvironmentVariable("DUOJIN01_ARM_VARIANT", LaunchConfiguration("arm_variant")),
            SetEnvironmentVariable("DUOJIN01_ARM_MOUNT_XYZ", LaunchConfiguration("arm_mount_xyz")),
            SetEnvironmentVariable("DUOJIN01_ARM_MOUNT_RPY", LaunchConfiguration("arm_mount_rpy")),
            SetEnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", LaunchConfiguration("e4_use_low_mesh")),
            LogInfo(msg=["E4 joint test topic: ", LaunchConfiguration("e4_joint_test_cmd_topic")]),
            LogInfo(msg=["E4 G-code mirror topic: ", LaunchConfiguration("e4_gcode_cmd_topic")]),
            LogInfo(msg=["E4 end-effector pose topic: ", LaunchConfiguration("e4_end_effector_pose_topic")]),
            LogInfo(msg=["E4 end-effector query service: ", LaunchConfiguration("e4_end_effector_query_service")]),
            LogInfo(msg=["E4 one-shot CLI: ros2 run duojin01_sim_tools e4_joint_test_cli --deg 0 30 -45 15"]),
            LogInfo(msg=["E4 single joint CLI: ros2 run duojin01_sim_tools e4_joint_test_cli --joint 1 --deg 30"]),
            LogInfo(
                msg=[
                    "E4 raw single-joint pub: ros2 topic pub --once /e4_joint1_cmd std_msgs/msg/Float64 "
                    '"{data: 0.52}"',
                ]
            ),
            sim_launch,
            sim_nodes,
        ]
    )
