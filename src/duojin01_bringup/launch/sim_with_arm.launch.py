import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression


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
                default_value=EnvironmentVariable("DUOJIN01_GZ_PARTITION", default_value=f"duojin01_arm_{os.getpid()}"),
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
            SetEnvironmentVariable("DUOJIN01_WITH_ARM", LaunchConfiguration("with_arm")),
            SetEnvironmentVariable("DUOJIN01_ARM_VARIANT", LaunchConfiguration("arm_variant")),
            SetEnvironmentVariable("DUOJIN01_ARM_MOUNT_XYZ", LaunchConfiguration("arm_mount_xyz")),
            SetEnvironmentVariable("DUOJIN01_ARM_MOUNT_RPY", LaunchConfiguration("arm_mount_rpy")),
            SetEnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", LaunchConfiguration("e4_use_low_mesh")),
            sim_launch,
        ]
    )
