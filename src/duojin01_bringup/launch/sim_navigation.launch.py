import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.sim_default_map import resolve_sim_navigation_map
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_map_action(context):
    resolved_map = resolve_sim_navigation_map(
        map_value=LaunchConfiguration("map").perform(context),
        world_value=LaunchConfiguration("world").perform(context),
    )
    return [SetLaunchConfiguration("map", resolved_map)]


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    description_share = get_package_share_directory("duojin01_description")

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_profile = LaunchConfiguration("sim_profile")
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_composition = LaunchConfiguration("use_composition")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    use_arm = LaunchConfiguration("use_arm")
    e4_use_low_mesh = LaunchConfiguration("e4_use_low_mesh")
    arm_variant = LaunchConfiguration("arm_variant")
    e4_gcode_cmd_topic = LaunchConfiguration("e4_gcode_cmd_topic")
    e4_gcode_result_topic = LaunchConfiguration("e4_gcode_result_topic")
    e4_gcode_result_mode = LaunchConfiguration("e4_gcode_result_mode")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    rviz_urdf_file = os.path.join(description_share, "urdf", "duojin01_sim.xacro")
    rviz_robot_description = ParameterValue(
        Command(
            [
                "env",
                " ",
                "DUOJIN01_WITH_ARM=false",
                " ",
                "xacro",
                " ",
                rviz_urdf_file,
            ]
        ),
        value_type=str,
    )
    e4_arm_condition = IfCondition(
        PythonExpression(
            [
                '("',
                use_arm,
                '" == "true" or "',
                use_arm,
                '" == "1") and "',
                arm_variant,
                '" == "e4_kinematic"',
            ]
        )
    )
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "sim_navigation.rviz"])
    sim_nav_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "odom_topic": "/sim/odom",
        },
        convert_types=True,
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py")),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "headless": headless,
            "sim_profile": sim_profile,
            "use_sim_tf": "true",
            "use_sim_base_driver": "false",
            "use_teleop": "false",
            "use_foxglove": use_foxglove,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
            "rviz_software_gl": rviz_software_gl,
            "use_arm": use_arm,
            "e4_use_low_mesh": e4_use_low_mesh,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": "false",
            "use_composition": use_composition,
            "map": map_yaml,
            "params_file": sim_nav_params,
        }.items(),
    )

    nav_launch_delayed = TimerAction(period=8.0, actions=[nav_launch])

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

    e4_joint_cmd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="e4_joint_cmd_bridge",
        output="screen",
        arguments=[
            "/e4_joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
        ],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=e4_arm_condition,
    )

    e4_gcode_adapter = Node(
        package="duojin01_sim_tools",
        executable="e4_gcode_adapter_node",
        name="e4_gcode_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "command_topic": e4_gcode_cmd_topic,
                "result_topic": e4_gcode_result_topic,
                "result_mode": e4_gcode_result_mode,
                "trajectory_topic": "/e4_joint_trajectory",
                "joint_state_topic": "/joint_states",
                "end_effector_pose_frame": LaunchConfiguration("e4_end_effector_pose_frame"),
                "end_effector_pose_topic": LaunchConfiguration("e4_end_effector_pose_topic"),
                "end_effector_query_service": LaunchConfiguration("e4_end_effector_query_service"),
            }
        ],
        condition=e4_arm_condition,
    )

    rviz_model_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="rviz_model",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": rviz_robot_description, "use_sim_time": use_sim_time_param}],
        remappings=[
            ("joint_states", "/joint_states"),
            ("/tf", "/rviz_model_tf"),
            ("/tf_static", "/rviz_model_tf_static"),
        ],
        condition=IfCondition(use_rviz),
    )

    sim_nodes = GroupAction(
        scoped=True,
        actions=[
            scan_rewriter,
            e4_joint_cmd_bridge,
            e4_gcode_adapter,
            rviz_model_state_publisher,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([bringup_share, "worlds", "race_track.sdf"]),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value=PythonExpression(['"', LaunchConfiguration("world"), '".split("/")[-1].rsplit(".", 1)[0]']),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true"),
            ),
            DeclareLaunchArgument(
                "sim_profile",
                default_value=EnvironmentVariable("DUOJIN01_SIM_PROFILE", default_value="gpu"),
            ),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument(
                "use_arm",
                default_value=EnvironmentVariable("DUOJIN01_WITH_ARM", default_value="false"),
            ),
            DeclareLaunchArgument(
                "e4_use_low_mesh",
                default_value=EnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", default_value="true"),
            ),
            DeclareLaunchArgument(
                "arm_variant",
                default_value=EnvironmentVariable("DUOJIN01_ARM_VARIANT", default_value="e4_kinematic"),
            ),
            DeclareLaunchArgument("e4_gcode_cmd_topic", default_value="/arm/gcode_cmd"),
            DeclareLaunchArgument("e4_gcode_result_topic", default_value="/arm/gcode_result"),
            DeclareLaunchArgument("e4_gcode_result_mode", default_value="always"),
            DeclareLaunchArgument("e4_end_effector_pose_frame", default_value="arm_gcode_frame"),
            DeclareLaunchArgument("e4_end_effector_pose_topic", default_value="/arm/end_effector_pose"),
            DeclareLaunchArgument("e4_end_effector_query_service", default_value="/arm/query_end_effector_pose"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            SetEnvironmentVariable("DUOJIN01_ARM_VARIANT", arm_variant),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            OpaqueFunction(function=_resolve_map_action),
            sim_launch,
            sim_nodes,
            nav_launch_delayed,
        ]
    )
