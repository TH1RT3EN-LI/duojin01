import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory("duojin01_bringup")
    pkg_description = get_package_share_directory("duojin01_description")

    world_path = os.path.join(pkg_bringup, "worlds", "empty_world.sdf")
    ekf_config_path = os.path.join(pkg_bringup, "config", "ekf.yaml")
    bridge_cfg = os.path.join(pkg_bringup, "config", "ros_gz_bridge.yaml")
    foxglove_bridge_config_path = os.path.join(pkg_bringup, "config", "foxglove_bridge.yaml")

    # Needed so Gazebo can resolve model://duojin01_description when converting URDF->SDF.
    description_share_parent = os.path.dirname(pkg_description)

    urdf_file = os.path.join(pkg_description, "urdf", "duojin01.xacro")

    headless = LaunchConfiguration("headless")
    world_name = LaunchConfiguration("world_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    use_sim_tf = LaunchConfiguration("use_sim_tf")
    controller_port = LaunchConfiguration("controller_port")
    use_teleop = LaunchConfiguration("use_teleop")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    localhost_only = LaunchConfiguration("localhost_only")
    use_foxglove = LaunchConfiguration("use_foxglove")

    robot_description = ParameterValue(Command(["xacro", " ", urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
    )

    # Fix up joint_state stamps before robot_state_publisher turns them into TF.
    # Out-of-order JointState stamps can cause TF time to go backwards, which clears tf2 buffers
    # and breaks SLAM (seen as pose flicker in visualization tools).
    joint_state_stamp_fix = Node(
        package="duojin01_sim_tools",
        executable="joint_state_stamp_fix_node",
        name="joint_state_stamp_fix",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_topic": "/joint_states_raw",
                "output_topic": "/joint_states",
            }
        ],
    )

    # Sanitize /clock to avoid out-of-order delivery from the simulator bridge.
    clock_guard = Node(
        package="duojin01_sim_tools",
        executable="clock_guard_node",
        name="duojin01_clock_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": "/clock_raw",
                "output_topic": "/clock",
            }
        ],
    )

    # Bridge Gazebo topics to ROS topics used by the sim graph.
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        namespace="sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param, "lazy": True, "config_file": bridge_cfg}],
    )

    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, "launch", "joy_teleop.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_teleop),
    )

    controller_emulator = Node(
        package="duojin01_controller_emulator",
        executable="duojin01_controller_emulator_node",
        name="duojin01_controller_emulator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "pty_link_path": controller_port,
                "odom_topic": "/sim/odom",
                "cmd_vel_out_topic": "/cmd_vel_sim",
            }
        ],
    )

    # Publish TF odom->base_footprint directly from simulator odometry when requested.
    sim_odom_to_tf = Node(
        package="duojin01_sim_tools",
        executable="odom_to_tf_node",
        name="sim_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_odom_topic": "/sim/odom",
                "odom_frame_id": "odom",
                "child_frame_id": "base_footprint",
            }
        ],
        condition=IfCondition(use_sim_tf),
    )

    base_driver = Node(
        package="duojin01_base_driver",
        executable="duojin01_base_driver_node",
        name="duojin01_base_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "usart_port_name": controller_port,
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time_param}],
        condition=UnlessCondition(use_sim_tf),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_foxglove),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[
            "-world",
            world_name,
            "-name",
            "duojin01",
            "-param",
            "robot_description",
            "-z",
            "0.0",
        ],
    )
    spawn_robot_delayed = TimerAction(period=5.0, actions=[spawn_robot])

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=":".join([description_share_parent, pkg_bringup, "/usr/share/ignition"]),
            ),
            DeclareLaunchArgument("world", default_value=world_path),
            DeclareLaunchArgument("world_name", default_value="empty_world"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_sim_tf", default_value="false"),
            DeclareLaunchArgument("controller_port", default_value="/tmp/duojin01_controller"),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("ros_domain_id", default_value="13"),
            DeclareLaunchArgument("localhost_only", default_value="0"),
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=ros_domain_id),
            SetEnvironmentVariable(name="ROS_LOCALHOST_ONLY", value=localhost_only),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", "-s", "--headless-rendering", LaunchConfiguration("world")],
                output="screen",
                condition=IfCondition(headless),
            ),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
                output="screen",
                condition=UnlessCondition(headless),
            ),
            robot_state_publisher,
            joint_state_stamp_fix,
            clock_guard,
            ros_gz_bridge,
            controller_emulator,
            sim_odom_to_tf,
            base_driver,
            ekf_node,
            joy_teleop_launch,
            spawn_robot_delayed,
            foxglove_bridge,
        ]
    )
