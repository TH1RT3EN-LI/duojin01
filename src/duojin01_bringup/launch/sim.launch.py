import os
import shutil

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
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    description_share = get_package_share_directory("duojin01_description")
    orbbec_description_share = get_package_share_directory("orbbec_description")

    world_path = os.path.join(bringup_share, "worlds", "empty_world.sdf")
    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")
    bridge_cfg = os.path.join(bringup_share, "config", "ros_gz_bridge.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove", "bridge.yaml")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])
    watchdog_share = get_package_share_directory("duojin01_safety_watchdog")
    watchdog_config_path = os.path.join(watchdog_share, "config", "safety_watchdog.yaml")

    description_share_parent = os.path.dirname(description_share)
    orbbec_description_share_parent = os.path.dirname(orbbec_description_share)
    resource_dirs = [description_share_parent, orbbec_description_share_parent, bringup_share]
    if os.path.isdir("/usr/share/gz"):
        resource_dirs.append("/usr/share/gz")
    resource_path = ":".join(resource_dirs)

    urdf_file = os.path.join(description_share, "urdf", "duojin01.xacro")

    if not shutil.which("gz"):
        raise RuntimeError(
            "Gazebo Harmonic CLI 'gz' not found in PATH. "
            "Please use the Harmonic image/environment (e.g. duojin01:humble-harmonic)."
        )
    gazebo_cmd = ["gz", "sim"]
    default_gz_partition = f"duojin01_{os.getpid()}"

    headless = LaunchConfiguration("headless")
    world_name = LaunchConfiguration("world_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    sim_profile = LaunchConfiguration("sim_profile")
    render_engine = LaunchConfiguration("render_engine")
    software_gl = LaunchConfiguration("software_gl")
    separate_gui = LaunchConfiguration("separate_gui")
    gz_partition = LaunchConfiguration("gz_partition")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    rviz_config = LaunchConfiguration("rviz_config")

    use_sim_tf = LaunchConfiguration("use_sim_tf")
    use_sim_camera = LaunchConfiguration("use_sim_camera")
    controller_port = LaunchConfiguration("controller_port")
    use_teleop = LaunchConfiguration("use_teleop")
    use_foxglove = LaunchConfiguration("use_foxglove")
    base_driver_start_delay = LaunchConfiguration("base_driver_start_delay")

    robot_description = ParameterValue(Command(["xacro", " ", urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
    )

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

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        namespace="sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param, "lazy": True, "config_file": bridge_cfg}],
    )

    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "joy_teleop.launch.py")),
        condition=IfCondition(use_teleop),
    )

    controller_emulator = Node(
        package="duojin01_controller_emulator",
        executable="duojin01_controller_emulator_node",
        name="duojin01_controller_emulator",
        output="screen",
        parameters=[
            {
                # Keep emulator on wall time so PTY feedback starts immediately
                # even before /clock is bridged at launch startup.
                "use_sim_time": False,
                "pty_link_path": controller_port,
                "odom_topic": "/sim/odom",
                "cmd_vel_out_topic": "/cmd_vel_sim",
            }
        ],
    )

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


    orbbec_topic_compat = Node(
        package="duojin01_sim_tools",
        executable="orbbec_topic_compat_node",
        name="orbbec_topic_compat",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_sim_camera),
    )

    camera_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "depth_cam",
            "camera_link",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_depth_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "camera_link",
            "camera_depth_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_color_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_color_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "camera_link",
            "camera_color_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_ir_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_ir_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "camera_link",
            "camera_ir_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_depth_optical_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "-0.5",
            "0.5",
            "-0.5",
            "0.5",
            "camera_depth_frame",
            "camera_depth_optical_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_color_optical_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_color_optical_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "-0.5",
            "0.5",
            "-0.5",
            "0.5",
            "camera_color_frame",
            "camera_color_optical_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    camera_ir_optical_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_ir_optical_frame_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "-0.5",
            "0.5",
            "-0.5",
            "0.5",
            "camera_ir_frame",
            "camera_ir_optical_frame",
        ],
        condition=IfCondition(use_sim_camera),
    )

    twist_mux_config_path = os.path.join(bringup_share, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("/cmd_vel_out", "/cmd_vel_safe")],
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
        remappings=[("/cmd_vel", "/cmd_vel_safe")],
    )
    base_driver_delayed = TimerAction(
        period=base_driver_start_delay,
        actions=[base_driver],
    )

    safety_watchdog = Node(
        package="duojin01_safety_watchdog",
        executable="safety_watchdog_node",
        name="safety_watchdog",
        output="screen",
        parameters=[
            watchdog_config_path,
            {"use_sim_time": use_sim_time_param},
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
    rviz_soft_condition = IfCondition(
        PythonExpression(['"', use_rviz, '" == "true" and "', rviz_software_gl, '" == "true"'])
    )
    rviz_hw_condition = IfCondition(
        PythonExpression(['"', use_rviz, '" == "true" and "', rviz_software_gl, '" == "false"'])
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
        additional_env={
            "LIBGL_DRI3_DISABLE": "1",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
            "QT_XCB_GL_INTEGRATION": "none",
            "QT_OPENGL": "software",
        },
        condition=rviz_soft_condition,
    )
    rviz_node_hw = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=rviz_hw_condition,
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
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
            DeclareLaunchArgument("world", default_value=world_path),
            DeclareLaunchArgument("world_name", default_value="empty_world"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("DUOJIN01_GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
            DeclareLaunchArgument(
                "use_sim_camera",
                default_value=EnvironmentVariable("DUOJIN01_SIM_CAMERA_ENABLED", default_value="false"),
            ),
            DeclareLaunchArgument(
                "sim_profile",
                default_value=EnvironmentVariable("DUOJIN01_SIM_PROFILE", default_value="gpu"),
                description="simulation profile: gpu or cpu",
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            DeclareLaunchArgument(
                "software_gl",
                default_value=PythonExpression(['"true" if "', sim_profile, '" == "cpu" else "false"']),
            ),
            DeclareLaunchArgument("separate_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            SetEnvironmentVariable("DUOJIN01_SIM_CAMERA_ENABLED", use_sim_camera),
            SetEnvironmentVariable("DUOJIN01_SIM_PROFILE", sim_profile),
            SetEnvironmentVariable("DUOJIN01_GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", rviz_software_gl),
            SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_XCB_GL_INTEGRATION", "none", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_OPENGL", "software", condition=IfCondition(software_gl)),
            DeclareLaunchArgument("use_sim_tf", default_value="false"),
            DeclareLaunchArgument(
                "controller_port",
                default_value=EnvironmentVariable("DUOJIN01_CONTROLLER_PORT", default_value="/tmp/duojin01_controller"),
            ),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("base_driver_start_delay", default_value="2.0"),
            ExecuteProcess(
                cmd=gazebo_cmd
                + [
                    "-r",
                    "-s",
                    "--headless-rendering",
                    "--render-engine-server",
                    render_engine,
                    LaunchConfiguration("world"),
                ],
                output="screen",
                condition=IfCondition(headless),
            ),
            ExecuteProcess(
                cmd=gazebo_cmd
                + ["-r", "-s", "--render-engine-server", render_engine, LaunchConfiguration("world")],
                output="screen",
                condition=IfCondition(
                    PythonExpression(['"', headless, '" == "false" and "', separate_gui, '" == "true"'])
                ),
            ),
            TimerAction(
                period=1.5,
                actions=[
                    ExecuteProcess(
                        cmd=gazebo_cmd + ["-g", "--render-engine-gui", render_engine],
                        output="screen",
                    )
                ],
                condition=IfCondition(
                    PythonExpression(['"', headless, '" == "false" and "', separate_gui, '" == "true"'])
                ),
            ),
            ExecuteProcess(
                cmd=gazebo_cmd + ["-r", "--render-engine", render_engine, LaunchConfiguration("world")],
                output="screen",
                condition=IfCondition(
                    PythonExpression(['"', headless, '" == "false" and "', separate_gui, '" == "false"'])
                ),
            ),
            robot_state_publisher,
            joint_state_stamp_fix,
            clock_guard,
            ros_gz_bridge,
            orbbec_topic_compat,
            controller_emulator,
            sim_odom_to_tf,
            camera_link_tf,
            camera_depth_frame_tf,
            camera_color_frame_tf,
            camera_ir_frame_tf,
            camera_depth_optical_frame_tf,
            camera_color_optical_frame_tf,
            camera_ir_optical_frame_tf,
            twist_mux_node,
            base_driver_delayed,
            safety_watchdog,
            ekf_node,
            joy_teleop_launch,
            spawn_robot_delayed,
            foxglove_bridge,
            rviz_node,
            rviz_node_hw,
        ]
    )
