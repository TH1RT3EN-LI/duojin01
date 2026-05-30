import os
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _create_bridge_node(context, *, bridge_cfg_template_path, use_sim_time):
    world_name = LaunchConfiguration("world_name").perform(context)
    safe_world_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in world_name)
    # Keep per-UID bridge temp dirs so a previous root-launched session does not
    # block regular-user launches with a root-owned /tmp/duojin01_bridge folder.
    bridge_cfg_dir = os.path.join(tempfile.gettempdir(), f"duojin01_bridge_{os.getuid()}")
    bridge_cfg_path = os.path.join(bridge_cfg_dir, f"ros_gz_bridge_{safe_world_name}_{os.getpid()}.yaml")

    os.makedirs(bridge_cfg_dir, exist_ok=True)

    with open(bridge_cfg_template_path, "r", encoding="utf-8") as template_file:
        bridge_cfg_contents = template_file.read().replace("__WORLD_NAME__", world_name)

    with open(bridge_cfg_path, "w", encoding="utf-8") as bridge_cfg_file:
        bridge_cfg_file.write(bridge_cfg_contents)

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            namespace="sim_bridge",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "lazy": True,
                    "config_file": bridge_cfg_path,
                }
            ],
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    description_share = get_package_share_directory("duojin01_description")
    gz_plugins_prefix = get_package_prefix("duojin01_gz_plugins")

    world_path = os.path.join(bringup_share, "worlds", "race_track.sdf")
    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")
    bridge_cfg_template_path = os.path.join(bringup_share, "config", "ros_gz_bridge.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove", "bridge.yaml")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])

    description_share_parent = os.path.dirname(description_share)
    resource_dirs = [
        description_share_parent,
        description_share,
        os.path.join(description_share, "models"),
        bringup_share,
    ]
    if os.path.isdir("/usr/share/gz"):
        resource_dirs.append("/usr/share/gz")
    resource_path = ":".join(resource_dirs)
    system_plugin_path = ":".join(
        filter(
            None,
            [
                os.path.join(gz_plugins_prefix, "lib"),
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ],
        )
    )

    urdf_file = os.path.join(description_share, "urdf", "duojin01_sim.xacro")

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
    use_sim_base_driver = LaunchConfiguration("use_sim_base_driver")
    controller_port = LaunchConfiguration("controller_port")
    use_teleop = LaunchConfiguration("use_teleop")
    use_foxglove = LaunchConfiguration("use_foxglove")
    base_driver_start_delay = LaunchConfiguration("base_driver_start_delay")
    use_arm = LaunchConfiguration("use_arm")
    e4_use_low_mesh = LaunchConfiguration("e4_use_low_mesh")
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

    ros_gz_bridge = OpaqueFunction(
        function=lambda context: _create_bridge_node(
            context,
            bridge_cfg_template_path=bridge_cfg_template_path,
            use_sim_time=use_sim_time,
        )
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
                "use_sim_time": False,
                "pty_link_path": controller_port,
                "odom_topic": "/sim/odom",
                "cmd_vel_out_topic": "/cmd_vel_sim",
            }
        ],
        condition=IfCondition(use_sim_base_driver),
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


    sim_camera_topic_compat = Node(
        package="duojin01_sim_tools",
        executable="sim_camera_topic_compat_node",
        name="sim_camera_topic_compat",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
    )

    sim_mono_camera_compat = Node(
        package="duojin01_sim_tools",
        executable="sim_mono_camera_compat_node",
        name="sim_mono_camera_compat",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_arm),
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
    )

    twist_mux_config_path = os.path.join(bringup_share, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[
            (
                "/cmd_vel_out",
                PythonExpression(
                    ['"/cmd_vel_safe" if "', use_sim_base_driver, '" == "true" else "/cmd_vel_sim"']
                ),
            )
        ],
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
        condition=IfCondition(use_sim_base_driver),
    )
    base_driver_delayed = TimerAction(
        period=base_driver_start_delay,
        actions=[base_driver],
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

    sim_actions = GroupAction(
        scoped=True,
        actions=[
            clock_guard,
            ros_gz_bridge,
            sim_camera_topic_compat,
            sim_mono_camera_compat,
            sim_odom_to_tf,
            joint_state_stamp_fix,
            camera_link_tf,
            camera_depth_frame_tf,
            camera_color_frame_tf,
            camera_ir_frame_tf,
            camera_depth_optical_frame_tf,
            camera_color_optical_frame_tf,
            camera_ir_optical_frame_tf,
        ],
    )

    sim_driver_actions = GroupAction(scoped=True, actions=[controller_emulator])

    base_actions = GroupAction(scoped=True, actions=[robot_state_publisher, ekf_node])

    mux_actions = GroupAction(scoped=True, actions=[twist_mux_node])

    base_driver_actions = GroupAction(scoped=True, actions=[base_driver_delayed])

    client_actions = GroupAction(scoped=True, actions=[foxglove_bridge, rviz_node, rviz_node_hw])

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
            SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=system_plugin_path),
            DeclareLaunchArgument("world", default_value=world_path),
            DeclareLaunchArgument(
                "world_name",
                default_value=PythonExpression(['"', LaunchConfiguration("world"), '".split("/")[-1].rsplit(".", 1)[0]']),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("DUOJIN01_GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
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
            DeclareLaunchArgument(
                "use_arm",
                default_value=EnvironmentVariable("DUOJIN01_WITH_ARM", default_value="false"),
            ),
            DeclareLaunchArgument(
                "e4_use_low_mesh",
                default_value=EnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", default_value="true"),
            ),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            SetEnvironmentVariable("DUOJIN01_SIM_CAMERA_ENABLED", "true"),
            SetEnvironmentVariable("DUOJIN01_SIM_PROFILE", sim_profile),
            SetEnvironmentVariable("DUOJIN01_GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("DUOJIN01_WITH_ARM", use_arm),
            SetEnvironmentVariable("DUOJIN01_E4_USE_LOW_MESH", e4_use_low_mesh),
            SetEnvironmentVariable("DUOJIN01_RVIZ_SOFTWARE_GL", rviz_software_gl),
            SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_XCB_GL_INTEGRATION", "none", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_OPENGL", "software", condition=IfCondition(software_gl)),
            DeclareLaunchArgument("use_sim_tf", default_value="false"),
            DeclareLaunchArgument(
                "use_sim_base_driver",
                default_value=EnvironmentVariable("DUOJIN01_SIM_USE_BASE_DRIVER", default_value="true"),
            ),
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
            base_actions,
            mux_actions,
            sim_driver_actions,
            base_driver_actions,
            joy_teleop_launch,
            client_actions,
            sim_actions,
            spawn_robot_delayed,
        ]
    )
