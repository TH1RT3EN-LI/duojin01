import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _resolve_default_map_yaml(bringup_share: str) -> str:
    maps_dir = os.path.join(bringup_share, "maps")
    numeric_maps = []
    newest_map = None
    newest_mtime = -1.0

    if os.path.isdir(maps_dir):
        for filename in os.listdir(maps_dir):
            if not filename.endswith(".yaml"):
                continue
            path = os.path.join(maps_dir, filename)
            if not os.path.isfile(path):
                continue
            stem = os.path.splitext(filename)[0]
            if stem.isdigit():
                numeric_maps.append((int(stem), path))
            mtime = os.path.getmtime(path)
            if mtime > newest_mtime:
                newest_mtime = mtime
                newest_map = path

    if numeric_maps:
        numeric_maps.sort(key=lambda item: item[0])
        return numeric_maps[-1][1]
    if newest_map is not None:
        return newest_map
    return os.path.join(maps_dir, "my_map.yaml")


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    orbbec_camera_share = get_package_share_directory("orbbec_camera")
    default_map_yaml = _resolve_default_map_yaml(bringup_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_depth_camera = LaunchConfiguration("use_depth_camera")
    depth_camera_launch_file = LaunchConfiguration("depth_camera_launch_file")
    depth_camera_name = LaunchConfiguration("depth_camera_name")
    depth_camera_serial_number = LaunchConfiguration("depth_camera_serial_number")
    depth_camera_usb_port = LaunchConfiguration("depth_camera_usb_port")
    nav_log_level = LaunchConfiguration("nav_log_level")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "base.launch.py")
        ),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "autostart": autostart,
            "map": map_yaml,
            "params_file": params_file,
            "log_level": nav_log_level,
        }.items(),
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([orbbec_camera_share, "launch", depth_camera_launch_file])
        ),
        launch_arguments={
            "camera_name": depth_camera_name,
            "serial_number": depth_camera_serial_number,
            "usb_port": depth_camera_usb_port,
            "enable_depth": "true",
            "enable_color": "false",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
            "publish_tf": "false",
            "cloud_frame_id": "camera_depth_optical_frame",
            "log_level": "info",
        }.items(),
        condition=IfCondition(use_depth_camera),
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
        condition=IfCondition(use_depth_camera),
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
        condition=IfCondition(use_depth_camera),
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
        condition=IfCondition(use_depth_camera),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument(
                "use_depth_camera",
                default_value=EnvironmentVariable("DUOJIN01_USE_DEPTH_CAMERA", default_value="false"),
            ),
            DeclareLaunchArgument(
                "depth_camera_launch_file",
                default_value=EnvironmentVariable("DUOJIN01_DEPTH_CAMERA_LAUNCH_FILE", default_value="astra_pro_plus.launch.py"),
            ),
            DeclareLaunchArgument("depth_camera_name", default_value="camera"),
            DeclareLaunchArgument("depth_camera_serial_number", default_value=""),
            DeclareLaunchArgument("depth_camera_usb_port", default_value=""),
            DeclareLaunchArgument("nav_log_level", default_value="info"),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_yaml,
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            base_launch,
            depth_camera_launch,
            camera_link_tf,
            camera_depth_frame_tf,
            camera_depth_optical_frame_tf,
            nav_launch,
        ]
    )
