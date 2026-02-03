import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory("mirobot_description")
    lib_path = os.path.join(os.path.dirname(pkg_path), "..", "lib", "mirobot_description")
    ensure_perms = os.path.join(os.path.normpath(lib_path), "ensure_serial_permissions.py")
    rviz_config_file = os.path.join(pkg_path, "rviz", "description.rviz")
    urdf_file = os.path.join(pkg_path, "urdf", "mirobot_urdf_2.urdf")
    serial_config = os.path.join(pkg_path, "config", "rviz_params.yaml")

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
    )

    # Launch RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    serial_node = Node(
        package="mirobot_description",
        executable="mirobot_gcode_writer",
        name="mirobot_write_node",
        output="screen",
        arguments=["-d", serial_config],
    )

    # 红色「正常退出」按钮窗口，点击后关闭串口并结束 launch
    normal_exit_button = Node(
        package="mirobot_description",
        executable="normal_exit_button.py",
        name="normal_exit_button_node",
        output="screen",
    )

    # 启动前检查串口权限，无写权限时尝试 pkexec chmod 或提示安装 udev / 加入 dialout
    ensure_serial_permissions = ExecuteProcess(
        cmd=["python3", ensure_perms],
        shell=False,
        output="screen",
    )

    return LaunchDescription([
        ensure_serial_permissions,
        TimerAction(
            period=3.0,
            actions=[rviz2]
        ),
        robot_state_publisher,
        joint_state_publisher_gui,
        serial_node,
        normal_exit_button,
    ])