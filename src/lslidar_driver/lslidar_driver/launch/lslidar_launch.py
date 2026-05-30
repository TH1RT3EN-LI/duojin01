#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[params_file, {"scan_topic": scan_topic}],
    )

    # 驱动自带的雷达测试，如有需要自行启动；启动方法见上级目录README.md
    # rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz', 'lslidar.rviz')

    # rviz_node = Node(
    #     package='rviz2',
    #     namespace='',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_dir],
    #     output='screen')

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=driver_dir),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        driver_node,
        # rviz_node,
    ])
