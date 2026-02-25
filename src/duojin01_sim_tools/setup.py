from setuptools import setup


package_name = "duojin01_sim_tools"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="litianshun.cn@gmail.com",
    maintainer_email="litianshun.cn@gmail.com",
    description="用于实现仿真的数个小工具node。",
    license='GPL-3.0-only',
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_stamp_fix_node = duojin01_sim_tools.joint_state_stamp_fix_node:main",
            "scan_frame_rewriter = duojin01_sim_tools.scan_frame_rewriter:main",
            "odom_to_tf_node = duojin01_sim_tools.odom_to_tf_node:main",
            "initial_pose_publisher = duojin01_sim_tools.initial_pose_publisher:main",
            "clock_guard_node = duojin01_sim_tools.clock_guard_node:main",
            "orbbec_topic_compat_node = duojin01_sim_tools.orbbec_topic_compat_node:main",
        ],
    },
)
