from setuptools import setup

package_name = "duojin01_controller_emulator"

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
    maintainer="litianshun",
    maintainer_email="litianshun.cn@gmail.com",
    description="通过PTY模拟 duojin01 底盘控制逻辑，实现与硬件驱动等效的仿真。",
    license='GPL-3.0-only',
    entry_points={
        "console_scripts": [
            "duojin01_controller_emulator_node = duojin01_controller_emulator.controller_emulator_node:main",
        ],
    },
)

