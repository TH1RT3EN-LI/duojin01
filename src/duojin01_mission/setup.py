from setuptools import find_packages, setup


package_name = "duojin01_mission"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="litianshun",
    maintainer_email="litianshun.cn@gmail.com",
    description="Simulation task helpers and mission scripts for Duojin01.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_task = scripts.sim_task:main",
        ],
    },
)
