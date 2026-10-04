from setuptools import find_packages, setup

package_name = 'duojin01_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shuo',
    maintainer_email='14346918+shuonanana@user.noreply.gitee.com',
    description='Hardware mission API using Nav2, USB camera images and serial G-code.',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'demo = scripts.demo:main',
            'grab_calibration_demo = scripts.grab_calibration_demo:main',
            'nav_demo = scripts.nav_demo:main',
            'pick_demo = scripts.pick_demo:main',
            'place_demo = scripts.place_demo:main',
            'mission_executor = duojin01_mission.mission_executor_node:main',
        ],
    },
)
