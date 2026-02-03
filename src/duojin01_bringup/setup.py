from setuptools import setup, find_packages
import os
from glob import glob
package_name = 'duojin01_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),

        # maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='启动车辆的有关软硬件',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'arm_gcode_bridge = duojin01_bringup.arm_gcode_bridge:main',
        ],
    },
)
