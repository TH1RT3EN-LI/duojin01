import os
from glob import glob
from setuptools import setup

package_name = 'duojin01_safety_watchdog'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='安全',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'safety_watchdog_node = duojin01_safety_watchdog.safety_watchdog_node:main',
        ],
    },
)
