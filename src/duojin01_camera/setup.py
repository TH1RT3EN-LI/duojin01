from setuptools import find_packages, setup

package_name = 'duojin01_camera'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'usb_camera_node = duojin01_camera.usb_camera_node:main',
            'calibrate_camera = duojin01_camera.calibrate_camera:main',
        ],
    },
)
