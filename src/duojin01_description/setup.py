from setuptools import setup
import os

package_name = 'duojin01_description'


def collect_data_files(relative_dir):
    data_files = []

    for root, _, files in os.walk(relative_dir):
        if not files:
            continue

        install_dir = os.path.join('share', package_name, root)
        file_paths = [os.path.join(root, filename) for filename in sorted(files)]
        data_files.append((install_dir, file_paths))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + collect_data_files('urdf') + collect_data_files('meshes') + collect_data_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='duojin01 robot description',
    license='GPL-3.0-only',
    entry_points={'console_scripts': []},
)
