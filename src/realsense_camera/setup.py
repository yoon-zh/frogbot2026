from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'realsense_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='RealSense Camera Driver and Glass Obstacle Detection for Nav2 Costmap',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'glass_detection_node = realsense_camera.glass_detection_node:main',
            'realsense_driver_node = realsense_camera.realsense_node:main',
        ],
    },
)

