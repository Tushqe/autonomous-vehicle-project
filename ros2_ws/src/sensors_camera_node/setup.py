from setuptools import setup
import os
from glob import glob

package_name = 'sensors_camera_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 camera publisher for USB/CSI cameras on Jetson Orin Nano',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = sensors_camera_node.cam_node:main',
        ],
    },
)
