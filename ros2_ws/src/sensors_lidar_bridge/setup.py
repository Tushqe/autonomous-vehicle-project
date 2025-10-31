from setuptools import setup
import os
from glob import glob

package_name = 'sensors_lidar_bridge'

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
    description='ROS2 LiDAR bridge (bridge/serial/sim) → /lidar/scan_raw and /sync/lidar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensors_lidar_bridge = sensors_lidar_bridge.sensors_lidar_bridge:main',
        ],
    },
)
