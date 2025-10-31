from setuptools import setup
import os
from glob import glob

package_name = 'sensors_imu_bridge'

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
    description='ROS2 IMU bridge for I2C/Serial/Gazebo/Sim → /imu/data_raw and /sync/imu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensors_imu_bridge = sensors_imu_bridge.imu_bridge:main',
        ],
    },
)
