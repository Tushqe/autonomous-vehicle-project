from setuptools import setup
package_name = 'perception_sync_node'

setup(
    name = package_name,
    version = '0.0.1',
    packages = [package_name],
    install_requires = ['setuptools'],
    zip_safe = True,
    maintainer = 'Your Name',
    maintainer_email = 'mart2520@msu.edu',
    description = 'Node that synchronizes LiDAR and IMU data for perception fusion.',
    license = 'MIT',
    entry_points = {
        'console_scripts': [
            'perception_sync_node = perception_sync_node.perception_sync_node:main',

        ],
    },
)