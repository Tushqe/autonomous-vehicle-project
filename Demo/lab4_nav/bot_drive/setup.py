from setuptools import find_packages, setup

package_name = 'bot_drive'

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
    maintainer='wangnat1',
    maintainer_email='wangnat1@msu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_drive = bot_drive.circle_drive:main',
            'bot_monitor = bot_drive.bot_monitor:main',
            'waypoint_nav = bot_drive.waypoint_nav:main',
            'square_drive = bot_drive.square_drive:main',
        ],
    },
)
