from setuptools import setup

package_name = 'perception_ml_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['perception_ml_node/launch/yolopv2_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tushig',
    maintainer_email='you@example.com',
    description='Offline YOLOPv2 perception demo node',
    license='MIT',
)

