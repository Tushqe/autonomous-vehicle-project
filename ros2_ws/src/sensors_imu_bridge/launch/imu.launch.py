from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    args = [
        ('mode', 'i2c'),                 # i2c | serial | gazebo | sim
        ('frame_id', 'imu_link'),
        ('rate_hz', '100.0'),
        ('i2c_bus', '1'),
        ('i2c_address', '104'),          # 0x68
        ('serial_port', '/dev/ttyUSB0'),
        ('serial_baud', '115200'),
        ('gazebo_input_topic', '/imu'),
        ('raw_topic', '/imu/data_raw'),
        ('sync_topic', '/sync/imu'),
    ]
    declares = [DeclareLaunchArgument(k, default_value=v) for k, v in args]
    params = {k: LaunchConfiguration(k) for k, _ in args}

    return LaunchDescription(declares + [
        Node(
            package='sensors_imu_bridge',
            executable='sensors_imu_bridge',
            name='sensors_imu_bridge',
            output='screen',
            parameters=[params],
        )
    ])
