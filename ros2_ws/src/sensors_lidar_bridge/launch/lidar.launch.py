from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    args = [
        ('mode', 'bridge'),                 # bridge | serial | sim
        ('frame_id', 'laser_frame'),
        ('scan_topic_in', '/scan'),
        ('raw_topic', '/lidar/scan_raw'),
        ('sync_topic', '/sync/lidar'),
        ('serial_port', '/dev/ttyUSB0'),
        ('serial_baud', '115200'),
        ('sim_rate_hz', '10.0'),
        ('sim_angle_min', '-3.14159'),
        ('sim_angle_max', '3.14159'),
        ('sim_angle_increment', '0.00436'),
        ('sim_range_min', '0.15'),
        ('sim_range_max', '8.0'),
    ]
    declares = [DeclareLaunchArgument(k, default_value=v) for k, v in args]
    params = {k: LaunchConfiguration(k) for k, _ in args}

    return LaunchDescription(declares + [
        Node(
            package='sensors_lidar_bridge',
            executable='sensors_lidar_bridge',
            name='sensors_lidar_bridge',
            output='screen',
            parameters=[params],
        )
    ])
