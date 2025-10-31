from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    args = [
        ('topic', '/camera/image_raw'),
        ('frame_id', 'camera_link'),
        ('width', '1280'),
        ('height', '720'),
        ('fps', '30'),
        ('device_id', '0'),
        ('use_gst', 'false'),
        ('gst_pipeline', ''),
        ('flip', 'false'),
    ]
    declares = [DeclareLaunchArgument(k, default_value=v) for k, v in args]
    params = {k: LaunchConfiguration(k) for k, _ in args}

    return LaunchDescription(declares + [
        Node(
            package='sensors_camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params],
        )
    ])
