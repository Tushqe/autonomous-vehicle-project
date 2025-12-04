import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return launch.LaunchDescription([

        # Lane Detection Node
        launch_ros.actions.Node(
            package='lane_follow',
            executable='detect',          # console script name
            name='detect_node',
            output='screen'
        ),

        # Ground Spot Node
        launch_ros.actions.Node(
            package='lane_follow',
            executable='ground_spot',     # console script name
            name='ground_spot_node',
            output='screen'
        ),

        # Pure Pursuit Node
        launch_ros.actions.Node(
            package='lane_follow',
            executable='follow_pure_pursuit',  # console script name
            name='pure_pursuit_node',
            output='screen'
        ),
    ])
