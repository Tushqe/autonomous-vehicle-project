#!/usr/bin/env python
'''
    track_drive_with_figure8.py

    Drives the TurtleBot in a racetrack-style path with a built-in figure-8 segment.
    Uses open-loop control (no odometry or feedback).

    Based on circle_drive.py
    Modified by Sameer Torke & ChatGPT, Oct 2025
'''

import os, time, argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Vector3


class TrackDrive(Node):
    def __init__(self, is_sim: bool, lin_x: float, ang_z: float, topic: str):
        super().__init__('track_drive_with_figure8')
        self.is_sim = is_sim
        self.lin_x = lin_x
        self.ang_z = ang_z

        if self.is_sim:
            self.publisher_ = self.create_publisher(Twist, topic, 1)
        else:
            self.publisher_ = self.create_publisher(TwistStamped, topic, 1)

        self.get_logger().info(f'Publishing velocity commands to {topic}')

    def get_motion(self, lin_x=0.0, ang_z=0.0):
        """Return motion command for sim or real robot."""
        if self.is_sim:
            return Twist(linear=Vector3(x=lin_x), angular=Vector3(z=ang_z))
        else:
            motion = TwistStamped(
                twist=Twist(linear=Vector3(x=lin_x), angular=Vector3(z=ang_z))
            )
            motion.header.stamp = self.get_clock().now().to_msg()
            return motion

    def drive_segment(self, lin_x, ang_z, duration):
        """Drive a motion segment for a fixed duration."""
        start = time.time()
        while time.time() - start < duration:
            self.publisher_.publish(self.get_motion(lin_x, ang_z))
            time.sleep(0.1)
        self.publisher_.publish(self.get_motion(0.0, 0.0))
        time.sleep(0.2)

    def drive_track(self):
        self.get_logger().info("Starting racetrack with figure-8 section...")

        # Parameters (adjust for tuning)
        straight_time = 4.0
        curve_time = 3.0
        figure8_curve_time = 2.5

        # --- Racetrack Start ---
        self.get_logger().info("Section: Straight start")
        self.drive_segment(self.lin_x, 0.0, straight_time)

        self.get_logger().info("Section: Left sweeping turn")
        self.drive_segment(self.lin_x, self.ang_z / 2.0, curve_time)

        self.get_logger().info("Section: Straight stretch before figure-8")
        self.drive_segment(self.lin_x, 0.0, straight_time)

        # --- Figure 8 section ---
        self.get_logger().info("Section: Entering figure-8")
        self.drive_segment(self.lin_x, self.ang_z, figure8_curve_time)
        self.drive_segment(self.lin_x, -self.ang_z, figure8_curve_time)
        self.drive_segment(self.lin_x, self.ang_z, figure8_curve_time)
        self.drive_segment(self.lin_x, -self.ang_z, figure8_curve_time)

        # --- Exit and finish track ---
        self.get_logger().info("Section: Straight exit")
        self.drive_segment(self.lin_x, 0.0, straight_time)

        self.get_logger().info("Section: Right sweeping turn to finish")
        self.drive_segment(self.lin_x, -self.ang_z / 2.0, curve_time)

        self.get_logger().info("Section: Final straight finish")
        self.drive_segment(self.lin_x, 0.0, straight_time)

        self.publisher_.publish(self.get_motion(0.0, 0.0))
        self.get_logger().info("Racetrack drive complete!")


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--lin_x', type=float, default=0.2, help='Linear velocity (m/s)')
    parser.add_argument('--ang_z', type=float, default=0.4, help='Angular velocity (rad/s)')
    parser.add_argument('--real_robot', action='store_true', help='Use for real TurtleBot')
    parsed_args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])

    topic = os.environ.get("ROBOT_NAMESPACE", "") + "/cmd_vel"
    node = TrackDrive(not parsed_args.real_robot, parsed_args.lin_x, parsed_args.ang_z, topic)
    node.drive_track()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
