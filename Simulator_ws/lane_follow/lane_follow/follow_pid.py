#!/usr/bin/env python3

"""
PID Lane Follower for Curvy Road
Subscribes to /lane_point and publishes /cmd_vel
Saves pid_params.txt when the course is completed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import time

# ---------------- PID class ---------------- #
class pid_controller():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd       
        self.previous_time_sec = None
        self.previous_error = 0.
        self.I_error = 0

    def update_control(self, target, state, current_time_sec):
        current_error = target - state
        dt = 0
        if self.previous_time_sec is not None:
            dt = current_time_sec - self.previous_time_sec

        # Integral
        self.I_error += 0.5 * (current_error + self.previous_error) * dt

        # Derivative
        D_error = 0
        if dt > 0:
            D_error = (current_error - self.previous_error) / dt

        # Save for next step
        self.previous_time_sec = current_time_sec
        self.previous_error = current_error

        # PID output
        u = self.kp * current_error + self.ki * self.I_error + self.kd * D_error
        return u

    def reset_integral(self):
        self.previous_error = 0.0
        self.I_error = 0.0

# ---------------- PID Lane Follower Node ---------------- #
class FollowPIDNode(Node):
    def __init__(self):
        super().__init__('follow_pid')

        # ---- PID Gains (tune these) ---- #
       # self.Kp = 0.005   # proportional
       # self.Ki = 0.0     # integral
        #self.Kd = 0.002   # derivative

        #self.Kp = 0.2      # proportional
        #self.Ki = 1.0   # small integral to remove steady bias
        #self.Kd = 0.6      # derivative to damp oscillation

        self.Kp = 0.4     # proportional
        self.Ki = 0.1   # small integral to remove steady bias
        self.Kd = 0.8     # derivative to damp oscillation
        self.pid = pid_controller(self.Kp, self.Ki, self.Kd)

        # Linear velocity (m/s)
        self.linear_vel = 0.1  # slower for sharper turns was 0.25

        # Focal length (pixels) for converting pixel error to angular velocity
        self.f_pixels = 500
        self.scaling_factor = 10.0  # scales PID output for noticeable turning was 10

        # Camera image center (pixels)
        self.image_center_x = 1280 / 2  # adjust if different was 640 / 2 

        # Publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lane_sub = self.create_subscription(PointStamped, '/lane_point', self.lane_callback, 10)
        self.get_logger().info("lane_callback function created")

        # Time tracking
        self.start_time = None
        self.finished = False

        self.get_logger().info("PID lane follower started.")

    def lane_callback(self, msg: PointStamped):
        self.get_logger().info("lane_callback function start")
        
        if self.finished:
            return

        # Start timer
        if self.start_time is None:
            self.start_time = time.time()

        # End of road detection
        if msg.point.x == 0 and msg.point.y == 0 and msg.point.z == 0:
            self.finished = True
            elapsed = time.time() - self.start_time
            self.stop_robot()
            self.pid.reset_integral()
            # Save PID params
            with open("pid_params.txt", "w") as f:
                f.write(f"Kp: {self.Kp}\nKi: {self.Ki}\nKd: {self.Kd}\n")
                f.write(f"Linear Velocity: {self.linear_vel}\n")
                f.write(f"Time to complete: {elapsed:.2f}\n")
            self.get_logger().info(f"Finished course in {elapsed:.2f} seconds")
            return

        # PID control
        current_time = time.time()
        pid_output = self.pid.update_control(self.image_center_x, msg.point.x, current_time)

        # Convert PID output to angular velocity
        angular_vel = pid_output / self.f_pixels * self.scaling_factor

        # CLAMP IT!
        max_angular = 0.5  # rad/s
        angular_vel = max(min(angular_vel, max_angular), -max_angular)


        # Publish Twist
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = angular_vel
        print(f"Angular vel = {angular_vel}")
        self.cmd_pub.publish(twist)


    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
