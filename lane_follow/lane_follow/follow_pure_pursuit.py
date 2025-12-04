#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import math
import time


class FollowPurePursuit(Node):
    def __init__(self):
        super().__init__('follow_pure_pursuit')
        
        # Sub: ground point (lane center)
        self.subscription = self.create_subscription(
            PointStamped,
            '/ground_point',
            self.pure_pursuit_callback,
            1)

        # Sub: obstacle detection flag
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/front_obstacle',
            self.obstacle_callback,
            10)

        # Pub: REAL CAR COMMAND TOPICS (Float64)
        self.pub_speed = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.pub_steer = self.create_publisher(Float64, '/commands/servo/position', 10)

        # Fixed motor speed command (VESC expects high Float64 values)
        self.motor_speed_command = 5000.0

        # State machine
        self.reached_end = False
        self.start_time = None
        self.end_time = None
        self.no_lane_threshold = 0.001

        # Obstacle flag
        self.front_obstacle = False

        self.get_logger().info("Pure Pursuit node initialized")


    # ============================================================
    # OBSTACLE CALLBACK
    # ============================================================
    def obstacle_callback(self, msg: Bool):
        self.front_obstacle = msg.data

        if self.front_obstacle:
            self.get_logger().info("[OBSTACLE] Stopping vehicle.")
            self.stop_robot()


    # ============================================================
    # MAIN PURE PURSUIT
    # ============================================================
    def pure_pursuit_callback(self, msg: PointStamped):

        # If obstacle → STOP immediately
        if self.front_obstacle:
            self.stop_robot()
            return

        # Start timer when we see the first lane point
        if self.start_time is None and not self.is_zero_point(msg.point):
            self.start_time = time.time()
            self.get_logger().info('Started lane following!')

        # Lane disappears → STOP and record time
        if self.is_zero_point(msg.point):
            if not self.reached_end and self.start_time is not None:
                self.stop_robot()
                self.reached_end = True
                self.end_time = time.time()
                elapsed = self.end_time - self.start_time

                print("\n" + "="*40)
                print("LANE FOLLOWING COMPLETE!")
                print("="*40)
                print(f"Speed Command: {self.motor_speed_command}")
                print(f"Time to complete: {elapsed:.2f} seconds")
                print("="*40 + "\n")

                self.get_logger().info(f"Completed course in {elapsed:.2f} seconds")
            return
        
        if self.reached_end:
            return

        # Extract lane center point
        x = msg.point.x
        y = msg.point.y

        L = math.sqrt(x * x + y * y)
        if L < 0.01:
            self.stop_robot()
            return

        # Pure Pursuit curvature → steering
        curvature = 2.0 * y / (L * L)
        steering = curvature    # You can scale this later if needed

        # ================================
        # PUBLISH COMMANDS (Float64)
        # ================================
        self.pub_speed.publish(Float64(data=self.motor_speed_command))
        self.pub_steer.publish(Float64(data=float(steering)))


    # ============================================================
    def is_zero_point(self, point):
        return (abs(point.x) < self.no_lane_threshold and
                abs(point.y) < self.no_lane_threshold)


    # ============================================================
    def stop_robot(self):
        """Stop motor and steering."""
        self.pub_speed.publish(Float64(data=0.0))
        self.pub_steer.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = FollowPurePursuit()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
