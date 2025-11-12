#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math
import time

class FollowPurePursuit(Node):
    def __init__(self):
        super().__init__('follow_pure_pursuit')
        
        # Subscribe to ground_point topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/ground_point',
            self.pure_pursuit_callback,
            1)
        
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Tunable parameters
        self.linear_velocity = 0.45  # m/s - tune this for performance
        
        # Track if we've reached the end
        self.reached_end = False
        self.start_time = None
        self.end_time = None
        
        # Threshold for detecting end of road (no visible lane)
        self.no_lane_threshold = 0.001  # Small value to detect zeros
        
        self.get_logger().info('Pure Pursuit node initialized')

    def pure_pursuit_callback(self, msg):
        '''
        Callback to implement pure pursuit algorithm
        '''
        # Start timer on first valid point
        if self.start_time is None and not self.is_zero_point(msg.point):
            self.start_time = time.time()
            self.get_logger().info('Started lane following!')
        
        # Check if we've reached the end (no visible lane)
        if self.is_zero_point(msg.point):
            if not self.reached_end and self.start_time is not None:
                self.stop_robot()
                self.reached_end = True
                self.end_time = time.time()
                elapsed_time = self.end_time - self.start_time
                
                # Print results
                print("\n" + "="*40)
                print("LANE FOLLOWING COMPLETE!")
                print("="*40)
                print(f"Linear Velocity: {self.linear_velocity}")
                print(f"Time to complete: {elapsed_time:.2f}")
                print("="*40 + "\n")
                
                self.get_logger().info(f'Completed course in {elapsed_time:.2f} seconds')
            return
        
        # Don't process if we've already finished
        if self.reached_end:
            return
        
        # Extract ground point coordinates (in base_footprint frame)
        x = msg.point.x
        y = msg.point.y
        
        # Pure Pursuit Algorithm:
        # The lookahead distance is the Euclidean distance to the ground point
        lookahead_distance = math.sqrt(x**2 + y**2)
        
        # Handle case where lookahead distance is very small
        if lookahead_distance < 0.01:
            self.stop_robot()
            return
        
        # Pure pursuit formula for curvature:
        # curvature = 2 * y / lookahead_distance^2
        # where y is the lateral offset of the goal point
        curvature = 2.0 * y / (lookahead_distance ** 2)
        
        # Angular velocity = linear_velocity * curvature
        angular_velocity = self.linear_velocity * curvature
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(twist)
    
    def is_zero_point(self, point):
        '''
        Check if point is essentially zero (no visible lane)
        '''
        return (abs(point.x) < self.no_lane_threshold and 
                abs(point.y) < self.no_lane_threshold)
    
    def stop_robot(self):
        '''
        Publish zero velocities to stop the robot
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowPurePursuit()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure robot stops when node is killed
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()