#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
import math

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')

        # Subscribers
        self.ground_sub = self.create_subscription(PointStamped, '/ground_point', self.ground_callback, 1)
        self.obstacle_sub = self.create_subscription(MarkerArray, '/in_lane_obstacles', self.obstacle_callback, 1)
        
        #need to change 
        self.sign_sub = self.create_subscription(PointStamped, '/move_state', self.sign_callback, 1)

        # Publishers
        self.behavior_pub = self.create_publisher(PointStamped, '/behavior_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.latest_ground_point = None
        self.latest_obstacles = []
        self.stop_sign_detected = False
        self.safe_distance = 0.6  # meters to consider obstacle "too close"

        self.linear_velocity = 0.45  # default speed

        self.get_logger().info("Behavior node initialized")

    # ------------------- Callbacks ------------------- #
    def ground_callback(self, msg: PointStamped):
        self.latest_ground_point = msg
        self.update_behavior()

    def obstacle_callback(self, msg: MarkerArray):
        # Extract positions of obstacles
        self.latest_obstacles = []
        for marker in msg.markers:
            self.latest_obstacles.append((marker.pose.position.x, marker.pose.position.y))
        self.update_behavior()

    def sign_callback(self, msg: PointStamped):
        # Here we just use x>0 as a flag for stop sign
        self.stop_sign_detected = msg.point.x > 0.0
        self.update_behavior()

    # ------------------- Core Behavior ------------------- #
    def update_behavior(self):
        if self.latest_ground_point is None:
            return

        x = self.latest_ground_point.point.x
        y = self.latest_ground_point.point.y

        # Default: follow lane
        target_x = x
        target_y = y
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = 0.0

        # 1️ Stop for stop sign
        if self.stop_sign_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Stop sign detected: stopping")
            target_x = 0.0
            target_y = 0.0

        # 2️ Check obstacles
        for ox, oy in self.latest_obstacles:
            distance = math.sqrt(ox**2 + oy**2)
            if distance < self.safe_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Obstacle too close ({distance:.2f} m): stopping")
                target_x = 0.0
                target_y = 0.0
                break

        # Publish modified ground point
        behavior_point = PointStamped()
        behavior_point.header.stamp = self.latest_ground_point.header.stamp
        behavior_point.header.frame_id = "base_footprint"
        behavior_point.point.x = target_x
        behavior_point.point.y = target_y
        behavior_point.point.z = 0.0
        self.behavior_pub.publish(behavior_point)

        # Publish cmd_vel (optional, stops robot immediately)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure robot stops
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
