#!/usr/bin/env python
'''
    SquareDrive.py 

    This is an exmple ROS 2 node for driving a Turtlebot in a square and then stopping.
    It is open-loop and does not use feedback.

    
'''
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class SquareDrive(Node):
    def __init__(self, topic_name='/cmd_vel'):
        super().__init__('circle_drive')
        self.publisher_ = self.create_publisher(Twist, topic_name, 1)
        time.sleep(2.0)  # Wait for the node to connect
        self.get_logger().info('Publishing: ' + topic_name)        
	
    def drive(self, wait, motion = Twist() ):
        info = f'Drive for {wait:.2f} sec, linear speed {motion.linear.x:.2f} m/s, angular speed {motion.angular.z:.2f} rad/s'
        self.get_logger().info(info)
        self.publisher_.publish( motion )
        time.sleep( wait )
        self.publisher_.publish( Twist() )  # All-zero motion
        self.get_logger().info('Done')
        
def main(args=None):
    rclpy.init(args=args)

    av = SquareDrive()  # Initialize Node
    speed = 0.2  # meters per second
    side_length = 0.8  # meters
    straight_time = side_length / speed  # time to travel one side of the square

    # Define the motion for straight and turn
    straight_motion = Twist(linear=Vector3(x=speed))
    motion = Twist( linear=Vector3(x=0.5), angular=Vector3(z=1.79))
    # Drive in a square
    av.drive(wait= straight_time, motion=straight_motion)
    av.drive(wait = 1, motion = motion )
    av.drive(wait= straight_time-0.3/0.25, motion=straight_motion)
    for _ in range(2):
        av.drive(wait = 1, motion = motion )
        av.drive(wait= straight_time-0.2/0.25, motion=straight_motion)
    av.drive(wait= 0.1, motion=straight_motion)
    #av.drive(wait= straight_time, motion=straight_motion)
    #av.drive(wait = wait, motion = motion )
