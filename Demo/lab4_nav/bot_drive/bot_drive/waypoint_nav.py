#!/usr/bin/env python
''' waypoint_nav.py

    To drive the TurtleBot to the way points being published to /waypoints topic.

    <...> Complete missing portions

    Copyright: Ankur Kamboj, 2025
'''
import rclpy
import argparse
import os
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation as R
import numpy as np

def angdiff(a,b,maxdiff=np.pi):
    ''' Returns difference of two angles: a - b in range -maxdiff to maxdiff
        For radians, maxdiff is pi, in degrees maxdiff is 180
    '''
    return (a-b+maxdiff)%(2*maxdiff)-maxdiff

def check_heading(waypoint: tuple[float, float],
                            current_pos: tuple[float, float],
                            current_yaw: float) -> float:
    ''' Checks the heading of the robot towards the waypoint. '''  
    # Compute the desired yaw angle
    desired_yaw = np.arctan2(waypoint[1] - current_pos[1], waypoint[0] - current_pos[0])

    # Check and return the difference
    return angdiff(desired_yaw, current_yaw)

def yaw_from_quaternion(quat) -> float:
    ''' Returns yaw from a quaternion '''
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    return rot.as_rotvec()[2]

class WaypointNav(Node):
    ''' Drives to waypoints '''

    def __init__(self, lin_x: float, ang_z: float, simulator: bool):
        super().__init__('waypoint_nav')

        namespace = os.environ.get("ROBOT_NAMESPACE","")
        self.simulator = simulator
    
        # <...> create a /cmd_vel publisher with an appropriate namespace and output a log with what is being published
        if namespace:
            topic = f"/{namespace}/cmd_vel"
        else:
            topic = "cmd_vel"
        
        if simulator:
            self.publisher= self.create_publisher(Twist,topic,10)
            self.get_logger().info(f"Publishing Twist commands: {topic}")
        else:
            self.publisher = self.create_publisher(TwistStamped,topic,10)
            self.get_logger().info(f"Publishing TwistStamped commands: {topic}")

        # These are useful variables for the callbacks:
        self.prev_dist = np.inf # Set to infinity initially
        self.rotated  = False
        self.waiting_for_waypoint = False
        self.current_waypoint = None
        self.prev_waypoint = None

        self.ang_z = ang_z
        self.lin_x = lin_x

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # <...> Create a subscriber for /odom (with appropriate namespace).  The callback should be: self.odom_callback
        if namespace:
            odom_topic = f"/{namespace}/odom"
        else:
            odom_topic = "/odom"
        # <...> Make a log describing the subscriber
        self.create_subscription(Odometry,odom_topic,self.odom_callback,qos_profile)
        self.get_logger().info(f"subscribed to {odom_topic}")
        # <...> Create a subscriber for /waypoints with callback: self.odom_callback
        self.create_subscription(PointStamped,"/waypoints",self.waypoint_callback,10)
        # <...> Make a log describing the subscriber
        self.get_logger().info("subscribed to /waypoints")
    def waypoint_callback(self, msg):
        ''' Callback to receive waypoints '''
        # <...> This should set the current waypoint from the message
        #       It should also check if the waypoint is different from the previous and turn off 
        #       a condition in which we are stopped at a waypoint
        #       Also, output a log message if we have a new waypoint
        waypoint = (msg.point.x,msg.point.y)
        if self.current_waypoint != waypoint:
            self.current_waypoint = waypoint
            self.rotated = False
            self.prev_dist = np.inf
            self.get_logger().info(f"Receive new waypoint {waypoint}")

    def odom_callback(self, msg):

        if self.current_waypoint is None:
            if not self.waiting_for_waypoint:
                self.get_logger().info('No waypoint received yet, waiting ...')
                self.waiting_for_waypoint = True # Set flag so it doesn't print again
            
            if self.simulator:
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
            else:
                stop_msg = TwistStamped()
                stop_msg.twist.linear.x = 0.0
                stop_msg.twist.angular.z = 0.0
            # <...> publish a zero motion
            self.publisher.publish(stop_msg)
            return

        if self.prev_waypoint is None:
            self.prev_waypoint = self.current_waypoint
            self.get_logger().info(f'Moving to waypoint {self.current_waypoint}')

        if self.prev_waypoint != self.current_waypoint:
            self.rotated = False
            self.prev_waypoint = self.current_waypoint
            self.prev_dist = np.inf
            self.get_logger().info(f'Moving to new waypoint {self.current_waypoint}')
            
        # <...> Find the current position as a tuple (x,y):
        pos = msg.pose.pose.position
        current_pos = (pos.x,pos.y)
        # <...> Find the distance to the current waypoint 
        current_dist = np.linalg.norm(np.array(self.current_waypoint) - np.array(current_pos))

        # Determine the motion command with two components with default values of 0:
        angular_vel = 0.0
        linear_vel = 0.0

        if not self.rotated:
            # If we are not yet pointing at the waypoint
            # <...> get the current yaw from odometry:
            quadternion = msg.pose.pose.orientation
            current_yaw = yaw_from_quaternion(quadternion)
            # Rotate till facing the waypoint
            # <...> heading error:
            heading_error = check_heading(self.current_waypoint,current_pos,current_yaw)


            if np.abs(heading_error) > 3e-2:
                # <...> set angular_vel and log this
                angular_vel = self.ang_z * np.sign(heading_error)
                self.get_logger().info(f"Rotating: error ={heading_error:.3f}")

            else:
                self.rotated = True
                self.get_logger().info(f'Heading aligned, moving forward to waypoint {self.current_waypoint}')
        
        else:
            # Moving forward to waypoint
            if current_dist < 0.1:
                if self.prev_dist is not np.inf:
                    self.get_logger().info(f'Reached waypoint {self.current_waypoint}')
                self.prev_dist = np.inf
            elif current_dist > self.prev_dist:
                self.get_logger().info(f'Oops, getting further away from waypoint, correcting rotation')
                self.rotated = False
                self.prev_dist = current_dist
            else:
                # <...> set linear_vel
                linear_vel = self.lin_x
                self.get_logger().info(f'Moving forward to waypoint {self.current_waypoint}, distance {current_dist:.2f} m')
                self.prev_dist = current_dist
        
        # <...> Create a motion command depending on if simulator or not
        if self.simulator:
            msg_out = Twist()
            msg_out.linear.x = linear_vel
            msg_out.angular.z = angular_vel
        else:
            msg_out = TwistStamped()
            msg_out.twist.linear.x = linear_vel
            msg_out.twist.angular.z = angular_vel
        # <...> Publish the motion command
        self.publisher.publish(msg_out)


def main(args=None):
    
    rclpy.init(args=args)

    # <...> Read in arguments (see circle_drive.py for an example)
    parser = argparse.ArgumentParser(description="waypoint nav node")
    parser.add_argument('--lin_x',type=float,default=0.1,help='linear velocity in x (m/s)')
    parser.add_argument('--ang_z',type=float,default=0.3,help='linear velocity in z (rad/s)')
    parser.add_argument('--sim',action='store_true',help='Use simulator mode if set')
    # <...> Initialize a WaypointNav class variable with appropriate parameters
    parsed_args = parser.parse_args()
    Waypoint_nav = WaypointNav(parsed_args.lin_x,parsed_args.ang_z,parsed_args.sim)
    # <...> Do a spin    
    try:
        rclpy.spin(Waypoint_nav)
    except KeyboardInterrupt:
        pass
    finally:
        Waypoint_nav.destroy_node()
        rclpy.shutdown()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()

