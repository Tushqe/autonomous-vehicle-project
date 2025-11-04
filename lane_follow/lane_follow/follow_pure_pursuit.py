import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PointStamped
import math
import time
import os

class FollowPurePursuit(Node):
    def __init__(self):
        super().__init__('follow_pure_pursuit')

        #parameters
        self.linear_velocity = 0.2
        self.reached_threshold =0.05

        #state
        self.goal_point = None
        self.start_time = None
        self.finished = False

        #publisher and suscriber
        self.cmd_pub = self.create_publisher(Twist, 
                                             '/cmd_vel',
                                             10)
        self.sub_goal = self.create_subscription(PointStamped,
                                                 '/ground_point',
                                                 self.update_goal,
                                                 10)
        #timer
        self.timer = self.create_timer(0.05,
                                       self.control_loop)
    def update_goal(self, msg:PointStamped):
        self.goal_point = msg.point
        if self.start_time is None:
            self.start_time = time.time()
    def control_loop(self):
        if self.goal_point is None or self.finished:
            return
        x = self.goal_point.x
        y = self.goal_point.y

        #if at ground_point
        if x == 0.0 and y == 0.0:
            
            self.publish_twist(0.0,0.0)
            if not self.finished and self.start_time is not None:
                self.finished = True
                end_time = time.time()
                time_taken = end_time - self.start_time
                self.get_logger().info(f'Linear Velocity: {self.linear_velocity:.2f} m/s')
                self.get_logger().info(f'Time to complete: {time_taken:.2f} s')
            try:
                save_dir = os.path.expanduser("~/av/wangnat1_av/lab9_lane/lane_follow")
                save_path = os.path.join(save_dir,"pp_params.txt")
                with open(save_path, 'w') as f:
                    f.write(f'Linear Velocity: {self.linear_velocity:.2f} m/s\n')
                    f.write(f'Time to complete: {time_taken:.2f} s\n')
                    f.flush()
                    os.fsync(f.fileno())
                    self.get_logger().info("saved pp_params.txt")
            except Exception as e:
                self.get_logger().warn(f"failed to write pp_param.txt: {e}")
            return
        #compute angle to goal
        angle_to_goal = math.atan2(y,x)

        dist = math.hypot(x,y)
        #pure pursuit, 
        k_angular = 2.0
        angular_vel = k_angular * angle_to_goal
        self.publish_twist(self.linear_velocity,angular_vel)
    
    def publish_twist(self,linear,angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = FollowPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
            
        
