'''
    Very simple PID controller
    Has option to input state-rate rather than calculating this numerically

    Daniel Morris, 2022, 2023
'''
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import os


class pid_controller():
    def __init__(self, kp, ki, kd):
        ''' Can optionally require a state rate input
            This avoids latencies from numerical calculation of the derivative
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd       
        self.previous_time_sec = None
        self.previous_error = 0.
        self.previous_target = None
        self.I_error = 0

    def update_control(self, target, state, current_time_sec):
        ''' Will calculate derivative numerically '''

        current_error = target - state

        if self.previous_time_sec:
            dt = current_time_sec - self.previous_time_sec
        else:
            dt = 0
        
        # Numerical integration
        self.I_error +=  (self.previous_error + current_error) * dt / 2
        max_I = 0.5 
        self.I_error = max(min(self.I_error, max_I), -max_I)

        if self.previous_error and dt > 0:        
            # Numerical differentiation
            D_error = ( current_error - self.previous_error ) / dt
        else:
            D_error = 0
        
        self.previous_time_sec = current_time_sec
        self.previous_error = current_error
        self.previous_target = target

        u = self.kp * current_error + self.ki * self.I_error + self.kd * D_error

        return (u, current_error, self.I_error, D_error)

    def update_control_with_rate(self, target, state, state_rate, current_time_sec):
        ''' Uses state rate as part of the derivative '''

        current_error = target - state

        if self.previous_time_sec:
            dt = current_time_sec - self.previous_time_sec
        else:
            dt = 0

        # Numerical integration       
        self.I_error +=  (self.previous_error + current_error) * dt / 2
        max_I = 0.5 
        self.I_error = max(min(self.I_error, max_I), -max_I)

        # Use state_rate instead of differencing state -- can be more stable
        if self.previous_target and dt > 0:
            D_error = (target - self.previous_target) / dt - state_rate
        else:
            D_error = -state_rate
        
        self.previous_time_sec = current_time_sec
        self.previous_error = current_error
        self.previous_target = target

        u = self.kp * current_error + self.ki * self.I_error + self.kd * D_error

        return (u, current_error, self.I_error, D_error)
        
    def reset_integral(self):
        ''' Reset integral and last tracked error. Useful when tracking is lost. '''
        self.previous_error = 0.0
        self.I_error = 0.0

class FollowerPID(Node):
    def __init__(self):
        super().__init__('follower_pid')


        self.declare_parameter('kp',0.4)
        self.declare_parameter('ki',0.0)
        self.declare_parameter('kd',0.05)
        self.declare_parameter('linear_velocity', 0.1)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('focal_length', 576.83)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.image_width = self.get_parameter('image_width').value
        self.focal_length = self.get_parameter('focal_length').value
        #approx focal length in pixel angle conversion

        # pid setup
        self.pid = pid_controller(kp,ki,kd)

        #subscription and publishers

        self.error_sub = self.create_subscription(
            PointStamped, 
            '/lane_point',
            self.lane_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, 
                                             '/cmd_vel',
                                             10)
        #control loop
        self.timer = self.create_timer(0.1, self.control_loop)


        #state
        self.lane_error = 0.0
        self.start_time = time.time()
        self.end_time = None
        self.finished = False
        self.get_logger().info('PID lane follower node has started')
        
    def lane_callback(self, msg):
        # checking for ms shoudl be [x,y]
        if  msg.point.x == 0 and msg.point.y == 0:
            self.finished = True
            self.end_time = time.time()
            self.pid.reset_integral()
            self.lane_error = None
            return 
        if msg.point.x is None or msg.point.y is None:
            self.pid.reset_integral()
            self.lane_error = None
            return
       
            
            
        x = msg.point.x
        pixel_error = (self.image_width/2.0) - float(x)

        # compute pixel error from center
        self.lane_error = pixel_error / self.focal_length # covert to angluar error
        self.get_logger().info(f"x = {x}, lane_error = {self.lane_error:.4f}")
    
    def control_loop(self):
        
        if self.finished: 
            twist_msg = Twist()
            self.cmd_pub.publish(twist_msg)
            time_taken = self.end_time - self.start_time
            print(f"Kp: {self.pid.kp}")
            print(f"Ki: {self.pid.ki}")
            print(f"Kd: {self.pid.kd}")
            print(f"linear vel: {self.linear_velocity}")
            print(f"Time to complete: {time_taken:.2f} seconds")
            try:
                save_dir = os.path.expanduser("~/av/wangnat1_av/lab9_lane/lane_follow")
                save_path = os.path.join(save_dir,"pid_params.txt")
                
                with open(save_path, "w") as f:
                    f.write(f"Kp: {self.pid.kp}\n")
                    f.write(f"Ki: {self.pid.ki}\n")
                    f.write(f"Kd: {self.pid.kd}\n")
                    f.write(f"Linear Vel: {self.linear_velocity}\n")
                    f.write(f"Time to complete: {time_taken:.2f} seconds\n")
                    f.flush()
                    os.fsync(f.fileno())
                self.get_logger().info("saved pid_params.txt")
            except Exception as e:
                self.get_logger().warn(f"failed to write pid_param.txt: {e}")
            rclpy.shutdown()
            time.sleep(0.2)
            return
        if self.lane_error is None:
            twist_msg =Twist()
            twist_msg.linear.x = float(self.linear_velocity)
            twist_msg.angular.z = 0.0
            self.cmd_pub.publish(twist_msg)
            self.get_logger().info("pausing PID")
            return 


        current_time = time.time()
        
        u,err,I,D = self.pid.update_control(0.0,self.lane_error,current_time)

        twist_msg = Twist()
        abs_u = abs(u)
        max_u_limit = 0.6
        min_vel = 0.05 
        clamped_u = min(abs_u, max_u_limit)
        speed_factor = clamped_u/max_u_limit 
        new_linear_vel = self.linear_velocity - speed_factor * (self.linear_velocity - min_vel)
        twist_msg.linear.x = new_linear_vel
        max_angular = 0.5
        twist_msg.angular.z = max(min(-u,max_angular),-max_angular)
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(f"PID output: u = {u:.2f}, P = {err:.2f}, I = {I:.2f},D = {D:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
        
        
