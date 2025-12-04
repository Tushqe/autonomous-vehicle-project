#!/usr/bin/env python 
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool   # <--- NEW
import numpy as np
import math 

class Obstacles(Node):
    def __init__(self, delta_r=0.25):
        super().__init__('bus_monitor')             
        self.delta_r = delta_r

        self.publisher = self.create_publisher(MarkerArray, 'obstacles', 1)    
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_centroids, 1)

        # =====================================
        # NEW: Bool publisher for pure pursuit
        # =====================================
        self.pub_flag = self.create_publisher(Bool, '/front_obstacle', 10)


    # Helper: check angle inside [-135°, +135°]
    def in_front_270(self, angles):
        return (angles > -2.356) & (angles < 2.356)


    # MAIN LIDAR PROCESSING
    def lidar_centroids(self, msg):       
        ranges = np.array(msg.ranges)
        if len(ranges) == 0:
            return

        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        good = np.isfinite(ranges) & (ranges > 0)

        ranges = ranges[good]
        angles = angles[good]

        if len(ranges) == 0:
            print("ranges len is 0")
            return
        
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # ---- clustering ----
        clusters = []
        current_cluster = []
        threshold = 0.25

        for i in range(len(ranges)-1):
            current_cluster.append((x[i], y[i]))
            distance = abs(ranges[i+1] - ranges[i])
            if distance >= threshold:
                clusters.append(current_cluster)
                current_cluster = []
        
        current_cluster.append((x[-1], y[-1]))
        
        if current_cluster:
            clusters.append(current_cluster)

        if len(clusters) > 1 and abs(ranges[0] - ranges[-1]) < threshold:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop(-1)
       
        # Compute centroids
        centroids = []
        for cluster in clusters:
            xs = [pt[0] for pt in cluster]
            ys = [pt[1] for pt in cluster]
            if len(xs) == 0:
                continue
            cx = np.mean(xs)
            cy = np.mean(ys)
            centroids.append((cx, cy))

        if len(centroids) == 0:
            # Nothing detected → send False
            self.pub_flag.publish(Bool(data=False))
            self.publisher.publish(MarkerArray())
            return

        # Filter by 270° + distance threshold
        dist_thresh = 0.75
        front_centroids = []

        for (cx, cy) in centroids:
            ang = math.atan2(cy, cx)
            dist = math.sqrt(cx*cx + cy*cy)

            if self.in_front_270(np.array([ang])) and dist < dist_thresh:
                front_centroids.append((cx, cy, dist))

        # None in front → publish False
        if len(front_centroids) == 0:
            self.pub_flag.publish(Bool(data=False))
            self.publisher.publish(MarkerArray())
            return

        # Closest obstacle
        front_centroids.sort(key=lambda x: x[2])
        cx, cy, d = front_centroids[0]

        # =============================
        # NEW: Publish True (obstacle!)
        # =============================
        self.pub_flag.publish(Bool(data=True))

        # Publish marker for RViz
        points = [(cx, cy, 0.0)]
        ids = np.array([0])
        self.pub_centroids(points, ids, msg.header)


    def pub_centroids(self, points, ids, header):

        ma = MarkerArray()

        for id, p in zip(ids, points):
            mark = Marker()            
            mark.header = header
            mark.id = int(id)
            mark.type = Marker.SPHERE
            mark.pose = Pose(position=Point(x=p[0],y=p[1],z=p[2]),
                             orientation=Quaternion(x=0.,y=0.,z=0.,w=1.))
            mark.scale.x = 0.25
            mark.scale.y = 0.25
            mark.scale.z = 0.25
            mark.color.a = 0.9
            mark.color.r = 1.0
            mark.color.g = 0.2
            mark.color.b = 0.2
            mark.lifetime = Duration(seconds=0.4).to_msg()
            ma.markers.append(mark)

        self.publisher.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = Obstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
