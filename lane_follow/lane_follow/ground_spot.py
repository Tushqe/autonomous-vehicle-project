#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class Obstacles(Node):
    def __init__(self, delta_r=0.25, lane_width=1.6):
        super().__init__('obstacle_detector')

        # LiDAR parameters
        self.delta_r = delta_r
        self.threshold = 0.25

        # Lane parameters
        self.lane_center_y = 0.0
        self.lane_width = lane_width

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/in_lane_obstacles', 1)
        self.target_pub = self.create_publisher(PointStamped, '/pp_target', 1)

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.lane_sub  = self.create_subscription(PointStamped, '/ground_point', self.update_lane_center, 1)

        self.get_logger().info("Obstacle detector with lane filtering initialized.")

    def update_lane_center(self, msg: PointStamped):
        """Update lane center position from GroundSpot node."""
        self.lane_center_y = msg.point.y

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan and compute obstacle-aware target point."""
        ranges = np.array(msg.ranges)
        if len(ranges) == 0:
            return

        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        good = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[good]
        angles = angles[good]
        if len(ranges) == 0:
            return

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Cluster points based on distance jumps
        clusters = []
        current_cluster = []
        for i in range(len(ranges)-1):
            current_cluster.append((x[i], y[i]))
            if abs(ranges[i+1] - ranges[i]) >= self.threshold:
                clusters.append(current_cluster)
                current_cluster = []
        current_cluster.append((x[-1], y[-1]))
        if current_cluster:
            clusters.append(current_cluster)

        # Merge first and last clusters if wrap-around
        if len(clusters) > 1 and abs(ranges[0] - ranges[-1]) < self.threshold:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop(-1)

        # Compute cluster centroids
        xcen_list, ycen_list = [], []
        for cluster in clusters:
            if len(cluster) == 0:
                continue
            x_coords = [p[0] for p in cluster]
            y_coords = [p[1] for p in cluster]
            xcen_list.append(np.mean(x_coords))
            ycen_list.append(np.mean(y_coords))

        # Filter centroids within lane boundaries
        in_lane_points = []
        ids = []
        for i, (xc, yc) in enumerate(zip(xcen_list, ycen_list)):
            if abs(yc - self.lane_center_y) <= self.lane_width/2:
                in_lane_points.append((xc, yc, 0.0))
                ids.append(i)

        # Publish visualization markers
        self.publish_markers(in_lane_points, ids, msg.header)

        # -------- Determine target point for pure pursuit --------
        # If no obstacle → target is lane center
        if not in_lane_points:
            target_point = PointStamped()
            target_point.header = msg.header
            target_point.point.x = 2.0  # arbitrary lookahead in x
            target_point.point.y = self.lane_center_y
            target_point.point.z = 0.0
            self.target_pub.publish(target_point)
            return

        # Find closest cluster ahead (positive x)
        in_lane_points.sort(key=lambda p: p[0])  # sort by x
        obs_x, obs_y, _ = in_lane_points[0]

        # Safety threshold: start avoiding if obstacle within 2 meters
        SAFETY_X = 2.0
        if obs_x > SAFETY_X:
            # far enough → keep lane
            target_point = PointStamped()
            target_point.header = msg.header
            target_point.point.x = 2.0
            target_point.point.y = self.lane_center_y
            target_point.point.z = 0.0
            self.target_pub.publish(target_point)
            return

        # Determine closest lane edge
        left_edge  = self.lane_center_y + self.lane_width/2
        right_edge = self.lane_center_y - self.lane_width/2

        dist_left  = abs(obs_y - left_edge)
        dist_right = abs(obs_y - right_edge)

        if dist_left < dist_right:
            # obstacle closer to left → bypass on right
            bypass_y = right_edge + 0.3
        else:
            # obstacle closer to right → bypass on left
            bypass_y = left_edge - 0.3

        # Publish new target
        target_point = PointStamped()
        target_point.header = msg.header
        target_point.point.x = obs_x + 0.8  # go slightly past obstacle
        target_point.point.y = bypass_y
        target_point.point.z = 0.0
        self.target_pub.publish(target_point)

    def publish_markers(self, points, ids, header):
        ma = MarkerArray()
        for i, p in zip(ids, points):
            mark = Marker()
            mark.header = header
            mark.id = i
            mark.type = Marker.SPHERE
            mark.pose = Pose(position=Point(x=p[0], y=p[1], z=p[2]),
                             orientation=Quaternion(x=0., y=0., z=0., w=1.))
            mark.scale.x = 0.25
            mark.scale.y = 0.25
            mark.scale.z = 0.25
            mark.color.a = 0.8
            mark.color.r = 1.0
            mark.color.g = 0.3
            mark.color.b = 0.3
            mark.lifetime = Duration(seconds=0.5).to_msg()
            ma.markers.append(mark)
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = Obstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
