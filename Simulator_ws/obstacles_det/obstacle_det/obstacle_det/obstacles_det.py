#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


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
        self.publisher = self.create_publisher(MarkerArray, '/in_lane_obstacles', 1)

        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.lane_sub = self.create_subscription(PointStamped, '/ground_point', self.update_lane_center, 1)

        self.get_logger().info("Obstacle detector with lane filtering initialized.")

    def update_lane_center(self, msg: PointStamped):
        """Update lane center position from GroundSpot node."""
        self.lane_center_y = msg.point.y  # lateral position of lane center
        # You could also log once every few seconds if desired
        # self.get_logger().info(f"Lane center updated: y={self.lane_center_y:.2f}")

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan and publish in-lane obstacles only."""
        ranges = np.array(msg.ranges)
        if len(ranges) == 0:
            return

        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        good = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[good]
        angles = angles[good]

        if len(ranges) == 0:
            return

        # Convert to Cartesian coordinates (base_footprint frame)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Cluster points based on distance jumps
        clusters = []
        current_cluster = []

        for i in range(len(ranges) - 1):
            current_cluster.append((x[i], y[i]))
            if abs(ranges[i + 1] - ranges[i]) >= self.threshold:
                clusters.append(current_cluster)
                current_cluster = []
        current_cluster.append((x[-1], y[-1]))
        if current_cluster:
            clusters.append(current_cluster)

        # Merge first and last clusters if scan wraps around
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
            if abs(yc - self.lane_center_y) <= self.lane_width / 2:
                in_lane_points.append((xc, yc, 0.0))
                ids.append(i)

        # Publish visualization markers
        self.publish_markers(in_lane_points, ids, msg.header)

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

        self.publisher.publish(ma)


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
