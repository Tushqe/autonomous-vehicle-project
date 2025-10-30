import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber

class PerceptionSyncNode(Node):
    def __init__(self):
        super().__init__('perception_sync_node')
        self.get_logger().info('Perception Sync Node (LiDAR + IMU) Initialized')

        self.lidar_sub = Subscriber(self, LaserScan, '/scan')
        self.imu_sub = Subscriber(self, Imu, '/imu')

        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.imu_sub],
            queue_size=10,
            slop=0.05 
        )
        self.ts.registerCallback(self.sync_callback)

        self.sync_pub = self.create_publisher(String, '/perception_sync/data', 10)

    def sync_callback(self, lidar_msg, imu_msg):
        """Called whenever synchronized LiDAR + IMU data are received."""
        num_points = len(lidar_msg.ranges)
        imu_orient = imu_msg.orientation

        self.get_logger().info(
            f"Synced LiDAR ({num_points} points) + IMU orientation "
            f"({imu_orient.x:.2f}, {imu_orient.y:.2f}, {imu_orient.z:.2f})"
        )

        msg = String()
        msg.data = f"Synced LiDAR + IMU data | Points: {num_points}"
        self.sync_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
