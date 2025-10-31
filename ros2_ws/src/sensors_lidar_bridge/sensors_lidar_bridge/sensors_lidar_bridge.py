#!/usr/bin/env python3
import math, time, threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Optional serial backend (RPLIDAR)
try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except Exception:
    RPLIDAR_AVAILABLE = False

class SensorsLidarBridge(Node):
    """
    Modes:
      - 'bridge' : subscribe to existing LaserScan (e.g., /scan) and republish
      - 'serial' : read RPLIDAR on /dev/ttyUSB* and publish LaserScan
      - 'sim'    : synthetic scan for testing

    Publishes:
      - /lidar/scan_raw   (sensor_msgs/LaserScan)
      - /sync/lidar       (sensor_msgs/LaserScan)  ← for your sync node
    """
    def __init__(self):
        super().__init__('sensors_lidar_bridge')

        # Params
        self.declare_parameter('mode', 'bridge')           # bridge | serial | sim
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic_in', '/scan')   # bridge input
        self.declare_parameter('raw_topic', '/lidar/scan_raw')
        self.declare_parameter('sync_topic', '/sync/lidar')

        # Serial (RPLIDAR)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)

        # Sim
        self.declare_parameter('sim_rate_hz', 10.0)
        self.declare_parameter('sim_angle_min', -3.14159)
        self.declare_parameter('sim_angle_max',  3.14159)
        self.declare_parameter('sim_angle_increment', 0.00436)
        self.declare_parameter('sim_range_min', 0.15)
        self.declare_parameter('sim_range_max', 8.0)

        gp = lambda n: self.get_parameter(n).get_parameter_value
        self.mode       = gp('mode')().string_value
        self.frame_id   = gp('frame_id')().string_value
        self.scan_in    = gp('scan_topic_in')().string_value
        self.raw_topic  = gp('raw_topic')().string_value
        self.sync_topic = gp('sync_topic')().string_value

        self.pub_raw  = self.create_publisher(LaserScan, self.raw_topic, 10)
        self.pub_sync = self.create_publisher(LaserScan, self.sync_topic, 10)

        self._stop = threading.Event()

        if self.mode == 'bridge':
            self.sub = self.create_subscription(LaserScan, self.scan_in, self._bridge_cb, 10)
            self.get_logger().info(f"Bridging {self.scan_in} → {self.raw_topic}, {self.sync_topic}")
        elif self.mode == 'serial':
            if not RPLIDAR_AVAILABLE:
                self.get_logger().error("Missing rplidar library. Install with: pip install rplidar")
                raise RuntimeError("RPLIDAR unavailable")
            self._init_rplidar()
            self._thread = threading.Thread(target=self._loop_rplidar, daemon=True)
            self._thread.start()
        elif self.mode == 'sim':
            self._init_sim()
            self.timer = self.create_timer(1.0/max(self.sim_rate_hz,1.0), self._tick_sim)
        else:
            raise RuntimeError(f"Unknown mode: {self.mode}")

    # ---- bridge mode ----
    def _bridge_cb(self, msg: LaserScan):
        msg.header.frame_id = self.frame_id
        self.pub_raw.publish(msg)
        self.pub_sync.publish(msg)

    # ---- serial (RPLIDAR) ----
    def _init_rplidar(self):
        port = self.get_parameter('serial_port').value
        self.lidar = RPLidar(port)
        _ = self.lidar.get_info()
        _ = self.lidar.get_health()
        # Default scan properties
        self.angle_min = -math.pi
        self.angle_max =  math.pi
        self.range_min = 0.15
        self.range_max = 8.0
        self.angle_increment = math.radians(1.0)  # ~1°

    def _loop_rplidar(self):
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=5000):
                n = int((self.angle_max - self.angle_min)/self.angle_increment) + 1
                ranges = [float('inf')] * n
                for (_, angle_deg, dist_mm) in scan:
                    ang = ((math.radians(angle_deg)+math.pi)%(2*math.pi))-math.pi
                    idx = int((ang - self.angle_min)/self.angle_increment)
                    if 0 <= idx < n:
                        d = dist_mm/1000.0
                        if self.range_min <= d <= self.range_max:
                            ranges[idx] = min(ranges[idx], d)
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.angle_min = float(self.angle_min)
                msg.angle_max = float(self.angle_max)
                msg.angle_increment = float(self.angle_increment)
                msg.range_min = float(self.range_min)
                msg.range_max = float(self.range_max)
                msg.ranges = ranges
                msg.scan_time = 1.0/6.0
                msg.time_increment = msg.scan_time/max(len(ranges),1)
                self.pub_raw.publish(msg)
                self.pub_sync.publish(msg)
                if self._stop.is_set(): break
        except Exception as e:
            self.get_logger().error(f"RPLIDAR loop error: {e}")
        finally:
            try:
                self.lidar.stop(); self.lidar.disconnect()
            except Exception:
                pass

    # ---- sim ----
    def _init_sim(self):
        gp = lambda n: self.get_parameter(n).get_parameter_value
        self.sim_rate_hz     = float(gp('sim_rate_hz')().double_value)
        self.angle_min       = float(gp('sim_angle_min')().double_value)
        self.angle_max       = float(gp('sim_angle_max')().double_value)
        self.angle_increment = float(gp('sim_angle_increment')().double_value)
        self.range_min       = float(gp('sim_range_min')().double_value)
        self.range_max       = float(gp('sim_range_max')().double_value)

    def _tick_sim(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        n = int((self.angle_max - self.angle_min)/self.angle_increment) + 1
        ranges = [self.range_max]*n
        front = math.radians(20)
        for i in range(n):
            ang = self.angle_min + i*self.angle_increment
            if -front <= ang <= front:
                ranges[i] = 2.5 + 0.1*math.sin(0.5*t)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_increment)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = ranges
        msg.scan_time = 1.0/max(self.sim_rate_hz,1.0)
        msg.time_increment = msg.scan_time/max(n,1)
        self.pub_raw.publish(msg)
        self.pub_sync.publish(msg)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = SensorsLidarBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f"[sensors_lidar_bridge] Fatal: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
