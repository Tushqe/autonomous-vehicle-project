#!/usr/bin/env python3
import math, time, threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

# Optional deps (installed later on Jetson)
try:
    from smbus2 import SMBus
except Exception:
    try: from smbus import SMBus
    except Exception: SMBus = None

try:
    import serial
except Exception:
    serial = None


class SensorsImuBridge(Node):
    """
    Modes:
      - 'i2c'    : Read MPU6050 via I²C (/dev/i2c-*).
      - 'serial' : Read CSV lines "ax,ay,az,gx,gy,gz" from /dev/tty* (m/s^2, rad/s).
      - 'gazebo' : Subscribe to an existing sensor_msgs/Imu and republish.
      - 'sim'    : Publish synthetic IMU data.

    Publishes to:
      - /imu/data_raw   (sensor_msgs/Imu)
      - /sync/imu       (sensor_msgs/Imu)  ← for your sync node (LiDAR also publishes there)
    """
    # MPU6050 registers/sensitivity
    MPU6050_PWR_MGMT_1   = 0x6B
    MPU6050_ACCEL_XOUT_H = 0x3B
    MPU6050_GYRO_XOUT_H  = 0x43
    ACCEL_SENS = 16384.0     # LSB/g  (±2g)
    GYRO_SENS  = 131.0       # LSB/(deg/s) (±250 dps)

    def __init__(self):
        super().__init__('sensors_imu_bridge')

        # Parameters
        self.declare_parameter('mode', 'i2c')      # i2c | serial | gazebo | sim
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)

        # I2C params
        self.declare_parameter('i2c_bus', 1)       # /dev/i2c-1
        self.declare_parameter('i2c_address', 0x68)

        # Serial params
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)

        # Gazebo
        self.declare_parameter('gazebo_input_topic', '/imu')

        # Topics
        self.declare_parameter('raw_topic', '/imu/data_raw')
        self.declare_parameter('sync_topic', '/sync/imu')

        gp = lambda n: self.get_parameter(n).get_parameter_value
        self.mode      = gp('mode')().string_value
        self.frame_id  = gp('frame_id')().string_value
        self.rate_hz   = float(gp('rate_hz')().double_value)
        self.raw_topic = gp('raw_topic')().string_value
        self.sync_topic= gp('sync_topic')().string_value

        self.pub_raw  = self.create_publisher(Imu, self.raw_topic, 10)
        self.pub_sync = self.create_publisher(Imu, self.sync_topic, 10)

        self.get_logger().info(f"IMU mode={self.mode}, rate={self.rate_hz} Hz, frame_id={self.frame_id}")
        self.get_logger().info(f"Publishing → {self.raw_topic} and {self.sync_topic}")

        self._stop = threading.Event()
        self._thread = None

        if self.mode == 'i2c':
            self._init_i2c_mpu6050()
            self._thread = threading.Thread(target=self._loop_i2c, daemon=True)
            self._thread.start()
        elif self.mode == 'serial':
            self._init_serial()
            self._thread = threading.Thread(target=self._loop_serial, daemon=True)
            self._thread.start()
        elif self.mode == 'gazebo':
            topic = gp('gazebo_input_topic')().string_value
            self.sub = self.create_subscription(Imu, topic, self._gazebo_cb, 10)
            self.get_logger().info(f"Bridging Gazebo topic {topic} → {self.raw_topic}, {self.sync_topic}")
        elif self.mode == 'sim':
            self.timer = self.create_timer(1.0/max(self.rate_hz,1.0), self._tick_sim)
        else:
            raise RuntimeError(f"Unknown mode: {self.mode}")

    # ---------- Gazebo bridge ----------
    def _gazebo_cb(self, msg: Imu):
        msg.header.frame_id = self.frame_id
        self.pub_raw.publish(msg)
        self.pub_sync.publish(msg)

    # ---------- Simulation ----------
    def _tick_sim(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angular_velocity.z = 0.5*math.sin(0.5*t)
        msg.linear_acceleration.z = 9.81
        msg.orientation_covariance[0] = -1.0
        self.pub_raw.publish(msg)
        self.pub_sync.publish(msg)

    # ---------- Serial backend ----------
    def _init_serial(self):
        if serial is None:
            self.get_logger().error("pyserial missing. Install: pip install pyserial")
            raise RuntimeError("serial unavailable")
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"Opened serial {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise

    def _loop_serial(self):
        period = 1.0/max(self.rate_hz,1.0)
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                parts = line.split(',')
                if len(parts) == 6:
                    ax, ay, az, gx, gy, gz = [float(x) for x in parts]
                    self._publish(ax, ay, az, gx, gy, gz)
            except Exception:
                pass
            time.sleep(period)

    # ---------- I2C (MPU6050) ----------
    def _init_i2c_mpu6050(self):
        if SMBus is None:
            self.get_logger().error("smbus/smbus2 missing. Install python3-smbus or pip install smbus2")
            raise RuntimeError("i2c unavailable")
        bus_id = int(self.get_parameter('i2c_bus').value)
        addr   = int(self.get_parameter('i2c_address').value)
        try:
            self.bus = SMBus(bus_id)
            self.bus.write_byte_data(addr, self.MPU6050_PWR_MGMT_1, 0x00)  # wake
            self.i2c_addr = addr
            self.get_logger().info(f"MPU6050 ready on i2c-{bus_id} @ 0x{addr:02X}")
        except Exception as e:
            self.get_logger().error(f"MPU6050 init failed: {e}")
            raise

    def _read_word(self, reg_h):
        hi = self.bus.read_byte_data(self.i2c_addr, reg_h)
        lo = self.bus.read_byte_data(self.i2c_addr, reg_h+1)
        val = (hi<<8)|lo
        if val >= 0x8000: val = -((65535 - val) + 1)
        return val

    def _loop_i2c(self):
        period = 1.0/max(self.rate_hz,1.0)
        g_ms2 = 9.80665
        d2r = math.pi/180.0
        while not self._stop.is_set():
            try:
                axr = self._read_word(self.MPU6050_ACCEL_XOUT_H)
                ayr = self._read_word(self.MPU6050_ACCEL_XOUT_H+2)
                azr = self._read_word(self.MPU6050_ACCEL_XOUT_H+4)
                gxr = self._read_word(self.MPU6050_GYRO_XOUT_H)
                gyr = self._read_word(self.MPU6050_GYRO_XOUT_H+2)
                gzr = self._read_word(self.MPU6050_GYRO_XOUT_H+4)

                ax = (axr/self.ACCEL_SENS)*g_ms2
                ay = (ayr/self.ACCEL_SENS)*g_ms2
                az = (azr/self.ACCEL_SENS)*g_ms2
                gx = (gxr/self.GYRO_SENS)*d2r
                gy = (gyr/self.GYRO_SENS)*d2r
                gz = (gzr/self.GYRO_SENS)*d2r

                self._publish(ax, ay, az, gx, gy, gz)
            except Exception as e:
                self.get_logger().warn(f"I2C read error: {e}")
            time.sleep(period)

    # ---------- Common publish ----------
    def _publish(self, ax, ay, az, gx, gy, gz):
        msg = Imu()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        self.pub_raw.publish(msg)
        self.pub_sync.publish(msg)

    def destroy_node(self):
        self._stop.set()
        try:
            if hasattr(self, 'ser') and self.ser: self.ser.close()
            if hasattr(self, 'bus') and self.bus: self.bus.close()
        except Exception: pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SensorsImuBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f"[sensors_imu_bridge] Fatal: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
