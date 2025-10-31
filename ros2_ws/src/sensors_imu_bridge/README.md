# sensors_imu_bridge

ROS 2 (Humble) IMU bridge that reads **I²C (MPU6050)**, **Serial CSV**, **Gazebo**, or **Sim**, and publishes:
- `/imu/data_raw` (sensor_msgs/Imu)
- `/sync/imu` (sensor_msgs/Imu)  ← for your sync node with LiDAR

## Build (on Jetson later)
```bash
cd ~/autonomous-vehicle-project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sensors_imu_bridge
source install/setup.bash
# I2C (MPU6050)
ros2 launch sensors_imu_bridge imu.launch.py mode:=i2c i2c_bus:=1 i2c_address:=104

# Serial CSV
ros2 launch sensors_imu_bridge imu.launch.py mode:=serial serial_port:=/dev/ttyUSB0 serial_baud:=115200

# Gazebo bridge
ros2 launch sensors_imu_bridge imu.launch.py mode:=gazebo gazebo_input_topic:=/imu

# Sim
ros2 launch sensors_imu_bridge imu.launch.py mode:=sim
