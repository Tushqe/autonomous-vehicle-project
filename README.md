# Autonomous Vehicle Project — ROS2 Stack

**Pipeline:**  
Sensors (LiDAR/Camera/IMU) → Perception (ML + Sync + Fusion) → Localization (ORB-SLAM VIO + EKF) → Planning (Global Planner) → Control (Pure Pursuit).

## Folder Overview
# AV ROS2 Stack (Jetson Nano)

Pipeline: Sensors (LiDAR/Camera/IMU) → Perception (ML on camera, **Sync LiDAR+IMU**, Fusion) → Localization (ORB-SLAM VIO + EKF) → Planning (global) → Control (Pure Pursuit → MCU/CAN).

### Quickstart
```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch launch/bringup_sim.launch.py  # or bringup_real.launch.py
