# sensors_camera_node

ROS 2 (Humble) camera publisher for USB/CSI cameras on Jetson. Publishes `sensor_msgs/Image`
on `/camera/image_raw` for your ML node (e.g., YOLOPv2) to subscribe to.

## Build (run on the Jetson later)
```bash
cd ~/autonomous-vehicle-project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sensors_camera_node
source install/setup.bash
ros2 launch sensors_camera_node cam.launch.py device_id:=0 use_gst:=false width:=1280 height:=720 fps:=30
ros2 launch sensors_camera_node cam.launch.py use_gst:=true width:=1280 height:=720 fps:=30

