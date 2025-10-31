#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('use_gst', False)
        self.declare_parameter('gst_pipeline', '')
        self.declare_parameter('flip', False)

        p = lambda n: self.get_parameter(n).get_parameter_value
        self.topic     = p('topic')().string_value
        self.frame_id  = p('frame_id')().string_value
        self.width     = p('width')().integer_value
        self.height    = p('height')().integer_value
        self.fps       = p('fps')().integer_value
        self.device_id = p('device_id')().integer_value
        self.use_gst   = p('use_gst')().bool_value
        self.gst_pipe  = p('gst_pipeline')().string_value
        self.flip      = p('flip')().bool_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.topic, 10)

        self.cap = self._open_capture()
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            raise RuntimeError('Camera open failed')

        if not self.use_gst:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS,          self.fps)

        self.timer = self.create_timer(max(1.0/float(max(self.fps,1)), 0.001), self._tick)
        self.get_logger().info(f'CameraNode -> {self.topic} @ {self.width}x{self.height} {self.fps}fps')

    def _open_capture(self):
        if self.use_gst:
            pipeline = (self.gst_pipe.strip() or
                        'nvarguscamerasrc sensor-id=0 ! '
                        f'video/x-raw(memory:NVMM), width={self.width}, height={self.height}, '
                        f'framerate={self.fps}/1, format=NV12 ! '
                        'nvvidconv ! video/x-raw, format=BGRx ! '
                        'videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false')
            self.get_logger().info(f'GStreamer pipeline:\n{pipeline}')
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self.get_logger().info(f'Opening USB /dev/video{self.device_id}')
            return cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed — skipping')
            return
        if self.flip:
            frame = cv2.flip(frame, 0)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if getattr(self, 'cap', None):
                self.cap.release()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'[CameraNode] Fatal: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
