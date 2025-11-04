#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class DetectNode(Node):
    """
    A ROS2 node to detect lane lines using OpenCV RGB/BGR color masking
    and publish a target point.
    """
    def __init__(self):
        super().__init__('detect_node')
        
        self.bridge = CvBridge()
        
        # Color Ranges for Line Detection in BGR Space ---
        # BGR for Orange
        self.lower_orange_bgr = np.array([0, 80, 180])  # Complete with appropriate values
        self.upper_orange_bgr = np.array([80, 180, 255])
        
        # BGR for White (allowing for shadows/gray)
        self.lower_white_bgr = np.array([180, 180, 180])
        self.upper_white_bgr = np.array([255, 255, 255])

        # Kernel for morphological operations (for noise reduction)
        self.morph_kernel = np.ones((5, 5), np.uint8)
        
        # Lane Width Memory to Handle Sharp Turns ---
        # Initial guess for lane width in pixels. This will be updated
        # automatically whenever both lines are detected.
        self.DEFAULT_LANE_WIDTH_PX = 700 
        self.last_known_lane_width = self.DEFAULT_LANE_WIDTH_PX

        # Create Subscriber and Publishers ---
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/image_processed', 10)
        self.point_pub = self.create_publisher(PointStamped, '/lane_point', 10)

        self.get_logger().info('Lane detection node started (using BGR + Memory for sharp turns).')


    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Processes the image to find the target point.
        """
        try:
            # Convert ROS Image message to OpenCV image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Define Region of Interest (ROI)
        height, width, _ = cv_image.shape

        # roi = 
        #roi_height = int(height * 0.2)  # bottom 20% of image
        if self.last_known_lane_width < 600:
            roi_height = int(height * 0.5)  # look higher up if lane is narrow (turn)
        else:
            roi_height = int(height * 0.2)  # normal
        roi = cv_image[height - roi_height:height, :]
        
        # Detect Lines using OpenCV BGR Masking  and optionally cv2.morphologyEx ---
        mask_orange = cv2.inRange(roi, self.lower_orange_bgr, self.upper_orange_bgr)
        mask_white = cv2.inRange(roi, self.lower_white_bgr, self.upper_white_bgr)

        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, self.morph_kernel)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, self.morph_kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.morph_kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.morph_kernel)

        # Find Centroids ---
        left_centroid = self.get_centroid(mask_orange)
        right_centroid = self.get_centroid(mask_white)

        # Convert ROI coordinates to full-image coordinates
        if left_centroid is not None:
            left_centroid = (int(left_centroid[0]), int(left_centroid[1] + (height - roi_height)))
        if right_centroid is not None:
            right_centroid = (int(right_centroid[0]), int(right_centroid[1] + (height - roi_height)))


        # Define PointStamped message to publish
        lane_center = None
        if left_centroid and right_centroid:
            # both lines detected
            lane_center = ((left_centroid[0] + right_centroid[0]) // 2,
                           (left_centroid[1] + right_centroid[1]) // 2)
            self.last_known_lane_width = abs(right_centroid[0] - left_centroid[0])
        elif left_centroid and not right_centroid:
            # only left line detected
            lane_center = (left_centroid[0] + self.last_known_lane_width // 2, left_centroid[1])
        elif right_centroid and not left_centroid:
            # only right line detected
            lane_center = (right_centroid[0] - self.last_known_lane_width // 2, right_centroid[1])
        else:
            # neither line detected → end of curvy road
            self.publish_point(msg, 0.0, 0.0, 0.0)
            annotated = cv_image.copy()
            cv2.putText(annotated, "NO LANES DETECTED", (30, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            self.publish_image(annotated, msg)
            return
        
        # Depending on whether orange or white or both or neither lines are detected,
        # compute the target point accordingly 

        self.get_logger().debug('Output the which lines were detected as a debug message.')

        # Draw visualization dot ONLY if we have a valid (non-zero) point
        annotated = cv_image.copy()
        cv2.rectangle(annotated, (0, height - roi_height), (width - 1, height - 1), (60, 60, 60), 2)
        if left_centroid:
            cv2.circle(annotated, left_centroid, 6, (0, 140, 255), -1)
        if right_centroid:
            cv2.circle(annotated, right_centroid, 6, (255, 255, 255), -1)
        if lane_center:
            cv2.circle(annotated, lane_center, 8, (0, 0, 255), -1)

        # Publish the detected point
        self.publish_point(msg, lane_center[0], lane_center[1], 0.0)

        # Publish the processed image for detected center visualization
        self.publish_image(annotated, msg)


    # ---------------- Helper functions ---------------- #
    def get_centroid(self, mask):
        """Return centroid (x,y) of the largest contour in mask, or None."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 50:
            return None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    def publish_point(self, img_msg, x, y, z):
        """Publish a PointStamped message."""
        pt = PointStamped()
        pt.header = img_msg.header
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        self.point_pub.publish(pt)

    def publish_image(self, cv_image, img_msg):
        """Publish an annotated image to /image_processed."""
        img_msg_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg_out.header = img_msg.header
        self.image_pub.publish(img_msg_out)


def main(args=None):
    rclpy.init(args=args)
    
    detect_node = DetectNode()
    
    try:
        rclpy.spin(detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        if rclpy.ok():
            detect_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()


