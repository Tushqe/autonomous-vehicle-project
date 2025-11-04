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
        self.lower_orange_bgr = np.array([])  # Complete with appropriate values
        self.upper_orange_bgr = np.array([])
        
        # BGR for White (allowing for shadows/gray)
        self.lower_white_bgr = np.array([])
        self.upper_white_bgr = np.array([])

        # Kernel for morphological operations (for noise reduction)
        self.morph_kernel = np.ones((5, 5), np.uint8)
        
        self.previous_lane_center_x = None
        self.previous_lane_center_y = None
        self.previous_cx_orange = None
        self.previous_cx_white = None
        self.no_lane_count = 0

        # Lane Width Memory to Handle Sharp Turns ---
        # Initial guess for lane width in pixels. This will be updated
        # automatically whenever both lines are detected.
        self.DEFAULT_LANE_WIDTH_PX = 700 
        self.last_known_lane_width = self.DEFAULT_LANE_WIDTH_PX

        # Create Subscriber and Publishers ---
        # Subscriber: raw camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publisher: processed image
        self.image_pub = self.create_publisher(
            Image,
            '/image_processed',
            10
        )
        self.point_pub = self.create_publisher(
            PointStamped,
            '/lane_point',
            10
        )
        
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
        roi_y = int(height*0.8) 
        roi = cv_image[roi_y:height, 0:width] # bottom half of image

        # Detect Lines using OpenCV BGR Masking  and optionally cv2.morphologyEx ---
        # convert form BGR to HSV(more robust when lighting changes)
        hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        
        # define color ranges for orange and white lanes
        #orange
        lower_orange = np.array([10,100,150])
        upper_orange = np.array([25,255,255])

        # white
        lower_white = np.array([0,0,180])
        upper_white = np.array([179,60,225])
        #upper_white = np.array([180,65,225])
        #threshold, binary masks white pixels where the lanes are, black everywhere else
        mask_orange = cv2.inRange(hsv,lower_orange,upper_orange)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        #noise, Opening removes small white specks(false positives) 
        # Can add closing to fill in the samll holes inside the lane area 
        #mask_orange = cv2.morphologyEx(mask_orange,cv2.MORPH_OPEN, self.morph_kernel)
        #mask_white = cv2.morphologyEx(mask_white,cv2.MORPH_OPEN, self.morph_kernel)

        #mask_orange = cv2.morphologyEx(mask_orange,cv2.MORPH_CLOSE, self.morph_kernel)
        #mask_white = cv2.morphologyEx(mask_white,cv2.MORPH_CLOSE, self.morph_kernel)

        # Find Centroids ---
        # use momments which will compute a set of spaital and 
        # central moments, mathematical properteis that describe the 
        # shape and distribution of white pixel, nonzero ina  binary imagee
        # more on the opencv website
        # m00 area sum of all white pixels
        # m10 sum of x -coord
        # m01 sum of y -coord
        cc_orange = cv2.connectedComponentsWithStats(mask_orange, connectivity=8, ltype=cv2.CV_32S)
        cc_white = cv2.connectedComponentsWithStats(mask_white, connectivity=8, ltype=cv2.CV_32S)
        #numlabel_white, label_white, stats_white, centroid_white = cv2.connectedComponentsWithStats(mask_white,connectivity=8,ltype=cv2.CV_32S)
        detected_orange = False
        detected_white = False
        #cx_orange = None
        #cy_orange = None
        #cx_white = None
        #cy_white = None
         #check out cv.connectedcomponentswithstats 
        minpix = 30
        cx_orange = cy_orange = None
        if cc_orange[0] > 1:
            # skip label 0 the background, now find the largest component by area
            sorted_indices = np.argsort(cc_orange[2][1:,cv2.CC_STAT_AREA]) + 1
            centroid_orange = None
            for idx in sorted_indices[::-1]:
                if cc_orange[2][idx,cv2.CC_STAT_AREA] > minpix:
                    cx, cy = cc_orange[3][idx]
                    cx_orange = int(cx)
                    cy_orange = int(cy) + roi_y
                    detected_orange = True
                    
                    cv2.circle(cv_image,(cx_orange,cy_orange),radius=12,color=(255,0,0),thickness=10)
                    break
        
            
            
            #largest_index = 1 + np.argmax(stats_orange[1:, cv2.CC_STAT_AREA])
            #cx_orange = int(centroid_orange[largest_index][0])
            #cy_orange = int(centroid_orange[largest_index][1]) + roi_y
            #detected_orange = True

        
            # skip label 0 the background, now find the largest component by area
        cx_white = cy_white = None
        if cc_white[0] > 1:    
            sorted_indices = np.argsort(cc_white[2][1:, cv2.CC_STAT_AREA]) + 1
            centroid_white = None
            for idx in sorted_indices[::-1]:
                if cc_white[2][idx,cv2.CC_STAT_AREA] > minpix:
                    wx, wy = cc_white[3][idx]
                    cx_white = int(wx)
                    cy_white = int(wy) + roi_y
                    detected_white = True
                    cv2.circle(cv_image,(cx_white,cy_white),radius=12,color=(0,255,0),thickness=10)
                    break
        
                
            
        #largest_index = 1 + np.argmax(stats_white[1:, cv2.CC_STAT_AREA])
        #cx_white = int(centroid_white[largest_index][0])
        #cy_white = int(centroid_white[largest_index][1]) + roi_y
        #detected_white = True
       


        # Define PointStamped message to publish
        
        # Depending on whether orange or white or both or neither lines are detected,
        # compute the target point accordingly 


        # for both lanes detected 

        if not hasattr(self,'prev_cx_orange'):
            self.previous_cx_orange = None
        if not hasattr(self, 'prev_cx_white'):
            self.previous_cx_white = None
        max_jump = 50
        def smooth_jump(prev,curr):
            if prev is None:
                return curr
            if abs(curr-prev) > max_jump:
                return prev
            return curr

        lane_center_x = None
        lane_center_y = None
        
        if detected_orange and detected_white:
            cx_orange = smooth_jump(self.previous_cx_orange, cx_orange)
            cx_white = smooth_jump(self.previous_cx_white, cx_white)
            
            self.last_known_lane_width = int(abs(cx_white - cx_orange))
            lane_center_x = int ((cx_orange + cx_white) // 2)
            lane_center_y = int((cy_orange + cy_white) // 2) 
        # only one lane is detected use last known width to estimate the other lane 
        elif detected_white and not detected_orange:
            #white detected
            #check for last known
            
            if self.previous_cx_orange is not None:
                cx_orange = smooth_jump(self.previous_cx_orange, self.previous_cx_orange)
                lane_center_x = int((cx_orange +cx_white)//2)

            elif self.last_known_lane_width is not None:
                lane_center_x = int(cx_white-self.last_known_lane_width//2)
            else:
                #orange previous is none
                lane_center_x = cx_white 
            lane_center_y = cy_white
            
        elif detected_orange and not detected_white:
            if self.previous_cx_white is not None:
                cx_white = smooth_jump(self.previous_cx_white, self.previous_cx_white)
                lane_center_x = int((cx_orange + cx_white)//2)
            elif self.last_known_lane_width is not None:
                lane_center_x = int(cx_orange + self.last_known_lane_width //2)
            else:
                lane_center_x = cx_orange
            lane_center_y = cy_orange
        
        else:
            lane_center_x = 0.0
            lane_center_y = 0.0
            self.get_logger().info("No lanes detected, publishing")
        
        
        if detected_orange:
            self.previous_cx_orange = cx_orange
        if detected_white:
            self.previous_cx_white = cx_white
        self.get_logger().debug('Output the which lines were detected as a debug message.')
        
        if lane_center_x != 0 and lane_center_y != 0:
            cv2.circle(cv_image,(lane_center_x,lane_center_y),radius=12,color=(0,0,255),thickness=10)

        
        # Publish the detected point
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_rgb_frame"

        point_msg.point.x = float(lane_center_x)
        point_msg.point.y = float(lane_center_y)
        point_msg.point.z = 0.0
        self.point_pub.publish(point_msg)
        
        self.get_logger().debug(f"published lane point at: ({lane_center_x},{lane_center_y})")


        # Publish the processed image for detected center visualization
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image,encoding='bgr8')
        self.image_pub.publish(processed_msg)
        # update previous lane center 
        self.previous_lane_center_x = lane_center_x
        self.previous_lane_center_y = lane_center_y 


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


