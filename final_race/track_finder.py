#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from track_pipeline import get_filtered_lines
from std_msgs.msg import Float32MultiArray

class TrackFinder():
    """
    A class for applying track detection algorithms to images from the ZED camera.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False
        self.left_pos = -0.5
        self.right_pos = 0.5
        self.angle = 0

        # Subscribe to ZED camera RGB frames
        self.info_pub = rospy.Publisher("/track_pos", Float32MultiArray, queue_size=10)
        self.debug_pub = rospy.Publisher("/track_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # cv2.imwrite('outputs/zed_image.png', image)
        new_image, x_intercept_filtered, angle_from_vertical = get_filtered_lines(image)
        if angle_from_vertical is None:
            return
        
        if len(x_intercept_filtered) == 0:
            return

        debug_msg = self.bridge.cv2_to_imgmsg(new_image, "bgr8")
        self.debug_pub.publish(debug_msg)

        # find closest left and right to last left/right
        left_best = 1000
        right_best = 1000
        # rospy.loginfo_throttle(0.5, x
        for x in x_intercept_filtered:
            # rospy.loginfo(0.5, x)
            if abs(x-(-0.5)) < abs(left_best-(-0.5)) and abs(x-(-0.5)) < 0.5:
                left_best = x
            
            if abs(x-(0.5)) < abs(right_best-(0.5)) and abs(x-(0.5)) < 0.5:
                right_best = x
            # if abs(x-self.left_pos) < abs(left_best-self.left_pos):
            #     left_best = x
            
            # if abs(x-self.right_pos) < abs(right_best-self.right_pos):
            #     right_best = x
        
        if left_best == 1000:
            if right_best != 1000:
                left_best = right_best - 1
        
        if right_best == 1000:
            if left_best != 1000:
                right_best = left_best + 1
        
        # update if values are reasonable
        self.left_pos = left_best 
        self.right_pos = right_best 
        # if abs(left_best - self.left_pos) < 0.2:
        #     self.left_pos = left_best
        
        # if abs(right_best - self.right_pos) < 0.2:
        #     self.right_pos = right_best

        self.angle = angle_from_vertical
        # if abs(self.angle - angle_from_vertical) < 0.2:
        #     self.angle = angle_from_vertical

        float_array_msg = Float32MultiArray()
        float_array_msg.data = [self.left_pos, self.right_pos, self.angle]
        self.info_pub.publish(float_array_msg)

        # rospy.loginfo_throttle(0.5, x_intercept_filtered)
        rospy.loginfo_throttle(0.5, [left_best, right_best, self.angle])
        
    
        


if __name__ == '__main__':
    try:
        rospy.init_node('TrackFinder', anonymous=True)
        TrackFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass