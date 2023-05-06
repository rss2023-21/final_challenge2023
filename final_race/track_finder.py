#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from track_pipeline import get_filtered_lines

class TrackFinder():
    """
    A class for applying track detection algorithms to images from the ZED camera.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        # self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        # self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite('outputs/zed_image.png', image)
        # img, filtered_lines = get_filtered_lines(image)
        
        # self.debug_pub.publish(debug_msg)

    
        


if __name__ == '__main__':
    try:
        rospy.init_node('TrackFinder', anonymous=True)
        TrackFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass