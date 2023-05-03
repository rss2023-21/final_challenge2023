#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visual_servoing.msg import ConeLocation, ConeLocationPixel

def get_homography_matrix():

    #The following collection of pixel locations and corresponding relative
    #ground plane locations are used to compute our homography matrix

    # PTS_IMAGE_PLANE units are in pixels
    # see README.md for coordinate frame description

    ######################################################
    ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
    PTS_IMAGE_PLANE = [[228, 189], [328, 187], [429, 187], [191, 207],
                        [335, 205], [478, 206], [90, 253], [353, 249], 
                        [586, 247]]
    ######################################################

    # PTS_GROUND_PLANE units are in inches
    # car looks along positive x axis with positive y axis to left

    ######################################################
    ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
    PTS_GROUND_PLANE = [[65, 19], [65, 0], [65, -21.5], [45, 19], 
                        [45, 0], [45, -21.5], [25, 19], [25, 0], [25, -21.5]]
    ######################################################

    METERS_PER_INCH = 0.0254


    np_pts_ground = np.array(PTS_GROUND_PLANE)
    np_pts_ground = np_pts_ground * METERS_PER_INCH
    np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

    np_pts_ground = np_pts_ground * 100
    np_pts_ground[:, :, 1] += 200

    # flip x and y
    np_pts_ground = np_pts_ground[:, :, ::-1]

    # print(np_pts_ground)

    np_pts_image = np.array(PTS_IMAGE_PLANE)
    np_pts_image = np_pts_image * 1.0
    np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

    h, err = cv2.findHomography(np_pts_image, np_pts_ground)
    
    return h