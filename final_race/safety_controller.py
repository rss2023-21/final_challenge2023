#!/usr/bin/env python2

import numpy as np
import math
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# from visualization_tools import *

class SafetyController:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan" #rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation" # rospy.get_param("wall_follower/drive_topic")
    SIDE = 1 #rospy.get_param("wall_follower/side")
    VELOCITY = 1.0 #rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = 0.5 #1 #rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"
    prev_error = None
    
    #instead of 1081, should use len(data.ranges) 
    Ranges = {
            'front': {
            'low': int(np.rint(44*1081/100.0)),
            'high': int(np.rint(56*1081/100.0))
            },
            'left': {
            'low': int(np.rint(57*1081/100.0)),
            'high': int(np.rint(80*1081/100.0))
            },
            'right': {
            'low': int(np.rint(10*1081/100.0)),
            'high': int(np.rint(40*1081/100.0))
            },
            'rear':{
                'left': {
                    'low': int(np.rint(94*1081/100.0)),
                    'high': int(np.rint(99*1081/100.0)),
                },
                'right': {
                    'low': int(np.rint(0*1081/100.0)),
                    'high': int(np.rint(5*1081/100.0))
                }
            }
            
    }

    def radians_to_indexes(self, data, low_rad, high_rad):
        #uses len of ranges data to determin the ranges we want
        incr = data.angle_increment
        #low_rad = (-2*np.pi/3) + (incr*x) x = index (round at end)
        low_idx = np.rint(low_rad - (-2*np.pi/3) / inc) 
        high_idx = np.rint(high_rad - (-2*np.pi/3) / inc)
        return (low_idx, high_idx)
    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
        # self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)

        self.output_sub = rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/output', AckermannDriveStamped, self.safety_callback)
        self.safety_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)

        self.front_dist = 10
        self.left_dist = 10
        self.right_dist = 10
        self.rear_dist = 10

       
    
    def make_mask(self, low, high):
        # generate a mask with ones from `low` to `high`, inclusive
      	mask = np.zeros(1081).astype(bool)
        mask[low:high+1] = 1
        return mask
    
    def compute_distance(self, data, range_mask):
        #rospy.loginfo("data.ranges len: %s, range mask len: %s \n", len(data.ranges), len(range_mask))
        return np.mean(data.ranges[range_mask])
    # def compute_distance(self, data, range_mask):
    #   	"""
    #     range_mask: numpy boolean array of length 100, with 1s where range measurements
    #     should be used in the linear regression
        
    #     returns minimum distance from the robot to the obstacle, or None if there aren't enough
    #     sensor measurements. 
    #     """
        
    #     # get information from the data packet
    #     ranges = data.ranges
    #     angle_min = data.angle_min
    #     angle_max = data.angle_max
    #     angle_increment = data.angle_increment
        
    #     # filter measurements within a certain distance that are also within our angle bounds
    #     mask = (ranges < 4) & (range_mask)
        
    #     # if there aren't enough measurements to use, return None
    #     if np.sum(mask) <= 2:
    #       return np.inf
        
    #     # convert scan data to x, y coordinates
    #     range_angles = np.linspace(angle_min, angle_max, len(ranges))
    #     range_vectors = np.vstack([ranges * np.cos(range_angles), ranges * np.sin(range_angles)]).T
    #     range_vectors = range_vectors[mask]
    #     xs = np.vstack((range_vectors[:, 0], np.ones(range_vectors.shape[0]))).T
    #     ys = range_vectors[:, [1]]

    #     # get slope and intercept using least squares regression
    #     m, c = np.linalg.lstsq(xs, ys)[0]
    #     m = m[0]
    #     c = c[0]
        
    #     # compute projection of robot position (0, 0) on to the least squares regression line
    #     x_mid = (-c * m) / (m**2 + 1)
    #     y_mid = m * x_mid + c
        
    #     # plot the vector from the robot to the closest point on the line for visualization purposes (optional)
    #     # VisualizationTools.plot_line([0, x_mid], [0, y_mid], self.line_pub, frame="/laser")
        
    #     # compute the distance from the robot to the line 
    #     actual_dist = np.sqrt(x_mid**2 + y_mid**2)

        
    #     return actual_dist
    
    
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        
        # self.SIDE = 1

         #================== Safety Controls =======================
        left_mask = self.make_mask(self.Ranges['left']['low'], self.Ranges['left']['high'] )
        right_mask = self.make_mask(self.Ranges['right']['low'], self.Ranges['right']['high'] )
        front_mask = self.make_mask(self.Ranges['front']['low'], self.Ranges['front']['high'] )
        rear_left_mask = self.make_mask(self.Ranges['rear']['left']['low'], self.Ranges['rear']['left']['high'] )
        rear_right_mask = self.make_mask(self.Ranges['rear']['right']['low'], self.Ranges['rear']['right']['high'] )
        
        rear_mask = rear_left_mask | rear_right_mask
      	
        self.compute_distance(msg, left_mask)
        self.front_dist = self.compute_distance(msg, front_mask)
        self.left_dist = self.compute_distance(msg, left_mask)
        self.right_dist = self.compute_distance(msg, right_mask)
        self.rear_dist = self.compute_distance(msg, rear_mask)
        
        # =========================================================
    
    def safety_callback(self, data):
        rospy.loginfo("front dist: %s, rear dist: %s\n, left dist: %s, right dist: %s\n", self.front_dist, self.rear_dist, self.left_dist, self.right_dist )
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle = 0 

        if self.front_dist < 1.5*data.drive.speed or (self.rear_dist < 0.1*data.drive.speed and data.drive.speed<0) or self.left_dist < 0.5*data.drive.speed or self.right_dist < 0.5*data.drive.speed:
            rospy.loginfo("car is IN DANGER \n")
            drive_stop = AckermannDriveStamped()
            drive_stop.header.stamp = rospy.Time.now()
            drive_stop.header.frame_id = "base_link"
            drive_stop.drive.steering_angle = 0
            drive_stop.drive.speed = 0
            self.drive_pub.publish(drive_stop)
        else:
            #self.drive_pub.publish(drive_msg)
            rospy.loginfo("car is SAFE rn \n")
            pass
    
    #def safety_callback(self, data):
        #rospy.loginfo("The safety callback is actually working\n")

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()

          