#!/usr/bin/env python

import rospy
import numpy as np
import math

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/track_pos", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        
        #Adjust to change parking distance and speed constant
        self.parking_distance = 0 # .75  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.speed_constant = 0.5

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        drive_cmd = AckermannDriveStamped()
    
        # rospy.loginfo(str(self.relative_x) + ' ' +  str(self.relative_y));

        r = math.sqrt(self.relative_x**2 + self.relative_y**2)
        r = r - self.parking_distance
        
        if (np.abs(r) < .0001):
           r = 0
        
        nu = math.atan2(self.relative_y,self.relative_x)
        if (r < 0):
           nu = 0
        rospy.loginfo_throttle(0.5, str(r) + ' ' + str(nu));

        if (msg.x_pos == 0 and msg.y_pos == 0):
            rospy.loginfo("going straight")
            # just go straight
            drive_speed = 0.5
            drive_angle = 0

        else:
            if (r >= 0):
                drive_speed = min(1, self.speed_constant * r)
            else:
                drive_speed = max(-1, 2*self.speed_constant * r)

            if (r != 0):
                drive_angle = math.atan((2*r*math.sin(nu)/r))
            else:
                drive_angle = 0
            
        
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.steering_angle = drive_angle
        drive_cmd.drive.speed = drive_speed

        # rospy.loginfo((drive_speed)) 

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)
    
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

