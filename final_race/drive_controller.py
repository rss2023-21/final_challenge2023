#!/usr/bin/env python

import rospy
import numpy as np
import math

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/track_pos", Float32MultiArray,
            self.track_pos_callback)

        DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
        #DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        # self.error_pub = rospy.Publisher("/parking_error",
        #     ParkingError, queue_size=10)

        
        #Adjust to change parking distance and speed constant
        self.parking_distance = 0 # .75  # meters; try playing with this number!
        self.left_pos = -0.5
        self.right_pos = 0.5
        self.angle = 0
        self.speed_constant = 0.25

    def track_pos_callback(self, msg):
        self.left_pos = msg.data[0]
        self.right_pos = msg.data[1]
        self.angle = msg.data[2]

        drive_cmd = AckermannDriveStamped()

        dist_error = (self.left_pos + self.right_pos) / 2.0
        angle_error = self.angle

        kP = 1
        kD = 0
        kP_angle = 0
        
        drive_angle = np.clip(-kP_angle * angle_error + (-kP) * dist_error, -0.34, 0.34)  # + kD * (dist_error - self.prev_error)
        drive_speed = self.speed_constant
            
        
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.steering_angle = drive_angle
        drive_cmd.drive.speed = drive_speed 

        rospy.loginfo_throttle(0.5, [dist_error, angle_error])
        rospy.loginfo_throttle(0.5, drive_angle)
        self.drive_pub.publish(drive_cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

