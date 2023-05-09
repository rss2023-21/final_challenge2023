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
        self.speed_constant = 2.0
        self.last_time = rospy.Time.now()
        self.last_dist_error = 0
        self.total_error = 0

    def track_pos_callback(self, msg):
        self.left_pos = msg.data[0]
        self.right_pos = msg.data[1]
        self.angle = msg.data[2]

        drive_cmd = AckermannDriveStamped()

        dist_error = (self.left_pos + self.right_pos) / 2.0
        angle_error = self.angle
        dT = (rospy.Time.now() - self.last_time).to_sec()
        dist_deriv = (dist_error - self.last_dist_error) / dT
        self.total_error += dist_error * dT

        kP = 0.2  # ?
        kI = 0
        kD = 0.2  # ?
        kP_angle = 0.4
        
        drive_angle = np.clip(-kP_angle * angle_error + (-kP) * dist_error + (-kI) * self.total_error + (-kD) * dist_deriv, -0.34, 0.34)  # 
        drive_speed = self.speed_constant
        
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.steering_angle = drive_angle
        drive_cmd.drive.speed = drive_speed

        rospy.loginfo_throttle(0.5, [-kP_angle * angle_error, (-kP) * dist_error, (-kI) * self.total_error, (-kD) * dist_deriv])
        # rospy.loginfo_throttle(0.5, [dist_error, angle_error])
        # rospy.loginfo_throttle(0.5, drive_angle)

        self.last_time = rospy.Time.now()
        self.last_dist_error = dist_error
        self.drive_pub.publish(drive_cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

