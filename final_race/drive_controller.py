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
        self.speed_constant = 14.0
        self.start_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.last_dist_error = 0
        self.last_angle_error = 0
        self.last_drive_angle = 0
        self.steering_deriv = 0
        self.total_error = 0
        self.all_updates = []
        self.counter = 0

    def track_pos_callback(self, msg):
        self.counter += 1
        self.left_pos = msg.data[0]
        self.right_pos = msg.data[1]
        self.angle = msg.data[2]

        drive_cmd = AckermannDriveStamped()

        dist_error = (self.left_pos + self.right_pos ) / 2.0 # - 0.1
        angle_error = self.angle
        dT = (rospy.Time.now() - self.last_time).to_sec()
        dist_deriv = (dist_error - self.last_dist_error)  #/ dT
        angle_deriv = (angle_error - self.last_angle_error) # / dT

        self.total_error += dist_error * dT

        # 0.3 0 0 0.3
        # kP = 0.10 # ?
        kP = 0.10
        kI = 0 # 0.01
        kD = 0.0 # 0.03
        kP_angle = 0.3
        kD_angle = 0.02 # 0.04?
        kD_steering = 0.0

        # if abs(kI * self.total_error) > 0.10:
        #     self.total_error = 0


        # kP = 0.0
        # kI = 0.0
        # kD = 0.0
        # kP_angle = 0.0

        feedforward = 0.0 #  -0.02
        # feedforward = 0.015
        
        drive_angle = np.clip(feedforward + -kP_angle * angle_error + (-kP) * dist_error + (-kI) * self.total_error + (-kD) * dist_deriv + (-kD_steering)*self.steering_deriv + (-kD_angle) * angle_deriv, -0.34, 0.34)  #
        # drive_angle = feedforward

        drive_speed = self.speed_constant
        
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.steering_angle = drive_angle

        # safety against close to edges of track 
        drive_speed = drive_speed * np.clip((1.0 - 2.0 * abs(dist_error)), 0.2, 1.0)
        drive_cmd.drive.speed = drive_speed

        updates = [(rospy.Time.now() - self.start_time).to_sec(), dist_error, angle_error, -kP_angle * angle_error, (-kP) * dist_error, (-kI) * self.total_error, (-kD) * dist_deriv, drive_speed, (-kD_steering)*self.steering_deriv, (-kD_angle) * angle_deriv]
        self.all_updates.append(updates)

        rospy.loginfo_throttle(0.5, updates)
        # rospy.loginfo_throttle(0.5, [dist_error, angle_error])
        # rospy.loginfo_throttle(0.5, drive_angle)

        if (self.counter % 50) == 0:
            all_updates = np.array(self.all_updates)
            np.save('/home/racecar/racecar_ws/src/final_race/all_updates.npy', all_updates)


        self.last_time = rospy.Time.now()
        self.last_dist_error = dist_error
        self.last_angle_error = angle_error
        self.steering_deriv = drive_angle - self.last_drive_angle 
        self.last_drive_angle = drive_angle
        self.drive_pub.publish(drive_cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.loginfo('started')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

