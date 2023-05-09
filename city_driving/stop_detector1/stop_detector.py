#!/usr/bin/env python

import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from detector import StopSignDetector

sign_threshold = .5 #tune for confidence that it is a stop sign (input to detector)


class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector(sign_threshold)
        self.safety_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        self.drive_sub = rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/output', AckermannDriveStamped, self.check_stop)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        
        self.size_threshold = 80 #TODO: Tune this paramater so the robot stops when the stop sign is a certain size
        self.stop_time = 3 #600000000 #900000000      #TODO: tune how long the robot should stop once it sees a robot
        self.break_time = 3
        
        #create state machine
        self.stop_sign_1_start_time = -1
        self.see_first = False
        self.done_first = False
        
        self.stop_sign_2_start_time = -1
        self.see_second = False
        self.done_second = False

        self.go_second = False

    def callback(self, img_msg):
        #rospy.logerr('hi')
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        #get output from the staff detector thing
        see_stop_sign, bounding_box = self.detector.predict(rgb_img);

        #rospy.logerr('hello')
        #if we see a stop sign
        if (see_stop_sign):
            #get corners of the bounding box
            x_min = bounding_box[0]
            y_min = bounding_box[1]
            x_max = bounding_box[2]
            y_max = bounding_box[3]
            
            
            rospy.logerr('Y Size: ' + str(y_max - y_min))
            
            
            #if either the width or height is greater than our tuned threshold
            if (abs(y_max - y_min) >= self.size_threshold):
                
                #if have not yet seen one stop sign 
                if (not self.see_first):
                    rospy.logerr('saw first sign')
                    #record current time as time that first saw first stop sign
                    self.stop_sign_1_start_time = rospy.get_rostime().secs
                    self.see_first = True
                
                #if have not yet seen the second stop sign
                elif (not self.see_second and self.done_first and self.go_second):
                    rospy.logerr('saw second sign')
                    #record current time as time that first saw second stop sign
                    self.stop_sign_2_start_time = rospy.get_rostime().secs
                    self.see_second = True


                    
        curTime = rospy.get_rostime().secs
        if (self.see_first and not self.done_first):
            if (abs(curTime - self.stop_sign_1_start_time) <= self.stop_time):
                self.stop_robot()

            else:
                rospy.logerr('done first')
                self.done_first = True
        
        if (self.done_first):
            if (abs(curTime - self.stop_sign_1_start_time) > self.stop_time + self.break_time):
                self.go_second = True


        if (self.go_second and self.see_second and self.done_first and not self.done_second):
            if (abs(curTime - self.stop_sign_2_start_time) <= self.stop_time):
                self.stop_robot()

            else:
                rospy.logerr('done second')
                self.done_second = True


    def check_stop(self):
        rospy.logerr('checking stop')
        #get current time
        curTime = rospy.get_rostime().nsecs
        
        #if have seen first stop sign
        if (self.see_first and not self.done_first):
            #if difference between current time and first time seeing stop sign is less than our stop time
            if (abs(curTime - self.stop_sign_1_start_time) <= self.stop_time):
                #stop the robot
                self.stop_robot()
            else:
                self.done_first = True

        #repeat the same thing for the second robot
        if (self.see_second and not self.done_second):
            if (abs(curTime - self.stop_sign_2_start_time) <= self.stop_time):
                self.stop_robot()
            else:
                self.done_second = True
    
    def stop_robot(self):
        rospy.logerr('stopping')
        #just publish the speed and angle of zero to the robot
        drive_stamped = AckermannDriveStamped()
        drive_stamped.header.stamp = rospy.Time.now() #get_time()
        drive_stamped.header.frame_id = "base_link"
        drive_stamped.drive.steering_angle = 0
        drive_stamped.drive.speed = 0
        
        self.safety_pub.publish(drive_stamped)


    
if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
