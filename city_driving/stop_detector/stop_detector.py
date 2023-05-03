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
        
        self.size_threshold = 1 #TODO: Tune this paramater so the robot stops when the stop sign is a certain size
        self.stop_time = 1      #TODO: tune how long the robot should stop once it sees a robot
        
        
        #create state machine
        self.stop_sign_1_start_time = -1
        self.see_first = False
        
        self.stop_sign_2_start_time = -1
        self.see_second = False
        

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        #get output from the staff detector thing
        see_stop_sign, bounding_box = self.detector.predict(rgb_img);

        #if we see a stop sign
        if (see_stop_sign):
            #get sides of the bounding box
            x_min = bounding_box[0]
            y_min = bounding_box[1]
            x_max = bounding_box[2]
            y_max = bounding_box[3]
            
            
            rospy.logerr('X Size: ' + str(x_max - x_min) + ', Y Size: ' + str(y_max - y_min))
            
            
            #if either the width or height is greater than our tuned threshold
            if (abs(y_max - y_min) >= self.size_threshold):
                
                #if have not yet seen one stop sign 
                if (not self.see_first):
                    #record current time as time that first saw first stop sign
                    self.stop_sign_1_start_time = rospy.get_rostime().nsecs
                    self.see_first = True
                
                #if have not yet seen the second stop sign
                elif (not self.see_second):
                    #record current time as time that first saw second stop sign
                    self.stop_sign_2_start_time = rospy.get_rostime().nsecs
                    self.see_second = True
        
         
    def check_stop(self):
        #get current time
        curTime = rospy.get_rostime().nsecs
        
        #if have seen first stop sign
        if (self.see_first):
            #if difference between current time and first time seeing stop sign is less than our stop time
            if (abs(curTime - self.stop_sign_1_start_time) <= self.stop_time):
                #stop the robot
                self.stop_robot()
        
        #repeat the same thing for the second robot
        if (self.see_second):
            if (abs(curTime - self.stop_sign_2_start_time) <= self.stop_time):
                self.stop_robot()
                
    
    def stop_robot(self):
        #just publish the speed and angle of zero to the robot
        drive_stamped = AckermannDriveStamped()
        drive_stamped.header.stamp = rospy.get_time()
        drive_stamped.header.frame_id = "base_link"
        drive_stamped.drive.steering_angle = 0
        drive_stamped.drive.speed = 0
        
        self.safety_pub(drive_stamped)

    
if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
