import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.safety_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        self.drive_sub = rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/output', AckermannDriveStamped, self.check_stop)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.size_threshold = 1 #TODO: Tune this paramater on the robot so the stop sign is the correct size
        self.stop_time = 1      #TODO: tune how long the robot wants to stop
        
        
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

        see_stop_sign, bounding_box = self.detector.predict(rgb_img);
        
        if (see_stop_sign):
            x_min = bounding_box[0]
            y_min = bounding_box[1]
            x_max = bounding_box[2]
            y_max = bounding_box[3]
            if abs(x_max-x_min) > self.size_threshold or abs(y_max - x_min > self.size_threshold):
                if (not self.see_first):
                    self.stop_sign_1_start_time = rospy.get_rostime().nsecs
                elif (not self.see_second):
                    self.stop_sign_2_start_time = rospy.get_rostime().nsecs
        
         
    def check_stop(self):
        curTime = rospy.get_rostime().nsecs
        if (self.see_first):
            if (abs(curTime - self.stop_sign_1_start_time) <= self.stop_time):
                self.stop_robot()
        
        if (self.see_second):
            if (abs(curTime - self.stop_sign_2_start_time) <= self.stop_time):
                self.stop_robot()
                
    
    def stop_robot(self):
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
