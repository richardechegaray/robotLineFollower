#!/usr/bin/env python

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
#import roslib
#roslib.load_manifest('enph353_ros_lab')
import rospy
import sys


KP = 0.055
KD = 0.03
width = 800
height = 800
scan_height = 780

class robot_control:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.callback)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=30)
        self.bridge = CvBridge()
        self.last_err = 0

        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        offset = 0
        frame = robot_control.image_converter(cv_image)
        centre = robot_control.get_centre(frame)
     
        self.speed_controller(centre)
        #cv.circle(frame, (int(centre), 700), 20, (255, 0, 0), -1)

        cv.imshow("Robot", frame)
        cv.waitKey(3) 

    @staticmethod
    def image_converter(cv_image):

        frame = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        ret, frame = cv.threshold(frame, 100, 255, cv.THRESH_BINARY)

        return frame

    @staticmethod
    def get_centre(frame):
        black_1 = 0
        black_pixels = 0
        first_black = False

        for x in range(0,width-1):
            pixel_val = frame[scan_height,x]
            if pixel_val == 0 and black_1 == 0: 
                black_1 = x
                first_black = True
                black_pixels = black_pixels + 1
            elif pixel_val == 0 and first_black:
                black_pixels = black_pixels + 1
            elif pixel_val == 255 and first_black and black_pixels > 10:
                break
          
        centre = black_1 + int(black_pixels/2)

        return centre

    def speed_controller(self, centre):
        
        error = centre - (width/2) 

        p = KP * error
        d = KD * (error - self.last_err)
        self.last_err = error

        velocity = Twist()
        velocity.angular.z = p + d
        velocity.linear.x = 0.4
        self.vel_pub.publish(velocity)

        print(centre)


    
def main(args):
    
    rospy.init_node('image_processor', anonymous=True)
    rc = robot_control()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv.destroyAllWindows

if __name__ == '__main__':
    main(sys.argv)
   
    
