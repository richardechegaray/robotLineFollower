import roslib
import sys
import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

d = 0.04
speed = 0.8

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=30)
        self.prev_err = 0

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        height = np.size(frame, 0)
        width = np.size(frame,1)

        roi = frame[height-300:height] #region of interest

        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)

        M = cv.moments(thresh)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        drive_error = width/2 - cX
        velocity = Twist()
        velocity.angular.z = d * drive_error
        velocity.linear.x = speed
        self.vel_pub.publish(velocity)

        print(velocity.linear.x)
        cv.circle(frame, (int(cX), height-100), 20, (0, 255, 0), -1)
        cv.imshow("Robot Camera", frame)
        cv.waitKey(1)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)