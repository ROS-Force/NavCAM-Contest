#!/usr/bin/python

import rospy
import math
import sys
import numpy as np
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Falling_Object():

    def __init__(self):

        self.bridge = CvBridge()

        #Publisher
        self.pub = rospy.Publisher("/detection/moving_object", Image, queue_size=10)

        self.sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.imageCallback)


    
    def imageCallback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            #cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            cv_image_gray = cv2.GaussianBlur (cv_image, (31, 31), 0)

            try:
                dif_img = cv2.absdiff(self.previous_frame, cv_image_gray)
                self.previous_frame = cv_image_gray
                image_message = self.bridge.cv2_to_imgmsg(dif_img, encoding="passthrough")
                self.pub.publish(image_message)

            except:
                print(type(cv_image_gray))
                self.previous_frame = cv_image_gray

            
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()

def main():

    rospy.init_node('falling_object', anonymous=True)

    fod = Falling_Object()

    fod.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")