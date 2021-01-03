#!/usr/bin/python

from logging import error
import rospy
import math
import sys
import numpy as np
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time


class Falling_Object():

    def __init__(self):

        self.bridge = CvBridge()

        #Publisher
        self.pub = rospy.Publisher("/detection/moving_object", Image, queue_size=10)

        #self.sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.imageCallback, queue_size=1)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1)


    
    def imageCallback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)


            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            edge = cv2.Canny(cv_image_gray, 100,200)

            try:
                
                dif_img = cv2.absdiff(self.previous_frame, edge)
                self.previous_frame = edge

                #2 iterations
                dif_img_blur = cv2.GaussianBlur (dif_img, (31, 31), 0)
                dif_img_blur = cv2.GaussianBlur (dif_img_blur, (11, 11), 0)

                ret, test = cv2.threshold(dif_img_blur, 25, 255, 0)

                contours, hierarchy = cv2.findContours(test, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

                cv2.drawContours(cv_image, contours, -1, (0,255,0), 1)

                #image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

                self.pub.publish(image_message)
                time.sleep(0.01)


            except:
                self.previous_frame = edge

            
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