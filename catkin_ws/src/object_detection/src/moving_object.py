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


class Moving_Object():

    def __init__(self):

        self.bridge = CvBridge()
        self.countFrame = 1;

        #Publisher
        self.pub = rospy.Publisher("/detection/moving_object", Image, queue_size=10)
        self.pub_depth = rospy.Publisher("/detection/moving_object_depth", Image, queue_size=10)

        self.sub_depth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.imageCallbackDepth, queue_size=10)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=10)


    
    def imageCallback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            #edge = cv2.Laplacian(cv_image_gray,cv2.CV_8UC1)
            edge = cv2.Canny(cv2.GaussianBlur (cv_image_gray, (7, 7), 0), 120,200)

            try:
                
                dif_img = cv2.absdiff(self.previous_frame, edge)
                self.previous_frame = edge


                dif_img_blur = cv2.GaussianBlur (dif_img, (9, 9), 0)
                ret, test = cv2.threshold(dif_img_blur, 25, 255, 0)

                contours, hierarchy = cv2.findContours(test, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
                cv2.drawContours(cv_image, contours, -1, (0,255,0), 1)
                    

                image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                self.pub.publish(image_message)

            except:
                    
                self.previous_frame = edge
            
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
        

    def imageCallbackDepth(self, data):


        try:
            new_data = self.bridge.imgmsg_to_cv2(data, data.encoding)

            ret, high_thr = cv2.threshold(new_data, 6000, 255, 1)
            ret, low_thr = cv2.threshold(new_data, 20, 255, 0)
            #img = cv2.bitwise_and(new_data, new_data, mask = test)
            cv_image = cv2.convertScaleAbs(new_data)


            try:

                
                dif_img = cv2.absdiff(self.previous_frame_depth, cv_image)
                self.previous_frame_depth = cv_image
                dif_img_blur = cv2.GaussianBlur (dif_img, (21, 21), 0)
                

                image_message = self.bridge.cv2_to_imgmsg(low_thr, encoding="passthrough")

                self.pub_depth.publish(image_message)

            except:
                self.previous_frame_depth = cv_image

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

    mod = Moving_Object()

    mod.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")