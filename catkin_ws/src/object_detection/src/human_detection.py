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


class Human_Detection():

    def __init__(self):

        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        #Publisher
        self.pub = rospy.Publisher("/detection/human_image", Image, queue_size=10)

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)


    
    def imageCallback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            boxes, weights = self.hog.detectMultiScale(cv_image_gray, winStride=(8,8))

            boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

            for (xA, yA, xB, yB) in boxes: # display the detected boxes in the colour picture
                cv2.rectangle(cv_image, (xA, yA), (xB, yB),(0, 255, 0), 2)

            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

            self.pub.publish(image_message)

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

    rospy.init_node('human_detect', anonymous=True)

    hd = Human_Detection()

    hd.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")