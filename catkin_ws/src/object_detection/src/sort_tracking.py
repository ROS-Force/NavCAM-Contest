#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import bbox_msgs
from .sort import Sort
import rospkg

class Sort_tracking():

    def __init__(self):

        self.bridge = CvBridge()
        self.mo_tracker = Sort()


        #Publisher
        self.pub = rospy.Publisher("/detection/yolo/objects_image", Image, queue_size=1)
        self.pub_bbox = rospy.Publisher("/detection/yolo/bbox", bbox_msgs, queue_size=10)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)

    def imageCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            pass
            rate.sleep()


def main():

    rospy.init_node('sort_tracking')
    st = Sort_tracking()
    st.run()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")