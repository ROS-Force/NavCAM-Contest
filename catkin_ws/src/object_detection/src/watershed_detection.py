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

class Watershed_Detection():

    def __init__(self):

        self.bridge = CvBridge()
        self.countFrame = 1

        #Publisher
        self.pub = rospy.Publisher("/detection/watershed_blob", Image, queue_size=10)
        #Subscriber
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=10)
    
    def imageCallback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            
        ws = self.watershed(cv_image)
                
        image_message = self.bridge.cv2_to_imgmsg(ws, encoding="passthrough")
        self.pub.publish(image_message)
            

    def watershed(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 3)
        
        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.3*dist_transform.max(),255,0)

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)

        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1

        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0

        markers = cv2.watershed(image,markers)
        #image[markers == -1] = [255,0,0]
        markers[markers == -1] = 31

        img = markers.astype(np.uint16)
        return img


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()

def main():

    rospy.init_node('watershed_detection', anonymous=True)

    mod = Watershed_detection()

    mod.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")