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

        ws_8bit = self.map_uint16_to_uint8(ws)

        contours, hierarchy = cv2.findContours(ws_8bit, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        cv2.drawContours(cv_image, contours, -1, (0,255,0), 1)

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)

            # draw a green rectangle to visualize the bounding rect
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # get the min area rect
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
                # convert all coordinates floating point values to int
            box = np.int0(box)
                # draw a red 'nghien' rectangle
            cv2.drawContours(cv_image, [box], 0, (0, 0, 255))

                # finally, get the min enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(c)
            # convert all values to int
            center = (int(x), int(y))
            radius = int(radius)
                # and draw the circle in blue
            cv_image = cv2.circle(cv_image, center, radius, (255, 0, 0), 2)

        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="8UC3")
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
        markers[markers == -1] = 0
        markers[markers == 1] = 0
        img = markers.astype(np.uint16)
        #unique, counts = np.unique(img, return_counts=True)
        #print(dict(zip(unique, counts)))
        return img


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()

    def map_uint16_to_uint8(self, img, lower_bound=None, upper_bound=None):
        if not(0 <= lower_bound < 2**16) and lower_bound is not None:
            raise ValueError(
                '"lower_bound" must be in the range [0, 65535]')
        if not(0 <= upper_bound < 2**16) and upper_bound is not None:
            raise ValueError(
                '"upper_bound" must be in the range [0, 65535]')
        if lower_bound is None:
            lower_bound = np.min(img)
        if upper_bound is None:
            upper_bound = np.max(img)
        if lower_bound >= upper_bound:
            raise ValueError(
                '"lower_bound" must be smaller than "upper_bound"')
        lut = np.concatenate([
            np.zeros(lower_bound, dtype=np.uint16),
            np.linspace(0, 255, upper_bound - lower_bound).astype(np.uint16),
            np.ones(2**16 - upper_bound, dtype=np.uint16) * 255
        ])
        return lut[img].astype(np.uint8)

def main():

    rospy.init_node('watershed_detection', anonymous=True)

    mod = Watershed_Detection()

    mod.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")