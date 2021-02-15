#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import imp
import rospkg
import pyrealsense2 as rs2

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from object_detection.msg import object_speed
from cv_bridge import CvBridge, CvBridgeError
from object_detection.msg import real_center


class Sort_tracking():

    def __init__(self):

        self.previous_center = []
        #Publisher
        self.pub = rospy.Publisher("/tracking/speed", object_speed, queue_size=10)

        #Subscribers
        self.sub = rospy.Subscriber("/tracking/yolo/center", real_center, self.centerCallback, queue_size=10)


#Callbacks
    def centerCallback(self, data):
        
        flatten_data = data.data
        flatten_data = np.asarray(flatten_data) 
        
        self.list_center = flatten_data.reshape(-1,4)

        print(self.list_center)

    def run(self):

        while not rospy.is_shutdown():
            pass


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