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


class Find_Speed():

    def __init__(self):

        self.previous_center = np.array([[1234, 3241, 5634, 1], [65, 6574,234, 0]])
        self.updated_center = np.array([])
        
        #Publisher
        self.pub = rospy.Publisher("/tracking/speed", object_speed, queue_size=10)

        #Subscribers
        self.sub = rospy.Subscriber("/tracking/yolo/center", real_center, self.centerCallback, queue_size=10)


#Callbacks
    def centerCallback(self, data):
        
        self.updated_center = self.previous_center

        array = data.data
        array = np.asarray(array)
        
        self.updatePositions(array.reshape(-1,4))
        
        self.previous_center = self.updated_center
        print(self.updated_center)
        #self.list_positions = array.reshape(-1,4)

        #print(self.list_center)


    def updatePositions(self, new_data):
        
        for id in new_data:

            if id[3] in self.previous_center:

                itemindex = np.where(self.previous_center[:, -1]==id[3])

                self.updated_center[itemindex[0]] = id
            
            



    def run(self):

        while not rospy.is_shutdown():
            pass


def main():

    rospy.init_node('find_speed')
    fs = Find_Speed()
    fs.run()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")