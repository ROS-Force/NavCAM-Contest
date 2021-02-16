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

        self.previous_center = np.array([])
        self.updated_center = np.array([])
        self.deslocamento = np.array([])
        self.previous_time = rospy.Time.now()
        
        #Publisher
        self.pub = rospy.Publisher("/tracking/speed", object_speed, queue_size=10)

        #Subscribers
        self.sub = rospy.Subscriber("/tracking/yolo/center", real_center, self.centerCallback, queue_size=10)


#Callbacks
    def centerCallback(self, data):
        
        self.new_time = data.header.stamp
        array = data.data
        array = np.asarray(array)
        
        self.updatePositions(array.reshape(-1,4))
        
        self.computeSpeed()
        
        self.previous_center = np.copy(self.updated_center)
        self.previous_time = self.new_time

    def updatePositions(self, new_data):
        
        for id in new_data:

            if self.previous_center.size == 0: #lista esta vazia
                self.updated_center = id

            else:
                if id[3] in self.previous_center:    # O ID ja existe
                    
                    try:
                        itemindex = np.where(self.previous_center[:, -1]==id[3])
                        self.updated_center[itemindex[0]] = id
                    
                    except: # Para apanhar o erro de a updated_center ser 1D
                        self.updated_center = id
                    
                    

                else:       # O ID ainda nao existe
                    self.updated_center = np.vstack((self.updated_center, id) )
                    

    def computeSpeed(self):
        
        if self.previous_center.size == 0:
            pass
        else:
            deltat = (self.new_time - self.previous_time)

            if self.updated_center.size == self.previous_center.size or len(self.previous_center.shape) == 1:
                
                try:
                    speed = self.updated_center[:,:3] - self.previous_center[:,:3]
                    #speed = np.hstack((speed, self.previous_center[:,3]) )
                
                except IndexError as ie:
                    try:
                        speed = self.updated_center[:3] - self.previous_center[:3]
                        #speed = np.hstack((speed, self.previous_center[3]) )
                
                    except ValueError as ve:
                        speed = self.updated_center[0,:3] - self.previous_center[:3]
                        #speed = np.hstack((speed, self.previous_center[3]) )
            else:
                n = self.previous_center.shape[0]
                speed = self.updated_center[:n,:3] - self.previous_center[:,:3]
                print(speed)
                #speed = np.hstack((speed, self.previous_center[:,3]) )




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