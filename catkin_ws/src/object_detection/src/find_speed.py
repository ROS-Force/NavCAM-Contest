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
        
        speed = self.computeSpeed()

        if not speed.size == 0:
            self.publishSpeed(speed)
        
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
            return np.array([])
        
        else:
            deltat = (self.new_time - self.previous_time)
            deltat = deltat.to_sec()

            if self.updated_center.size == self.previous_center.size or len(self.previous_center.shape) == 1:
                
                try:
                    speed = self.updated_center[:,:3] - self.previous_center[:,:3]
                    speed = speed*10**(-3)/deltat
                    speed = np.column_stack((speed, self.previous_center[:,3]))

                except IndexError as ie:
                    try:
                        speed = self.updated_center[:3] - self.previous_center[:3]
                        speed = speed*10**(-3)/deltat
                        speed = np.append(speed, self.previous_center[3])
                
                    except ValueError as ve:
                        speed = self.updated_center[0,:3] - self.previous_center[:3]
                        speed = speed*10**(-3)/deltat
                        speed = np.append(speed, self.previous_center[3])
            else:
                n = self.previous_center.shape[0]
                speed = self.updated_center[:n,:3] - self.previous_center[:,:3]
                speed = speed*10**(-3)/deltat
                speed = np.column_stack((speed, self.previous_center[:,3]))
            
            return speed


    def publishSpeed(self, speed):
        vector = Vector3()
        msg = object_speed()
        if len(speed.shape) == 1:
            vector.x = speed[0]
            vector.y = speed[1]
            vector.z = speed[2]
            id = int(speed[3])
            msg.velocity = vector
            msg.id = id
            self.pub.publish(msg)

        else:
            for row in speed:
                vector.x = row[0]
                vector.y = row[1]
                vector.z = row[2]
                id = int(row[3])
            
                msg.velocity = vector
                msg.id = id
                self.pub.publish(msg)

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