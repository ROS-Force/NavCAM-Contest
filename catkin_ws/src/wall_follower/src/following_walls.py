#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from __future__ import print_function
##################################### Header ############################################
""" obstruction.py: Description of the node """
#__author__ = ""
#__credits__ = [""]
#__version__ = "0.0.0"
#__maintainer__ = ""
#__email__ = ""
#########################################################################################
# import any libraries necessary to run script

import rospy
import numpy as np
import rospkg
import pyrealsense2 as rs2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class following_walls():
 
    def __init__(self):


        #Initialize variables
        self.cv_image_depth = None
        self.cv_image = None
        self.intrinsics = None
        self.check_resolution=True
        self.start=0;
        self.left_squadron_limit=0
        self.rigth_squadron_limit=0
        self.top_limit=0
        self.bottom_limit=0


        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

        wall_in_the_right=True
        target_distance=0.01
        wall_found=0

        #Topic subscribers
        self.sub_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)

        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)
        
        

        while not rospy.is_shutdown():

            #after the first topic as processed
            if(self.check_resolution==False):

                #until we find a wall we will be looking to measure distances in both sides of the camera
                if(wall_found==0):
                   
                    self.right_side_measurement()

                    self.left_side_measurement()

                    rospy.loginfo("Right side distance = %s Left side distance = %s" %(self.distance_right,self.distance_left))

                    if(self.distance_left<target_distance):
                        wall_in_the_right=False
                        wall_found=1
                    elif(self.distance_right<target_distance):
                        wall_in_the_right=True
                        wall_found=1

                else:

                    if(wall_in_the_right):
                        
                        self.right_side_measurement()
                        
                        rospy.loginfo("Distance to the right= %f m" %(self.distance_right))

                        if(self.distance_right<=target_distance):

                                #code for response placed here

                                rospy.loginfo("To close")

                    
                    else:
                        
                        self.distance_left=self.left_side_measurement()

                        rospy.loginfo("Distance to the left= %f m" %(self.distance_left))

                        if(self.distance_left<=target_distance):

                             #code for response placed here

                            rospy.loginfo("To close")



                self.rate.sleep()


    def imageDepthCallback(self, data): #Function that runs when a Depth image arrives
        try:
            
            self.data_encoding_depth = data.encoding
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2

            self.pixel_height=data.height
            self.pixel_width=data.width

            #check what kind of image resolution we are working with 
            if(self.check_resolution):

                if((self.pixel_width/self.pixel_height)==(4/3)):

                    self.left_squadron=self.pixel_width/8
                    self.rigth_squadron=7*self.pixel_width/8
                    self.top_limit=2*self.pixel_height/6
                    self.bottom_limit=4*self.pixel_height/6

                    rospy.loginfo("Low Resolution")

                else:
                    
                    self.left_squadron=self.pixel_width/32
                    self.rigth_squadron=31*self.pixel_width/32
                    self.top_limit=8*self.pixel_height/18
                    self.bottom_limit=10*self.pixel_height/18

                    rospy.loginfo("High Resolution")


                self.left_squadron_limit=int(self.left_squadron)
                self.rigth_squadron_limit=int(self.rigth_squadron)
                self.top_limit=int(self.top_limit)    
                self.bottom_limit=int(self.bottom_limit)
                          
                self.check_resolution=False

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    def imageDepthInfoCallback(self, cameraInfo): #Code copied from Intel script "show_center_depth.py". Gather camera intrisics parameters that will be use to compute the real coordinates of pixels
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]

        except CvBridgeError as e:
            print(e)
            return
    
    def left_side_measurement(self):
        i1=self.top_limit
        j1=0
        x_left=[]

        while (i1<=self.bottom_limit):
            while (j1<=self.left_squadron_limit):

                depth = self.cv_image_depth[i1,j1] #Depth of the central pixel
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [i1, j1], depth) # Real coordenates, in mm, of the central pixel

                #Create a vector with the coordinates
                x_left.append(result[0])
                j1=j1+1

            i1=i1+1

        #distance to the left in meters
        self.distance_left=-(sum(x_left)/len(x_left))*10**(-3)
        

    
    def right_side_measurement(self):
        i2=self.top_limit
        j2=self.rigth_squadron_limit
        x_right=[]
        
        while (i2<=self.bottom_limit):
            while (j2<=(self.pixel_width-1)):

                depth = self.cv_image_depth[i2,j2] #Depth of the central pixel
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [i2, j2], depth) # Real coordenates, in mm, of the central pixel

                #Create a vector with the coordinates
                x_right.append(result[0])
                j2=j2+1

            i2=i2+1

        #distance to the right in meters
        self.distance_right=-(sum(x_right)/len(x_right))*10**(-3)


     

def main():

    rospy.init_node('following_walls')
    st = following_walls()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")