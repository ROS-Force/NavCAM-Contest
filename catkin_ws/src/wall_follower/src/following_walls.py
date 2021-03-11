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


        self.bridge = CvBridge()
        self.rate = rospy.Rate(100)

        #self.sub_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)

        #self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)

        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)


        while not rospy.is_shutdown():

            

            self.rate.sleep()


    def imageDepthCallback(self, data): #Function that runs when a Depth image arrives
        try:
            self.data_encoding_depth = data.encoding
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2

            self.pixel_height=data.height
            self.pixel_width=data.width
            rospy.loginfo(self.cv_image_depth[0,0])
            rospy.loginfo(self.cv_image_depth)
            
            if(self.check_resolution):

                if((self.pixel_width/self.pixel_height)==(4/3)):

                    self.angle_horizontal=75
                    self.angle_height=62
                    self.angle_diagonal=89


                    self.left_squadron=self.pixel_width/4
                    self.rigth_squadron=1*self.pixel_width/4
                    self.top_limit=self.pixel_height/3
                    self.bottom_limit=2*self.pixel_height/3

                    rospy.loginfo("Low Resolution")

                else:

                    self.angle_horizontal=87
                    self.angle_height=58
                    self.angle_diagonal=95

                    rospy.loginfo("High Resolution")

                self.pixel_angle_horizontal_incrementation=self.angle_horizontal/self.pixel_width
                self.pixel_angle_height_incrementation=self.angle_height/self.pixel_height

                self.left_squadron_limit=int(self.left_squadron)
                self.rigth_squadron_limit=int(self.rigth_squadron)
                self.top_limit=int(self.top_limit)    
                self.bottom_limit=int(self.bottom_limit)
                
                self.angle_horizontal=90-(self.angle_horizontal/2)
                self.angle_height=90-(self.angle_height/2)
                          
                self.check_resolution=False




        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

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