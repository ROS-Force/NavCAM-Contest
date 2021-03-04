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

        self.bridge = CvBridge()
        
        self.pub_velocities=rospy.Publisher("cmd_vel", Twist ,queue_size=10)

        self.sub_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)

        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)
    

    def imageDepthCallback(self, data): #Function that runs when a Depth image arrives
        try:
            self.data_encoding_depth = data.encoding
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2
            rospy.loginfo(self.cv_image_depth)

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