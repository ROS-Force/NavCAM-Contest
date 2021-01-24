#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import cv2
import imp

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import bbox_msgs
import rospkg

class Sort_tracking():

    def __init__(self):

        self.bridge = CvBridge()
        abs_path = rospkg.RosPack().get_path("object_detection") + "/src/sort/"
    
        si = imp.load_source('sort.sort', abs_path + 'sort.py')
        self.mo_tracker = si.Sort()


        #Publisher
        self.pub = rospy.Publisher("/detection/yolo/objects_image", Image, queue_size=1)
        self.sub_bbox = rospy.Subscriber("/detection/yolo/bbox", bbox_msgs,self.bboxCallback, queue_size=10)
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
    
    def bboxCallback(self,data):
        
        flatten_bbox = data.bbox_list
        flatten_bbox = np.asarray(flatten_bbox) 
        
        classid_list = data.classid

        self.list_bbox = flatten_bbox.reshape(-1,5)

        print(self.list_bbox)
        print(len(classid_list))



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