#!/usr/bin/python


import rospy
import numpy as np
import cv2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import BoundingBox, BoundingBoxes
import random

class Watershed_Detection():

    def __init__(self):

        self.bridge = CvBridge()
        self.countFrame = 1

        self.threshold = rospy.get_param("threshold", 7000)

        self.cv_image = None
        #self.data_enconding = None

        #Publisher
        self.pub_blob = rospy.Publisher("/detection/watershed/blob", Image, queue_size=10)
        self.pub_image = rospy.Publisher("/detection/watershed/objects_image", Image, queue_size=10) 
        self.pub_bbox = rospy.Publisher("/detection/watershed/bboxes", BoundingBoxes, queue_size=10)
        
        #Subscriber
        self.sub = rospy.Subscriber("/image", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        

    def imageCallback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        #self.data_enconding = data.enconding
        
        h = Header()
        #Create a Time stamp
        h.stamp = rospy.Time.now()
        h.frame_id = "Yolo Frame"
        img = self.cv_image
        
        #Publica a imagem com os quadrados verdes desenhados
        image_message = self.bridge.cv2_to_imgmsg(img, encoding=data.encoding)
        self.pub_image.publish(image_message)


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()



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