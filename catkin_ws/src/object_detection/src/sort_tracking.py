#!/usr/bin/env python

import rospy
import math
import sys
import time
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

        self.list_bbox = np.array([])
        self.list_classid = np.array([])
        self.existImage = False
        self.existBbox = False

        self.bridge = CvBridge()
        abs_path = rospkg.RosPack().get_path("object_detection") 
    
        si = imp.load_source('sort', abs_path + "/src/sort/" + 'sort.py')
        self.mo_tracker = si.Sort() #cria o multiple object tracker com base no codigo do Andrew cenas

        self.classes = []
        with open(abs_path + "/cfg/" + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        #Publisher
        self.pub = rospy.Publisher("/tracking/SORT/objects_image", Image, queue_size=1)
        self.sub_bbox = rospy.Subscriber("/detection/yolo/bbox", bbox_msgs,self.bboxCallback, queue_size=10)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)

    def imageCallback(self, data):
        try:
            self.data_encoding = data.encoding
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.existImage= True

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
    
    def bboxCallback(self,data):
        
        flatten_bbox = data.bbox_list
        flatten_bbox = np.asarray(flatten_bbox) 
        
        self.list_classid = data.classid

        self.list_bbox = flatten_bbox.reshape(-1,5)
        self.existBbox = True

    def draw_labels(self, boxes, object_id, classes, img): 
        font = cv2.FONT_HERSHEY_PLAIN

        # iteracao em paralelo
        for (classid, id1, box) in zip(classes, object_id, boxes):
            label = "%s : %f" % (self.classes[classid], id1)
            color = self.colors[int(classid) % len(self.colors)]
            
            cv2.rectangle(img, box, color, 2)
            cv2.putText(img, label, (box[0], box[1] - 10), font, 1, color, 1)
                
        return img


    def run(self):
        rate = rospy.Rate(10)
        total_time = 0.0

        while not rospy.is_shutdown():

            if self.existImage and self.existBbox:
                start_time = time.time()

                trackers = self.mo_tracker.update(self.list_bbox)

                cycle_time = time.time() - start_time
                total_time += cycle_time

                objects_id = trackers[:,-1]
                objects_box = np.array(trackers[:,0:4], dtype=np.int64)
                

                if trackers.size != 0:
                    img = self.draw_labels(objects_box, objects_id, self.list_classid, self.cv_image)
                    image_message = self.bridge.cv2_to_imgmsg(img, encoding = self.data_encoding)
                    self.pub.publish(image_message)
                else:
                    image_message = self.bridge.cv2_to_imgmsg(self.cv_image, encoding = self.data_encoding)
                    self.pub.publish(image_message)


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