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

        IoU_THRESHOLD = 0.1 #Minimum IOU for match
        MIN_HITS = 3 #Minimum number of associated detections before track is initialised
        MAX_AGE = 10 #Maximum number of frames to keep alive a track without associated detections.

        self.list_bbox = np.array([])
        self.list_classid = np.array([])
        self.list_bbox_watershed = np.array([])
        self.list_classid_watershed = np.array([])

        self.existImage = False
        self.existnewBboxYolo = False
        self.existnewBboxWatershed = False

        self.bridge = CvBridge()
        abs_path = rospkg.RosPack().get_path("object_detection") 
    
        si = imp.load_source('sort', abs_path + "/src/sort/" + 'sort.py')
        self.mo_tracker = si.Sort(max_age=MAX_AGE, min_hits=MIN_HITS, iou_threshold=IoU_THRESHOLD) #cria o multiple object tracker com base no codigo do Andrew cenas
        self.mo_tracker_watershed = si.Sort(max_age=20, min_hits=1, iou_threshold=IoU_THRESHOLD)

        self.classes = []
        with open(abs_path + "/cfg/" + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        #Publisher
        self.pub = rospy.Publisher("/tracking/yolo/objects_image", Image, queue_size=10)
        self.pub_water = rospy.Publisher("/tracking/watershed/objects_image", Image, queue_size=10)
        self.pub_bbox = rospy.Publisher("/tracking/yolo/bbox", bbox_msgs, queue_size=10)

        #Subscribers
        self.sub_bbox_yolo = rospy.Subscriber("/detection/yolo/bbox", bbox_msgs,self.bboxCallback, queue_size=10)
        self.sub_bbox_watershed = rospy.Subscriber("/detection/watershed/bbox", bbox_msgs,self.bboxWatershedCallback, queue_size=10)

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
        self.existnewBboxYolo = True

    def bboxWatershedCallback(self,data):
        
        flatten_bbox = data.bbox_list
        flatten_bbox = np.asarray(flatten_bbox) 
        
        self.list_classid_watershed = data.classid

        self.list_bbox_watershed = flatten_bbox.reshape(-1,5)

        self.existnewBboxWatershed = True

    def draw_labels(self, boxes, object_id, classes, img): 
        font = cv2.FONT_HERSHEY_PLAIN
        pub_box= []

        # iteracao em paralelo
        for (classid, id1, box) in zip(classes, object_id, boxes):
            label = "ID : %i" % (int(id1))
            color = self.colors[int(classid) % len(self.colors)]

            pub_box.append(box)

            box[2] = box[2]-box[0]
            box[3] = box[3]-box[1]
            cv2.rectangle(img, box, color, 2)
            cv2.putText(img, label, (box[0], box[1] - 10), font, 1, color, 1)
        return img

    def publishBbox(self, list_bbox, list_classid):
        
        bboxs = bbox_msgs()
        bboxs.bbox_list = sum(list_bbox, [])
        bboxs.classid = list_classid

        self.pub_bbox.publish(bboxs)


    def run(self):

        while not rospy.is_shutdown():

            if self.existImage and self.existnewBboxYolo:
                
                self.existnewBboxYolo = False
                trackers = self.mo_tracker.update(self.list_bbox)

                objects_id = trackers[:,-1]
                objects_box = np.array(trackers[:,0:4], dtype=np.int64)

                img = self.draw_labels(objects_box, objects_id, self.list_classid, self.cv_image)
                image_message = self.bridge.cv2_to_imgmsg(img, encoding = self.data_encoding)

                self.pub.publish(image_message)

            if self.existImage and self.existnewBboxWatershed:
                
                self.existnewBboxWatershed = False
                trackers_watershed = self.mo_tracker_watershed.update(self.list_bbox_watershed)

                objects_id_w = trackers_watershed[:,-1]
                objects_box_w = np.array(trackers_watershed[:,0:4], dtype=np.int64)

                img_w = self.draw_labels(objects_box_w, objects_id_w, self.list_classid_watershed, self.cv_image)
                image_message1 = self.bridge.cv2_to_imgmsg(img_w, encoding = self.data_encoding)
                self.pub_water.publish(image_message1)



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