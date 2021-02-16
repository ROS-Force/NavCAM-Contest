#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import cv2

import ctypes

from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import BoundingBox
from object_detection.msg import BoundingBoxes
import rospkg

class Yolo_Detection():
    CONFIDENCE_THRESHOLD = 0.3
    NMS_THRESHOLD = 0.4

    def __init__(self):

        self.bridge = CvBridge()
        cuda = True

        libnames = ('libcuda.so', 'libcuda.dylib', 'cuda.dll')
        for libname in libnames:
            try:
                cuda = ctypes.CDLL(libname)
                print(cuda)
            except OSError:
                continue
            else:
                break
        else:
            cuda = False
            print("CUDA no found, loading light YOLO model...")

        abs_path = rospkg.RosPack().get_path("object_detection") + "/cfg/"
        
        if cuda:
            self.net = cv2.dnn.readNetFromDarknet(abs_path + "yolov4.cfg", abs_path + "yolov4.weights")
        else:
            self.net = cv2.dnn.readNetFromDarknet(abs_path + "yolov3-tiny.cfg", abs_path + "yolov3-tiny.weights")

        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA) 
        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(scale=0.00392, size=(320,320), mean=(0, 0, 0), swapRB=True, crop=False)
        self.classes = []
        with open(abs_path + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layers_names = self.net.getLayerNames()
        self.output_layers = [layers_names[i[0]-1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))


        #Publisher
        self.pub = rospy.Publisher("/detection/yolo/objects_image", Image, queue_size=1)
        self.pub_bbox = rospy.Publisher("/detection/yolo/bboxes", BoundingBoxes, queue_size=10)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
    
    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            classes, scores, boxes = self.model.detect(cv_image, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
            cv_image_with_labels = self.draw_labels(boxes, classes, scores, cv_image)
            image_message = self.bridge.cv2_to_imgmsg(cv_image_with_labels, encoding=data.encoding)
            self.pub.publish(image_message)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def draw_labels(self, boxes, classes, scores, img): 
        font = cv2.FONT_HERSHEY_PLAIN
        bbox = BoundingBox()
        
        list_tmp = []

        # iteracao em paralelo
        for (classid, score, box) in zip(classes, scores, boxes):
            label = "%s : %f" % (self.classes[classid[0]], score)
            color = self.colors[int(classid) % len(self.colors)]
            cv2.rectangle(img, box, color, 2)
            cv2.putText(img, label, (box[0], box[1] - 10), font, 1, color, 1)
            
                
            # juntar o score a bbox
            box = box.tolist()
            box[2] = box[0] + box[2]
            box[3] = box[1] + box[3]

            bbox.xmin = box[0]
            bbox.ymin = box[1]
            bbox.xmax = box[2]
            bbox.ymax = box[3]
            bbox.score = float(score)
            bbox.id = int(classid)
            bbox.Class = self.classes[classid[0]]

            list_tmp.append(bbox)

        self.publishBbox(list_tmp)
        
        return img

    def publishBbox(self, list_bbox):
        
        bboxes = BoundingBoxes()
        h = Header()

        h.stamp = rospy.Time.now()
        h.frame_id = "Yolo Frame"

        bboxes.header = h

        bboxes.bounding_boxes = list_bbox
        
        self.pub_bbox.publish(bboxes)


def main():

    rospy.init_node('yolo_detection')
    hd = Yolo_Detection()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")