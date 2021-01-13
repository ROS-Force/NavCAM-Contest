#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import bbox_msgs
import rospkg

class Yolo_Detection():
    CONFIDENCE_THRESHOLD = 0.3
    NMS_THRESHOLD = 0.4

    def __init__(self):

        self.bridge = CvBridge()

        abs_path = rospkg.RosPack().get_path("object_detection") + "/cfg/"
        
        self.net = cv2.dnn.readNetFromDarknet(abs_path + "yolov4.cfg", abs_path + "yolov4.weights")
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
        self.pub_bbox = rospy.Publisher("/detection/yolo/bbox", bbox_msgs, queue_size=10)
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
        # iteracao em paralelo
        for (classid, score, box) in zip(classes, scores, boxes):
                label = "%s : %f" % (self.classes[classid[0]], score)
                color = self.colors[int(classid) % len(self.colors)]
                cv2.rectangle(img, box, color, 2)
                cv2.putText(img, label, (box[0], box[1] - 10), font, 1, color, 1)

                bbox_message = self.prepareBbox(img.shape, box, int(classid))
                self.pub_bbox.publish(bbox_message)
        return img

    def prepareBbox(self, shape, box, id):

        w = int(abs(box[0]- box[2]))
        h = int(abs(box[3]- box[1]))
        x = int(w//2)
        y = int(h//2)
        bbox = bbox_msgs()
        bbox.classid = id
        bbox.data = box.tolist()
        bbox.center = [x,y]
        bbox.size = [w,h]
        bbox.area = w * h
        bbox.perimeter = 2*h + 2*w
        return bbox

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