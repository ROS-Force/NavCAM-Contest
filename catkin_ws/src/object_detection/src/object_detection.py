#!/usr/bin/python

import rospy
import math
import sys
import numpy as np
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Object_Detection():

    def __init__(self):

        self.bridge = CvBridge()
        self.load_yolo

        abs_path = "/home/mjpc13"

        self.net = cv2.dnn.readNet(abs_path + "/Yolo/yolov3-tiny.weights", abs_path + "/Yolo/yolov3-tiny.cfg")
        self.classes = []
        with open(abs_path+"/Yolo/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layers_names = self.net.getLayerNames()
        self.output_layers = [layers_names[i[0]-1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))


        #Publisher
        self.pub = rospy.Publisher("/detection/objects_image", Image, queue_size=10)

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)


    
    def imageCallback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            height, width, channels = cv_image.shape
            #blob, outputs = self.detect_objects(cv_image, self.net, self.output_layers)
            blob, outputs = self.detect_objects(cv_image)

            boxes, confs, class_ids = self.get_box_dimensions(outputs, height, width)
            self.draw_labels(boxes, confs, self.colors, class_ids, self.classes, cv_image)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()

    def load_yolo(self):
        pass


    def detect_objects(self,frame):

        blob = cv2.dnn.blobFromImage(frame, scalefactor=0.00392, size=(320,320), mean=(0, 0, 0), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        return blob, outputs

    def get_box_dimensions(self, outputs, height, width):
        boxes = []
        confs = []
        class_ids = []
        for output in outputs:
            for detect in output:
                scores = detect[5:]
                #print(scores)
                class_id = np.argmax(scores)
                conf = scores[class_id]
                if conf > 0.3:
                    center_x = int(detect[0] * width)
                    center_y = int(detect[1] * height)
                    w = int(detect[2] * width)
                    h = int(detect[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confs.append(float(conf))
                    class_ids.append(class_id)
	return boxes, confs, class_ids



    def draw_labels(self, boxes, confs, colors, class_ids, classes, img): 
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[i]
                cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                cv2.putText(img, label, (x, y - 5), font, 1, color, 1)

        image_message = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.pub.publish(image_message)





def main():

    rospy.init_node('object_detect', anonymous=True)

    hd = Object_Detection()

    hd.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")