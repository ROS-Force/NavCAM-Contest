#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import sys

import ctypes

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_msgs.msg import Object
from object_detection.msg import BoundingBox, BoundingBoxes
import rospkg

class Yolo_Detection():

    def __init__(self):

        #Initialize variables
        self.bridge = CvBridge()

        self.YOLO_MODEL = rospy.get_param("~yolo_model", "yolov4")
        self.YOLO_LIGHT_MODEL = rospy.get_param("~yolo_light_model", "yolov3-tiny")
        self.output_rgb = rospy.get_param("~show_output", True)

        #Model Variables
        self.size = rospy.get_param("~model_size", (320, 320))              #New input size.
        self.mean = rospy.get_param("~model_mean", (0, 0, 0))               #Scalar with mean values which are subtracted from channels.
        self.scale = rospy.get_param("~model_scale", 0.00392)               #Multiplier for frame values. 
        self.swapRB = rospy.get_param("~model_swapRGB", True)               #Flag which indicates that swap first and last channels.
        self.crop = rospy.get_param("~model_crop", False)                   #Flag which indicates whether image will be cropped after resize or not. blob(n, c, y, x) = scale * resize( frame(y, x, c) ) - mean(c) )
        self.CONFIDENCE_THRESHOLD = rospy.get_param("~conf_threshold", 0.3) #A threshold used to filter boxes by confidences.
        self.NMS_THRESHOLD = rospy.get_param("~nms_threshold", 0.4)         #A threshold used in non maximum suppression. 

        cuda = True

        #Check if this computer have CUDA libraries, loads a light model in case of missing CUDA library. 
        # This light model decreases accuracy but improves frame rate
        
        libnames = ('libcuda.so', 'libcuda.dylib', 'cuda.dll')
        for libname in libnames:
            try:
                cuda = ctypes.CDLL(libname)
            except OSError:
                continue
            else:
                break
        else:
            cuda = False


        abs_path = rospkg.RosPack().get_path("object_detection") + "/cfg/" # Gets the path to folder where the models are
        
        try:
            if cuda:
                self.net = cv2.dnn.readNetFromDarknet(abs_path + self.YOLO_MODEL +".cfg", abs_path + self.YOLO_MODEL + ".weights")
                rospy.loginfo("CUDA was found, loading %s model" % self.YOLO_MODEL);
            else:
                self.net = cv2.dnn.readNetFromDarknet(abs_path + self.YOLO_LIGHT_MODEL + ".cfg", abs_path + self.YOLO_LIGHT_MODEL + ".weights")
                rospy.loginfo("CUDA was NOT found, loading %s model" % self.YOLO_LIGHT_MODEL);
        except:
            rospy.logerr("Failed to find the models: %s and %s. Please ensure that the .cnf and .weights files are inside object_detection/cnf folder and/or you're calling the right model name" % (self.YOLO_MODEL, self.YOLO_LIGHT_MODEL))
            sys.exit()



        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA) 
        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(scale=self.scale, size=self.size, mean=self.mean, swapRB=self.swapRB, crop=self.crop)
        self.classes = []

        #read the coco file (where the class names are stored)
        with open(abs_path + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layers_names = self.net.getLayerNames()
        self.output_layers = [layers_names[i[0]-1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        #Subscriber
        self.sub = rospy.Subscriber("image", Image, self.imageCallback, queue_size=1, buff_size=2**24)

        #Publisher
        self.pub = rospy.Publisher("output_image", Image, queue_size=1)
        self.pub_bbox = rospy.Publisher("bounding_boxes", BoundingBoxes, queue_size=10)
        #self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
    
    def imageCallback(self, data): #Function that runs when an image arrives
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2
            h = Header()
            #Create a Time stamp
            h.stamp = rospy.Time.now()
            h.frame_id = "Yolo Frame"

            classes, scores, boxes = self.model.detect(cv_image, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD) #Runs the YOLO model
            cv_image_with_labels = self.draw_labels(boxes, classes, scores, cv_image, h) #function that publish/draws the bounding boxes

            if self.output_rgb:
                image_message = self.bridge.cv2_to_imgmsg(cv_image_with_labels, encoding=data.encoding) #Convert back the image to ROS format
                self.pub.publish(image_message) #publish the image (with drawn bounding boxes)
        
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def draw_labels(self, boxes, classes, scores, img, header): 
        font = cv2.FONT_HERSHEY_PLAIN
        bbox = BoundingBox()
        list_tmp = []

        for (classid, score, box) in zip(classes, scores, boxes):
            label = "%s : %f" % (self.classes[classid[0]], score)
            color = self.colors[int(classid) % len(self.colors)]
            cv2.rectangle(img, box, color, 2) #draws a rectangle in the original image
            cv2.putText(img, label, (box[0], box[1] - 10), font, 1, color, 1) #writes the Class and score in the original image
            
                
            #Change the formar of Bounding Boxes from [xmin, ymin, weight, heigh] to [xmin, ymin, xmax, ymax]
            box[2] = box[0] + box[2]
            box[3] = box[1] + box[3]

            #Create the Bounding Box object
            bbox = BoundingBox()
            bbox.xmin = box[0]
            bbox.ymin = box[1]
            bbox.xmax = box[2]
            bbox.ymax = box[3]
            bbox.score = float(score)
            bbox.id = int(classid)
            bbox.Class = self.classes[classid[0]]

            list_tmp.append(bbox) #append the bounding box to a list with all previous Bounding Box
        
        bboxes = BoundingBoxes()
        bboxes.header = header
        bboxes.bounding_boxes = list_tmp

        self.pub_bbox.publish(bboxes) # Publish the list of Bounding Boxes
        return img



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