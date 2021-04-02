#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import sys
import ctypes

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from object_detection.msg import BoundingBox, BoundingBoxes
import rospkg
from geometry_msgs.msg import Vector3

import deeplab.deeplab_ros as dlros

class Yolo_Detection():

    def __init__(self):

        #Initialize variables
        self.bridge = CvBridge()
        
        # create model config 
        modelConfig = dlros.DeepLabModelConfig
        self.graphPath = rospy.get_param("~deeplab_model/inference_graph/path")
        self.graphInput = rospy.get_param("~deeplab_model/inference_graph/input_tensor") 
        self.graphOutput = rospy.get_param("~deeplab_model/inference_graph/output_tensor")
        self.graphInputSize = rospy.get_param("~deeplab_model/inference_graph/input_size")
        self.graphInputSize = rospy.get_param("~deeplab_model/inference_graph/input_size")


        abs_path = rospkg.RosPack().get_path("object_detection") + "/cfg/" # Gets the path to folder where the models are
        
    
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

    def imageDepthCallback(self, data): #Function that runs when a Depth image arrives
        try:
            self.data_encoding_depth = data.encoding
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def imageDepthInfoCallback(self, cameraInfo): #Code copied from Intel script "show_center_depth.py". Gather camera intrisics parameters that will be use to compute the real coordinates of pixels
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
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
            
            bbox.topleft = self.computeRealCoor(box[0], box[1])

            bbox.topleft.z = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [(bbox.ymin + bbox.ymax)//2,(bbox.xmin + bbox.xmax)//2], self.cv_image_depth[(bbox.ymin + bbox.ymax)//2,(bbox.xmin + bbox.xmax)//2])[2]*(10**-3)
            
            bbox.bottomright = self.computeRealCoor(box[2], box[3])
            bbox.bottomright.z = bbox.topleft.z 

            bbox.score = float(score)
            bbox.id = int(classid)
            bbox.Class = self.classes[classid[0]]

            list_tmp.append(bbox) #append the bounding box to a list with all previous Bounding Box
        
        bboxes = BoundingBoxes()
        bboxes.header = header
        bboxes.bounding_boxes = list_tmp

        if len(list_tmp) != 0:
            self.pub_bbox.publish(bboxes) # Publish the list of Bounding Boxes
        
        return img

    def computeRealCoor(self, x, y):
        
        coor = Vector3()
        
        if(x == 640):
            x = 639
        if(y == 480):
            y = 479

        pix = [x, y] #Coordinates of the central point (in pixeis)
        depth = self.cv_image_depth[pix[1], pix[0]] #Depth of the central pixel
        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth) # Real coordenates, in mm, of the central pixel

        #Create a vector with the coordinates, in meters
        coor.x = result[0]*10**(-3)
        coor.y = result[1]*10**(-3)
        coor.z = result[2]*10**(-3)

        return coor


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