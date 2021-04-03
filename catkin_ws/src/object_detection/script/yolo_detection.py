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
from geometry_msgs.msg import Vector3
import pyrealsense2 as rs2

class YoloModelConfig(object):
    '''
    Helper class which holds the parameters for the YOLO model used. 
    '''

    def __init__(self, modelConfig):
        self.name = modelConfig.get("name") # yolo.weights path.
        self.weightPath = modelConfig.get("weights_path") # yolo.weights path.
        self.configPath = modelConfig.get("config_path") # yolo.cfg path.
        self.inputSize = tuple(modelConfig.get("input_size"))  # New input size.
        self.mean = tuple(modelConfig.get("model_mean")) # Scalar with mean values which are subtracted from channels.
        self.scale = modelConfig.get("model_scale") #Multiplier for frame values. 
        self.swapRB = modelConfig.get("model_swapRGB") #Flag which indicates that swap first and last channels.
        self.crop = modelConfig.get("model_crop") #Flag which indicates whether image will be cropped after resize or not.
        self.confidenceThreshold = modelConfig.get("conf_threshold") #A threshold used to filter boxes by confidences.
        self.NMSThreshold = modelConfig.get("nms_threshold") # A threshold used in non maximum suppression. 
        self.classes = modelConfig.get("dataset", {}).get("detection_classes") # List of classes detected by the model

class YoloNode():

    def __init__(self):
        # create CV bridge to handle conversion between ROS and OpenCV
        self.bridge = CvBridge()
        
        #Check if this computer have CUDA libraries, loads a light model in case of missing CUDA library. 
        # This light model decreases accuracy but improves frame rate  
        cuda = True
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

        # Load models depending if cuda was found or not
        defaultModelPrefix, lightModelPrefix = "/yolo_model", "/yolo_light_model"

        if cuda:
            rospy.loginfo("CUDA was found, loading best model..")
            
            if (rospy.has_param(defaultModelPrefix)):
                self.modelConfig = YoloModelConfig(rospy.get_param(defaultModelPrefix))
                rospy.loginfo("Found default model, loading '{}'".format(self.modelConfig.name))
                rospy.set_param(rospy.get_name() + "/model_prefix", defaultModelPrefix)
            elif (rospy.has_param(lightModelPrefix)):
                self.modelConfig = YoloModelConfig(rospy.get_param(lightModelPrefix))
                rospy.loginfo("Couldn't find default model, but found light model, loading '{}'".format(self.modelConfig.name))
                rospy.set_param(rospy.get_name() + "/model_prefix", lightModelPrefix)
            else:
                rospy.logerr("Couldn't find any valid model, exiting...")
                sys.exit(1)
        else:
            rospy.loginfo("CUDA was NOT found, loading lightest model..")

            if (rospy.has_param(lightModelPrefix)):
                self.modelConfig = YoloModelConfig(rospy.get_param(lightModelPrefix))
                rospy.loginfo("Found light model, loading '{}'".format(self.modelConfig.name))
                rospy.set_param("model_prefix", lightModelPrefix)
            elif (rospy.has_param(defaultModelPrefix)):
                self.modelConfig = YoloModelConfig(rospy.get_param(defaultModelPrefix))
                rospy.loginfo("Couldn't find light model, but found default model, loading '{}'".format(self.modelConfig.name))
                rospy.set_param("model_prefix", defaultModelPrefix)
            else:
                rospy.logerr("Couldn't find any valid model, exiting...")
                sys.exit(1)
        try:
            # load DNN
            self.net = cv2.dnn.readNetFromDarknet(self.modelConfig.configPath, self.modelConfig.weightPath)

            # change DNN backend if cuda is available
            if cuda:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA) 

            # create detection model from DNN and setup input parameters
            self.model = cv2.dnn_DetectionModel(self.net)
            self.model.setInputParams(scale=self.modelConfig.scale, size=self.modelConfig.inputSize, mean=self.modelConfig.mean, swapRB=self.modelConfig.swapRB, crop=self.modelConfig.crop)
        except:
            rospy.logerr("Failed to load the DNN model!")
            sys.exit(1)

        rospy.loginfo("Successfully loaded YOLO model!")

        # Loading remaining node configurations
        self.output_rgb = rospy.get_param("~show_output") # Show RGB image.

        self.cv_image_depth = None
        self.cv_image = None
        self.intrinsics = None
        self.colors = np.random.uniform(0, 255, size=(len(self.modelConfig.classes), 3))

        #Subscriber (buff_size is set to 2**24 to avoid delays in the callbacks)
        self.sub = rospy.Subscriber("image", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber("depth_image", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)

        # grab camera info
        self.imageDepthInfoCallback(rospy.wait_for_message("camera_info", CameraInfo))  

        #Publisher
        self.pub = rospy.Publisher("output_image", Image, queue_size=1)
        self.pub_bbox = rospy.Publisher("bounding_boxes", BoundingBoxes, queue_size=10)
    
    def imageCallback(self, data): #Function that runs when an image arrives
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2
            
            # exit callback if no depth image stored

            if (self.cv_image_depth is None):
                return 

            if(cv_image.shape != self.cv_image_depth.shape):
                cv_image = cv2.resize(cv_image, self.cv_image_depth.shape[1::-1])
            
            h = Header()
            #Create a Time stamp
            h.stamp = rospy.Time.now()
            h.frame_id = "Yolo Frame"
            

            classes, scores, boxes = self.model.detect(cv_image, self.modelConfig.confidenceThreshold, self.modelConfig.NMSThreshold) #Runs the YOLO model
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


    def imageDepthInfoCallback(self, cameraInfo): 
        '''
        Code copied from Intel script "show_center_depth.py". 
        Gather camera intrisics parameters that will be use to compute the real coordinates of pixels.
        
        '''
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
            label = "%s : %f" % (self.modelConfig.classes[classid[0]], score)
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
            
            if self.cv_image_depth is not None:

                bbox.topleft = self.computeRealCoor(box[0], box[1])
    
                bbox.topleft.z = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [(bbox.ymin + bbox.ymax)//2,(bbox.xmin + bbox.xmax)//2], self.cv_image_depth[(bbox.ymin + bbox.ymax)//2,(bbox.xmin + bbox.xmax)//2])[2]*(10**-3)
                
                bbox.bottomright = self.computeRealCoor(box[2], box[3])
                bbox.bottomright.z = bbox.topleft.z 

            bbox.score = float(score)
            bbox.id = int(classid)
            bbox.Class = self.modelConfig.classes[classid[0]]

            list_tmp.append(bbox) #append the bounding box to a list with all previous Bounding Box
        
        bboxes = BoundingBoxes()
        bboxes.header = header
        bboxes.bounding_boxes = list_tmp

        if len(list_tmp) != 0:
            self.pub_bbox.publish(bboxes) # Publish the list of Bounding Boxes
        
        return img

    def computeRealCoor(self, x, y):
        
        coor = Vector3()
        
        if(x ==  self.cv_image_depth.shape[1]):
            x = x-1
        if(y ==  self.cv_image_depth.shape[0]):
            y = y-1


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
    hd = YoloNode()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")