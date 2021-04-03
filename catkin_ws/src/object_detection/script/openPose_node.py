#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import sys
import os

import ctypes


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs2
from openpose import pyopenpose as op
import rospkg

class openPose():

    def __init__(self):

        #Initialize variables
        self.bridge = CvBridge()

        self.cv_image_depth = None
        self.cv_image = None
        self.intrinsics = None

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

        abs_path = rospkg.RosPack().get_path("object_detection") + "/cfg/"

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.setOpenPoseParam(abs_path))
        self.opWrapper.start()

        #Subscriber
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        self.sub_info = rospy.Subscriber("camera_info", CameraInfo, self.imageDepthInfoCallback)
        self.sub_depth = rospy.Subscriber("depth_image", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)

        #Publisher
        self.pub = rospy.Publisher("output_image", Image, queue_size=1)
    
    def imageCallback(self, data): #Function that runs when an image arrives
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding) # Transforms the format of image into OpenCV 2
            

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

        datum = op.Datum()
        datum.cvInputData = cv_image

        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        print("Body keypoints: \n" + str(datum.poseKeypoints))


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


    def setOpenPoseParam(self, path):

        params = dict()
        params["logging_level"] = 3
        params["output_resolution"] = "-1x-1"
        params["net_resolution"] = "-1x192"
        params["model_pose"] = "COCO"
        params["alpha_pose"] = 0.6
        params["scale_gap"] = 0.3
        params["scale_number"] = 1
        params["render_threshold"] = 0.05
        # If GPU version is built, and multiple GPUs are available, set the ID here
        params["num_gpu_start"] = 0
        params["disable_blending"] = False
        # Ensure you point to the correct path where models are located
        params["model_folder"] = path + "OpenPose/models/"

        return params



def main():

    rospy.init_node('yolo_detection')
    op = openPose()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")