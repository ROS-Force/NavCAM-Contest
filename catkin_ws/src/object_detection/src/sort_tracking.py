#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import imp
import rospkg
import pyrealsense2 as rs2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from object_detection.msg import BoundingBox, BoundingBoxes, TrackerBox, TrackerBoxes, CenterID, ObjectSpeed
from geometry_msgs.msg import Vector3

class Sort_tracking():

    def __init__(self):


        #Initialize variables
        self.method = rospy.get_param("~method", "yolo")
        self.IoU_THRESHOLD = rospy.get_param("~sort_threshold", 0.1) #Minimum IOU for match
        self.MIN_HITS = rospy.get_param("~min_hits", 3) #Minimum number of associated detections before track is initialised
        self.MAX_AGE = rospy.get_param("~max_age", 20) #Maximum number of frames to keep alive a track without associated detections.


        self.list_bbox = BoundingBoxes()
        self.previous_time = rospy.Time.now()
        self.previous_centers = {}

        self.cv_image_depth = None
        self.cv_image = None
        self.intrinsics = None

        self.bridge = CvBridge()
        
        abs_path = rospkg.RosPack().get_path("object_detection") #get path to package
        si = imp.load_source('sort', abs_path + "/include/sort/" + 'sort.py') #load the sort.py file
        self.mo_tracker = si.Sort(max_age=self.MAX_AGE, min_hits=self.MIN_HITS, iou_threshold=self.IoU_THRESHOLD) #Create the multi-object tracker

        self.classes = []
        with open(abs_path + "/cfg/" + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        #Publishers
        self.pub = rospy.Publisher("/tracking/" + self.method + "/objects_image", Image, queue_size=10)
        self.pub_speed = rospy.Publisher("/tracking/" + self.method + "/speed", ObjectSpeed, queue_size=10)

        #Subscribers
        self.sub_bbox_yolo = rospy.Subscriber("/detection/"+self.method+"/bboxes", BoundingBoxes,self.bboxCallback, queue_size=10)
        self.sub_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)


#Callbacks
    def imageCallback(self, data): #Function that runs when a RGB image arrives
        try:
            self.data_encoding = data.encoding #Stores the encoding of original image
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)  # Transforms the format of image into OpenCV 2

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


    def bboxCallback(self,data): #Function that runs when a Bounding Box list arrives (main code is here)

        self.list_bbox = data

        if self.cv_image is not None and self.cv_image_depth is not None: # if RGB and Depth images are available runs the code

                trackers = self.mo_tracker.update(self.list_bbox) #Update the Tracker Boxes positions

                center_list, new_time = self.computeRealCenter(trackers) #compute the Real pixels values
                
                self.computeSpeed(center_list, new_time) #compute the velocity vector of the diferent objects

                img = self.draw_labels(trackers) #draws the labels in the original image
                image_message = self.bridge.cv2_to_imgmsg(img, encoding = self.data_encoding)
                self.pub.publish(image_message) #publish the labelled image



    def draw_labels(self, tracker_boxes): 
        font = cv2.FONT_HERSHEY_PLAIN
        img = self.cv_image

        for t in tracker_boxes.tracker_boxes:
            label = "%s #%i" % (t.Class, t.id)
            color = self.colors[int(t.id) % len(self.colors)]
            cv2.rectangle(img, (int(t.xmin), int(t.ymin), int(t.xmax) - int(t.xmin), int(t.ymax) - int(t.ymin)), color, 2) #draws a rectangle in the original image
            cv2.putText(img, label, (int(t.xmin), int(t.ymin - 10)), font, 1, color, 1) #writes the Class and Object ID in the original image

        return img


    def computeRealCenter(self, trackers_msg):
        
        center = CenterID()
        center_list = {}
        trackers = trackers_msg.tracker_boxes

        new_time = trackers_msg.header.stamp #Time of the current trackers (Time was created when the image arrived in the code "yolo_detection.py" inside the imageCallback() function)

        for t in trackers:

            pix = [int(t.xmin + (t.xmax-t.xmin)//2), int(t.ymin + (t.ymax-t.ymin)//2)] #Coordinates of centrar pixel
            depth = self.cv_image_depth[pix[1], pix[0]] #Depth of the central pixel
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth) # Real coordenates, in mm, of the central pixel
            
            #Create the CenterID object
            center.x = result[0]
            center.y = result[1]
            center.z = result[2]
            center.id = t.id
            center.Class = t.Class
            center_list[t.id] = center
        
        return center_list, new_time


    def computeSpeed(self, centers, new_time):

        speed = Vector3()
        msg = ObjectSpeed()

        if len(self.previous_centers) == 0:
            self.previous_centers = centers
            self.previous_time = new_time
            return None
        
        else:

            deltat = (new_time - self.previous_time) #Compute the diference in times between the previous time and this frame
            self.previous_time = new_time
            deltat = deltat.to_sec() #Transform the time in sec


            for id in self.previous_centers: #go through all objects detected in the previous frame (The ID is unique to each object)
                
                previous_center = self.previous_centers.get(id) #get the center of previous object with that id
                updated_center = centers.get(id) #get the center of current object with that id, if the id isn't fouund return a None that is treated in the next IF

                if updated_center is not None and previous_center is not None: #If both of ID are found it computes the speed 
                    
                    speed.x = (updated_center.x - previous_center.x)*10**(-3)/deltat
                    speed.y = (updated_center.y - previous_center.y)*10**(-3)/deltat
                    speed.z = (updated_center.z - previous_center.z)*10**(-3)/deltat

                    msg.velocity = speed
                    msg.id = previous_center.id
                    msg.Class = previous_center.Class
                    
                    self.pub_speed.publish(msg) #publish the speed of current object

            self.previous_centers = centers #set the current object as previous objects




def main():

    rospy.init_node('sort_tracking')
    st = Sort_tracking()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")