#!/usr/bin/env python

from numpy.core.fromnumeric import mean
import rospy
import numpy as np
import cv2
import imp
from colorutils import Color
import rospkg
import pyrealsense2 as rs2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from object_detection.msg import BoundingBox, BoundingBoxes, TrackerBox, TrackerBoxes, CenterID, Object
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
        self.color_labels = []

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
        self.pub_speed = rospy.Publisher("/tracking/" + self.method + "/speed", Object, queue_size=10)

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
        center_list = {}

        if self.cv_image is not None and self.cv_image_depth is not None: # if RGB and Depth images are available runs the code

                trackers = self.mo_tracker.update(self.list_bbox) #Update the Tracker Boxes positions

                for t in trackers.tracker_boxes:

                    center_pose = self.computeRealCenter(t) #compute the Real pixels values
                
                    speed = self.computeSpeed(center_pose, t.id, trackers.header.stamp) #compute the velocity vector of the diferent objects

                    center_list[t.id] = center_pose #add this center to the dictionary of centers
                    self.computeFeatures(t)

                self.previous_centers = center_list #set the current center dictionary as previous dictionary
                self.previous_time = trackers.header.stamp
                

                img = self.draw_labels(trackers) #draws the labels in the original image
                image_message = self.bridge.cv2_to_imgmsg(img, encoding = self.data_encoding)
                self.pub.publish(image_message) #publish the labelled image




    def computeRealCenter(self, tracker):
        
        center = Vector3()

        pix = [int(tracker.xmin + (tracker.xmax-tracker.xmin)//2), int(tracker.ymin + (tracker.ymax-tracker.ymin)//2)] #Coordinates of central pixel
        depth = self.cv_image_depth[pix[1], pix[0]] #Depth of the central pixel
        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth) # Real coordenates, in mm, of the central pixel
            
        #Create a vectot with the coordinates
        center.x = result[0]
        center.y = result[1]
        center.z = result[2]

        return center


    def computeSpeed(self, center, id, new_time):

        speed = Vector3()

        if len(self.previous_centers) == 0:
            
            return None
        
        else:

            deltat = (new_time - self.previous_time) #Compute the diference in times between the previous time and this frame, 
                                                    # The time of the current tracker was created when the image arrived (in 
            deltat = deltat.to_sec()                 # the code "yolo_detection.py" inside the imageCallback() function).

            
            previous_center = self.previous_centers.get(id) #get the center of previous object with that id, if the id isn't fouund return a None that is treated in the next IF

            if previous_center is not None: #If exists a center in the last frame computes the speed
                    
                speed.x = (center.x - previous_center.x)*10**(-3)/deltat
                speed.y = (center.y - previous_center.y)*10**(-3)/deltat
                speed.z = (center.z - previous_center.z)*10**(-3)/deltat

            return speed

    def computeFeatures(self, t):


        img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        roi = img[int(t.ymin):int(t.ymax), int(t.xmin):int(t.xmax)]

        [counts, values] = np.histogram(roi[0], bins=18, range=(0,180))
        m = max(counts)
        dic = dict(zip(counts, values))
        hue = int(dic[m])
        sat = int(mean(roi[1]))
        
        ilumination = int(mean(roi[2]))
        c = Color(hsv=(hue, sat/255.0, ilumination/255.0))
        c1 = Color((int(c.red), int(c.green), int(c.blue)))

        print(c1.web)




    def draw_labels(self, tracker_boxes): 
        font = cv2.FONT_HERSHEY_PLAIN
        img = self.cv_image

        for t in tracker_boxes.tracker_boxes:
            label = "%s #%i" % (t.Class, t.id)
            color = self.colors[int(t.id) % len(self.colors)]
            cv2.rectangle(img, (int(t.xmin), int(t.ymin), int(t.xmax) - int(t.xmin), int(t.ymax) - int(t.ymin)), color, 2) #draws a rectangle in the original image
            cv2.putText(img, label, (int(t.xmin), int(t.ymin - 10)), font, 1, color, 1) #writes the Class and Object ID in the original image

        return img



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