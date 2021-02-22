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
        self.pub_obj = rospy.Publisher("/tracking/" + self.method + "/object", Object, queue_size=10)

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

                img = np.copy(self.cv_image)
                trackers = self.mo_tracker.update(self.list_bbox) #Update the Tracker Boxes positions

                for t in trackers.tracker_boxes: #Go through every detected object
                    
                    obj = Object()
                    center_pose = self.computeRealCenter(t) #compute the Real pixels values
                    
                    speed = self.computeSpeed(center_pose, t.id, trackers.header.stamp) #compute the velocity vector of the diferent objects

                    [color, sat, il], shape = self.computeFeatures(t) #compute diferent features, like color and shape

                    center_list[t.id] = center_pose #add this center to the dictionary of centers

                    #Construct the object with its atributes
                    obj.id = t.id
                    obj.Class = t.Class
                    obj.real_pose = center_pose
                    obj.velocity = speed
                    obj.bbox = [int(t.xmin), int(t.ymin), int(t.xmax), int(t.ymax)]
                    obj.color = color
                    obj.saturation = sat
                    obj.ilumination = il 
                    obj.shape = shape

                    self.pub_obj.publish(obj) # publish the object


                self.previous_centers = center_list #set the current center dictionary as previous dictionary
                self.previous_time = trackers.header.stamp

                img_labels = self.draw_labels(trackers, img) #draws the labels in the original image
                
                
                image_message = self.bridge.cv2_to_imgmsg(img_labels, encoding = self.data_encoding)
                self.pub.publish(image_message) #publish the labelled image


    def computeRealCenter(self, tracker):
        
        center = Vector3()

        pix = [int(tracker.xmin + (tracker.xmax-tracker.xmin)//2), int(tracker.ymin + (tracker.ymax-tracker.ymin)//2)] #Coordinates of the central point (in pixeis)
        depth = self.cv_image_depth[pix[1], pix[0]] #Depth of the central pixel
        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth) # Real coordenates, in mm, of the central pixel

        #Create a vector with the coordinates, in meters
        center.x = result[0]*10^(-3)
        center.y = result[1]*10^(-3)
        center.z = result[2]*10^(-3)

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
                    
                speed.x = (center.x - previous_center.x)/deltat
                speed.y = (center.y - previous_center.y)/deltat
                speed.z = (center.z - previous_center.z)/deltat

            return speed #in meters/sec

    def computeFeatures(self, t):

        img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # change image from bgr to hsv

        thr_img, roi = self.computeRoi(img, self.cv_image_depth, t) #select the roi of the object

        [counts, values] = np.histogram(thr_img[0], bins=18, range=(1,180)) #create an histogram to see the most present color
        m = max(counts)
        dic = dict(zip(counts, values))
        hue = int(dic[m] * 2) # hue valor of the most present color
        sat = mean(roi[1])/255.0 # sat mean of image (E PARA MUDAR ISTO)
        ilumination = mean(roi[2])/255.0

        shape = self.computeShape(thr_img);

        c = Color(hsv=(hue, sat, ilumination))
        c1 = Color((int(c.red), int(c.green), int(c.blue)))
        
        return [c1.web, sat, ilumination], shape 

    def computeShape(self, img):

        return "Feature in development"


    def computeRoi(self, img, depth, t):

        roi = img[int(t.ymin):int(t.ymax), int(t.xmin):int(t.xmax)] #select the region of interess of rgb and depth images
        roi_depth = depth[int(t.ymin):int(t.ymax), int(t.xmin):int(t.xmax)]
        
        m = mean(roi_depth)
        std = np.std(roi_depth)
        ret, thresh = cv2.threshold(roi_depth, m+std, np.amax(roi_depth),cv2.THRESH_BINARY_INV) #threshold to try to minimize the backgound influence
        mask = self.map_uint16_to_uint8(thresh, lower_bound=0, upper_bound=255)
        thr_img = cv2.bitwise_and(roi, roi,mask = mask) #apply the mask created to the rgb image
        
        return thr_img, roi


    def map_uint16_to_uint8(self, img, lower_bound=None, upper_bound=None):
        if not(0 <= lower_bound < 2**16) and lower_bound is not None:
            raise ValueError(
                '"lower_bound" must be in the range [0, 65535]')
        if not(0 <= upper_bound < 2**16) and upper_bound is not None:
            raise ValueError(
                '"upper_bound" must be in the range [0, 65535]')
        if lower_bound is None:
            lower_bound = np.min(img)
        if upper_bound is None:
            upper_bound = np.max(img)
        if lower_bound >= upper_bound:
            raise ValueError(
                '"lower_bound" must be smaller than "upper_bound"')
        lut = np.concatenate([
            np.zeros(lower_bound, dtype=np.uint16),
            np.linspace(0, 255, upper_bound - lower_bound).astype(np.uint16),
            np.ones(2**16 - upper_bound, dtype=np.uint16) * 255
        ])
        return lut[img].astype(np.uint8)

    def draw_labels(self, tracker_boxes, img): 
        font = cv2.FONT_HERSHEY_PLAIN

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