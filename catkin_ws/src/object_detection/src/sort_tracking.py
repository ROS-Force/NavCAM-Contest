#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import imp
import rospkg
import pyrealsense2 as rs2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from object_detection.msg import BoundingBox, BoundingBoxes, TrackerBox, TrackerBoxes, CenterID
from object_detection.msg import ObjectSpeed
from geometry_msgs.msg import Vector3
class Sort_tracking():

    def __init__(self):

        IoU_THRESHOLD = 0.1 #Minimum IOU for match
        MIN_HITS = 3 #Minimum number of associated detections before track is initialised
        MAX_AGE = 20 #Maximum number of frames to keep alive a track without associated detections.

        self.list_bbox = BoundingBoxes()

        self.previous_time = rospy.Time.now()
        self.old_time = rospy.Time.now()

        self.previous_centers = {}

        self.cv_image_depth = None
        self.cv_image = None
        self.intrinsics = None

        self.bridge = CvBridge()
        
        abs_path = rospkg.RosPack().get_path("object_detection")

        si = imp.load_source('sort', abs_path + "/include/sort/" + 'sort.py')
        self.mo_tracker = si.Sort(max_age=MAX_AGE, min_hits=MIN_HITS, iou_threshold=IoU_THRESHOLD) #cria o multiple object tracker com base no codigo do Andrew cenas
        self.mo_tracker_watershed = si.Sort(max_age=20, min_hits=1, iou_threshold=IoU_THRESHOLD)

        self.classes = []
        with open(abs_path + "/cfg/" + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        #Publisher
        self.pub = rospy.Publisher("/tracking/yolo/objects_image", Image, queue_size=10)
        self.pub_speed = rospy.Publisher("/tracking/speed", ObjectSpeed, queue_size=10)

        #Subscribers
        self.sub_bbox_yolo = rospy.Subscriber("/detection/yolo/bboxes", BoundingBoxes,self.bboxCallback, queue_size=10)
        self.sub_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)
        

#Callbacks
    def imageCallback(self, data):
        try:
            self.data_encoding = data.encoding
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.existImage= True

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    def imageDepthCallback(self, data):
        try:
            self.data_encoding_depth = data.encoding
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
    
    def bboxCallback(self,data):

        self.list_bbox = data

        if self.cv_image is not None and self.cv_image_depth is not None:

                trackers = self.mo_tracker.update(self.list_bbox)

                center_list, new_time = self.computeRealCenter(trackers)
                
                speed = self.computeSpeed(center_list, new_time)

                img = self.draw_labels(trackers)
                image_message = self.bridge.cv2_to_imgmsg(img, encoding = self.data_encoding)
                self.pub.publish(image_message)

    def imageDepthInfoCallback(self, cameraInfo):
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



    def draw_labels(self, tracker_boxes): 
        font = cv2.FONT_HERSHEY_PLAIN
        img = self.cv_image

        # iteracao em paralelo
        for t in tracker_boxes.tracker_boxes:
            label = "%s #%i" % (t.Class, t.id)
            color = self.colors[int(t.id) % len(self.colors)]


            cv2.rectangle(img, (int(t.xmin), int(t.ymin), int(t.xmax) - int(t.xmin), int(t.ymax) - int(t.ymin)), color, 2)
            cv2.putText(img, label, (int(t.xmin), int(t.ymin - 10)), font, 1, color, 1)
        return img


    def computeRealCenter(self, trackers_msg):
        
        center = CenterID()
        center_list = {}
        trackers = trackers_msg.tracker_boxes

        new_time = trackers_msg.header.stamp #Tempo dos centros atuais


        for t in trackers:

            pix = [int(t.xmin + (t.xmax-t.xmin)//2), int(t.ymin + (t.ymax-t.ymin)//2)] # coordenadas do pixel central
            depth = self.cv_image_depth[pix[1], pix[0]]
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
            
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

            #new_time = rospy.Time.now()
            deltat = (new_time - self.previous_time)
            self.previous_time = new_time
            deltat = deltat.to_sec()


            for id in self.previous_centers:
                
                previous_center = self.previous_centers.get(id)
                updated_center = centers.get(id)

                if updated_center is not None and previous_center is not None:
                    
                    speed.x = (updated_center.x - previous_center.x)*10**(-3)/deltat
                    speed.y = (updated_center.y - previous_center.y)*10**(-3)/deltat
                    speed.z = (updated_center.z - previous_center.z)*10**(-3)/deltat

                    msg.velocity = speed
                    msg.id = previous_center.id
                    msg.Class = previous_center.Class
                    
                    self.pub_speed.publish(msg)

            self.previous_centers = centers
        



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