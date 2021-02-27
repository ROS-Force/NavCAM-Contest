#!/usr/bin/python


import rospy
import numpy as np
import cv2

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from object_detection.msg import BoundingBox, BoundingBoxes
import random

class Watershed_Detection():

    def __init__(self):

        self.bridge = CvBridge()
        self.countFrame = 1

        self.threshold = rospy.get_param("threshold", 7000)

        self.cv_image = None
        #self.data_enconding = None

        #Publisher
        self.pub_blob = rospy.Publisher("/detection/watershed/blob", Image, queue_size=10)
        self.pub_image = rospy.Publisher("/detection/watershed/objects_image", Image, queue_size=10) 
        self.pub_bbox = rospy.Publisher("/detection/watershed/bboxes", BoundingBoxes, queue_size=10)
        
        #Subscriber
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepthCallback, queue_size=1, buff_size=2**24)


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

        self.cv_image_depth[self.cv_image_depth>10000] = 0
        
        norm = cv2.normalize(self.cv_image_depth, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        

        kernel = np.ones((9,9),np.uint8)
        tmp = cv2.morphologyEx(norm,cv2.MORPH_CLOSE,kernel, iterations = 9)
        self.thr_img = cv2.morphologyEx(tmp,cv2.MORPH_OPEN,kernel, iterations = 3)
        
        canny = cv2.Canny(tmp,100,200)

        img = self.bridge.cv2_to_imgmsg(canny)
        self.pub_blob.publish(img)
        #ret, thresh = cv2.threshold(self.cv_image_depth, 0, 7000, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        
        
        
        #mask = self.map_uint16_to_uint8(thresh, lower_bound=0, upper_bound=255)

        #if self.cv_image is not None:

        #    self.thr_img = cv2.bitwise_and(self.cv_image,self.cv_image,mask = mask)



    def imageCallback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        #self.data_enconding = data.enconding
        
        h = Header()
        #Create a Time stamp
        h.stamp = rospy.Time.now()
        h.frame_id = "Yolo Frame"
        img = self.cv_image

        #ws = self.watershed(img,self.thr_img)

        #ws_8bit = self.map_uint16_to_uint8(ws)

        contours, hierarchy = cv2.findContours(self.thr_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        list_bbox = []
        bboxes = BoundingBoxes()


        for c in contours:
            xmin, ymin, w, h = cv2.boundingRect(c)
            
            xmax = xmin + w
            ymax = ymin + h
            id1 = int(random.randint(1, 255))

            #clean rectangles with area < 100px
            if (w * h > 100 and w * h < 250000): 

                #Create the Bounding Box object
                bbox = BoundingBox()
                bbox.xmin = xmin
                bbox.ymin = ymin
                bbox.xmax = xmax
                bbox.ymax = ymax
                bbox.score = float(1.0)
                bbox.id = id1
                bbox.Class = "Unidentified Object"
                # draw a green rectangle to visualize the bounding rect
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                list_bbox.append(bbox)

        bboxes.header = h
        bboxes.bounding_boxes = list_bbox

        self.pub_bbox.publish(bboxes) # Publish the list of Bounding Boxes
        
        #Publica a imagem com os quadrados verdes desenhados
        image_message = self.bridge.cv2_to_imgmsg(img, encoding=data.encoding)
        self.pub_image.publish(image_message)





    def watershed(self, image, gray):

        #gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 9)

        
        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.4*dist_transform.max(),255,0)

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)

        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1

        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0

        markers = cv2.watershed(image,markers)
        
        #colocar as fronteiras dos blobs a 0, para nao dar stress com o background
        markers[markers == -1] = 0

        #Remover o background como sendo um blob
        markers[markers == 1] = 0

        img = markers.astype(np.uint16)

        #Publica a imagem com os quadrados verdes desenhados
        #image_message = self.bridge.cv2_to_imgmsg(opening)
        #self.pub_image.publish(image_message)

        #unique, counts = np.unique(img, return_counts=True)
        #print(dict(zip(unique, counts)))
        return img


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()

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



def main():

    rospy.init_node('watershed_detection', anonymous=True)

    mod = Watershed_Detection()

    mod.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")