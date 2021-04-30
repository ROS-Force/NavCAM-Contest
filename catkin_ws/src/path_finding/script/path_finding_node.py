#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2
import sys
import tf as TF
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from datetime import datetime

import tensorflow as tf

#import numba
#from numba import jit

#@jit(nopython=True, parallel=True)
def find_first(data):
    for i in range(len(data)):
        if data[i] != -1:
            return i
    return -1

#@jit(debug=True, nopython=True)
def cropMap2(data, map_width, map_height):
    
    print(datetime.now(), " Antes de ter o indice")
    inx = find_first(data)
    print(datetime.now(), " Depois de ter o indice")
    cut = inx//map_width * map_width

    inx = find_first(data)
    print(datetime.now(), " Depois de ter o indice")
    cut = inx//map_width * map_width

    cutted_map = data[cut:]

    map_array = np.reshape(cutted_map, [len(cutted_map)//map_width, map_width])

    print(datetime.now(), " Antes do crop: ", map_array.shape)
    xs, ys = np.where(map_array>=0)
    result = map_array[min(xs):max(xs)+1,min(ys):max(ys)+1] 
    print(datetime.now(), " Depois do crop: ", map_array.shape)
    return result

#@tf.function
def cropMapTF(data, map_width, map_height):
    print(datetime.now(), "Before tensor conv")
    map_tf = tf.convert_to_tensor(data)
    print(datetime.now(), "After tensor conv")
    map_tf = tf.reshape(map_tf, [map_width, map_height])
    print(datetime.now(), "Before searching and reducing columns")
    
    # sum columns
    indexes = tf.where(map_tf >= 0)
    min_idx = tf.reduce_min(indexes, axis=0)
    max_idx = tf.reduce_max(indexes, axis=0)

    print(min_idx, max_idx)
    print(datetime.now(), "After summing columns")
    result = map_tf[min_idx[0]:max_idx[0],min_idx[1]:max_idx[1]] 
    print(datetime.now(), "After slicing columns")

    result = tf.where(result == 0, x=1, y=result)
    result =tf.where(result == 100, x=-1, y=result)
   
    return result.numpy()

class Path_finding():

    def __init__(self):

        #Initialize variables

        self.intrinsics = None
        self.bridge = CvBridge()
        self.path = None
        self.stopPathFinding = False        

        #Publishers
        #self.pub = rospy.Publisher("output_image", Image, queue_size=1)
        #self.pub_obj = rospy.Publisher("object_info", Object_info, queue_size=1)

        self.tflistener = TF.TransformListener() 

        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

        #Subscriber
        #self.sub = rospy.Subscriber("map", OccupancyGrid, self.mapCallback, queue_size=10, buff_size=2**24)

        #self.sub = rospy.Subscriber("goal", OccupancyGrid, self.mapCallback, queue_size=10, buff_size=2**24)

        #Publisher (update this )
        self.pub = rospy.Publisher("path_image", Image, queue_size=1)



    def getCurrentMap(self):
        rospy.wait_for_service('dynamic_map')
        try:
            dynamicMap = rospy.ServiceProxy('dynamic_map', GetMap)
            dynamicMapResult = dynamicMap()

            return dynamicMapResult.map
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
#Callbacks
    def pathUpdateCallback(self): 
        print(datetime.now().timestamp(), "before map")
        
        # skip updates without map
        latestGrid = self.getCurrentMap()
        if (latestGrid is None):
            return
        print(datetime.now().timestamp(), "after map")
        
        header = latestGrid.header
        origin = latestGrid.info.origin
        resolution = latestGrid.info.resolution
        map_width = latestGrid.info.width
        map_height = latestGrid.info.height

        print(datetime.now().timestamp(), "before crop")
        
        map_array = cropMapTF(latestGrid.data, map_width, map_height)
        print(datetime.now().timestamp(), "after crop")
        

        grid = Grid(matrix=map_array)
        
        #try:
        #    (trans,rot) = self.tflistener.lookupTransform('/camera_link', '/map', rospy.Time(0))
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    print("Error on transform")
        #    pass

        # set
        start = np.array([map_array.shape[0]//2, map_array.shape[1]//2])
        end = start + 5

        start_node = grid.node(start[0],start[1])
        end_node = grid.node(end[0], end[1])

        print(datetime.now(), "before A star")
        
        path, runs = self.finder.find_path(start_node, end_node, grid)

        
        print(datetime.now(), "before map conv")
        #print('operations:', runs, 'path length:', len(path))

        image = self.map2Image(map_array, start, end, path)
        self.path = path

        print(datetime.now().timestamp(), "after map conv")
        
        image_message = self.bridge.cv2_to_imgmsg(image) 
        self.pub.publish(image_message) #publish the image
        
    def cropMap_shit(self, map):

        xs, ys = np.where(map>=0)
        result = map[min(xs):max(xs)+1,min(ys):max(ys)+1] 

        return result


    def map2Image(self, map, start, end, path):
        img = np.zeros((map.shape[1], map.shape[0], 3), np.int8)
        path = np.asarray(path)

        # paint the freespace of white
        free_space_idx = np.where(map==1)

        for x, y in zip(free_space_idx[0], free_space_idx[1]): 
            img[y,x,:] = 255

        #paint path
        for point in path:
            img[point[1],point[0],0] = 0
            img[point[1],point[0],1] = 255
            img[point[1],point[0],2] = 0
        
        #Paint start
        img[start[1],start[0],0] = 255
        img[start[1],start[0],1] = 0
        img[start[1],start[0],2] = 0

        #Paint end
        img[end[1],end[0],0] = 0
        img[end[1],end[0],1] = 0
        img[end[1],end[0],2] = 255
        
        return img

    def run(self):  
        
        # main thread will now be used to update periodically Astar path

        sleepRate = rospy.Rate(500)

        while(not rospy.is_shutdown()):
            self.pathUpdateCallback()
            sleepRate.sleep()

def main():

    rospy.init_node('path_finding')

    pf = Path_finding()
    pf.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")