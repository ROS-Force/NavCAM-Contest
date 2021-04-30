#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2
import sys
import tf
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

        self.tflistener = tf.TransformListener() 

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

#Callbacks
    def pathUpdateCallback(self): 
        print(datetime.now().timestamp(), "before map")

        latestGrid = self.getCurrentMap()

        print(datetime.now().timestamp(), "after map")
        
        header = latestGrid.header

        origin = latestGrid.info.origin
        resolution = latestGrid.info.resolution
        map_width = latestGrid.info.width
        map_height = latestGrid.info.height


        uncroppedMap = np.array(latestGrid.data)

        print(datetime.now().timestamp(), "after conv")
        
        map_array = np.reshape(latestGrid.data, [map_height, map_width])


        print(datetime.now().timestamp(), "before merdas do mario")
        

        row, col = self.test(map_array)

        print(row, col)

        print(datetime.now().timestamp(), "after merdas do mario")
        


        map_array = self.cropMap_shit(map_array)
        map_array[map_array == 0] = 1
        map_array[map_array == 100] = -1


        print(datetime.now().timestamp(), "after np ")
        

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

        print(datetime.now().timestamp(), "before A star")
        
        path, runs = self.finder.find_path(start_node, end_node, grid)

        
        print(datetime.now().timestamp(), "before map conv")
        #print('operations:', runs, 'path length:', len(path))

        image = self.map2Image(map_array, start, end, path)
        self.path = path

        print(datetime.now().timestamp(), "after map conv")
        
        image_message = self.bridge.cv2_to_imgmsg(image) 
        self.pub.publish(image_message) #publish the image

    
    
    def test(self,map):
        tmp = np.ones((1,map.shape(1)))
        emptyLine = val = np.dot(map[0,:], tmp)

        for row in map:
            val = np.dot(row, tmp)

            if not val == emptyLine:
                break

        for col in map.T:
            val = np.dot(col, tmp)

            if not val == emptyLine:
                break
        return row, col
        
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