#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2
import sys

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose

from nav_msgs.msg import OccupancyGrid, MapMetaData

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Path_finding():

    def __init__(self):

        #Initialize variables

        self.intrinsics = None
        self.bridge = CvBridge()

        #Publishers
        #self.pub = rospy.Publisher("output_image", Image, queue_size=1)
        #self.pub_obj = rospy.Publisher("object_info", Object_info, queue_size=1)

        #Subscriber
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.mapCallback, queue_size=10, buff_size=2**24)

        #Publisher
        self.pub = rospy.Publisher("path_image", Image, queue_size=1)



#Callbacks
    def mapCallback(self, data): #Function that runs when a RGB image arrives
        header = data.header

        origin = data.info.origin
        resolution = data.info.resolution
        map_width = data.info.width
        map_height = data.info.height

        map_array = np.reshape(data.data, [map_height, map_width])

        map_array = self.cropMap(map_array)

        map_array[map_array == 0] = 1
        map_array[map_array == 100] = -1


        grid = Grid(matrix=map_array)


        start = (map_array.shape[0]//2, map_array.shape[1]//2)
        end = (map_array.shape[0]//2+10, map_array.shape[1]//2-60)

        start_node = grid.node(start[0],start[1])
        end_node = grid.node(end[0], end[1])


        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start_node, end_node, grid)
        print('operations:', runs, 'path length:', len(path))

        image = self.map2Image(map_array, start, end, path)

        image_message = self.bridge.cv2_to_imgmsg(image) 
        self.pub.publish(image_message) #publish the image


    def cropMap(self, map):

        xs, ys = np.where(map>=0)
        result = map[min(xs):max(xs)+1,min(ys):max(ys)+1] 

        return result

    def map2Image(self, map, start, end, path):
        img = np.zeros((map.shape[0], map.shape[1], 3), np.int8)
        path = np.asarray(path)

        # paint the freespace of white
        free_space_idx = np.where(map==1)

        for x, y in zip(free_space_idx[0], free_space_idx[1]): 
            img[x,y,:] = 255

        #paint path
        for point in path:
            img[point[0],point[1],0] = 0
            img[point[0],point[1],1] = 255
            img[point[0],point[1],2] = 0
        
        #Paint start
        img[start[0],start[1],0] = 255
        img[start[0],start[1],1] = 0
        img[start[0],start[1],2] = 0

        #Paint end
        img[end[0],end[1],0] = 0
        img[end[0],end[1],1] = 0
        img[end[0],end[1],2] = 255
        
        return img

def main():

    rospy.init_node('path_finding')

    pf = Path_finding()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")