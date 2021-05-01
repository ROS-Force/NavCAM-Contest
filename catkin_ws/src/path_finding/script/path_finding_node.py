#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2
import sys
import tf2_ros, tf.transformations as tf2_trans

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from datetime import datetime

import tensorflow as tf


class PathFindingROS():

    def __init__(self):

        #Initialize variables

        self.intrinsics = None
        self.bridge = CvBridge()
        self.path = None
        self.goal = None
        self.stopPathFinding = False        

        #Publishers
        #self.pub = rospy.Publisher("output_image", Image, queue_size=1)
        #self.pub_obj = rospy.Publisher("object_info", Object_info, queue_size=1)

        self.tfbuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuffer) 
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

        #Subscriber
        #self.sub = rospy.Subscriber("map", OccupancyGrid, self.mapCallback, queue_size=10, buff_size=2**24)

        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback, queue_size=10, buff_size=2**24)

        #Publisher (update this )
        self.pub = rospy.Publisher("path_image", Image, queue_size=1)

    def goalCallback(self, pose_stamped):

        self.goal = [pose_stamped.pose.position.x, pose_stamped.pose.position.y]
        print(self.goal)


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

        print("Inicio: ", datetime.now())
        # skip updates without map
        latestGrid = self.getCurrentMap()
        if (latestGrid is None):
            return
        
        header = latestGrid.header
        origin = latestGrid.info.origin
        resolution = latestGrid.info.resolution
        map_width = latestGrid.info.width
        map_height = latestGrid.info.height
        map_array = PathFindingROS.__cropMapTF(latestGrid.data, map_width, map_height)


        grid = Grid(matrix=map_array)

        # set
        start = np.array([map_array.shape[0]//2, map_array.shape[1]//2])
        end = start + 50

        start_node = grid.node(start[0],start[1])
        end_node = grid.node(end[0], end[1])

        #print(datetime.now(), "before A star")
        
        path, runs = self.finder.find_path(start_node, end_node, grid)

        
        #print(datetime.now(), "before map conv")
        print('operations:', runs, 'path length:', len(path))

        image = PathFindingROS.__map2Image(map_array, start, end, path)
        self.path = path

        print("Fim: ", datetime.now())
        
        image_message = self.bridge.cv2_to_imgmsg(image) 
        self.pub.publish(image_message) #publish the image
        

    def run(self):  
        
        # main thread will now be used to update periodically A-star path

        sleepRate = rospy.Rate(500)

        while(not rospy.is_shutdown()):
            self.__transformPose(None, fromFrame="camera_link", toFrame="map")
            #self.pathUpdateCallback()
            sleepRate.sleep()

    def __transformPose(self, poseMatrix, fromFrame, toFrame):
        # check if we can transform to target frame
        if not (self.tfbuffer.can_transform(target_frame=toFrame, source_frame=fromFrame, time=rospy.Duration())):
            print("Error, can't obtain transform")
        else:
            # get transform 
            try:
                transform = self.tfbuffer.lookup_transform(toFrame, fromFrame, rospy.Time(0))
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error on transform")
                pass
            
            
            
            
            tf2_trans.quaternion_multiply()
            quaternion_multiply(q_rot, q_orig)

            print(transform)
            
            #msg = Twist()
   
            #msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            #msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)



    @staticmethod
    def __map2Image(map, start, end, path):
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

    @staticmethod
    def __cropMap(data, map_width, map_height):
        print(datetime.now(), "Before tensor conv")
        map_tf = tf.convert_to_tensor(np.asarray(data))
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

def main():

    rospy.init_node('path_finding')

    # initialize node 
    pf = PathFindingROS()

    # run node
    pf.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")