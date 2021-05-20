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
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
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
        self.latestGoal = None
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

        
        self.pub_pose = rospy.Publisher("pose_test", PoseStamped, queue_size=1)
        self.pub_pose_orig = rospy.Publisher("pose_test_orig", PoseStamped, queue_size=1)

        #Publisher (update this )
        self.pub_path = rospy.Publisher("path", Path, queue_size=1)
        self.pub = rospy.Publisher("path_image", Image, queue_size=1) #remove after debug is done

    def goalCallback(self, pose_stamped):

        self.latestGoal = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], dtype=np.float)
        print(pose_stamped)


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
        if (latestGrid is None or self.latestGoal is None):
            return

        header = latestGrid.header
        origin = latestGrid.info.origin
        resolution = latestGrid.info.resolution
        map_width = latestGrid.info.width
        map_height = latestGrid.info.height
        map_array, origin_dist = PathFindingROS.__cropMap(latestGrid.data, map_width, map_height)                
        grid = Grid(matrix=map_array)
        
        mapShape = np.array(map_array.shape).astype(np.uint)
        print("shape", mapShape)
        
        mapPosition = mapShape.astype(np.int)
        #mapPosition[0] = 0

        mapOriginPosition = np.array([origin.position.x,origin.position.y,origin.position.z], dtype=np.float)

        realOriginPosition = np.append(((origin_dist + mapPosition).astype(np.float)* resolution), 0)
        realOriginPosition += mapOriginPosition
        
        originPosition = Pose(position=Point(realOriginPosition[0],realOriginPosition[1],realOriginPosition[2]))
        self.pub_pose.publish(PoseStamped(header = header, pose = originPosition))
        

        realOriginPosition = np.append(((origin_dist).astype(np.float)* resolution), 0)
        realOriginPosition += mapOriginPosition
        
        originPosition = Pose(position=Point(realOriginPosition[0],realOriginPosition[1],realOriginPosition[2]))
        self.pub_pose_orig.publish(PoseStamped(header = header, pose = originPosition))
        return 

        print(header)
        #goalPositon = np.array([self.goal.position.x,self.goal.position.y,self.goal.position.z], dtype=np.float)
        mapOriginPosition = np.array([origin.position.x,origin.position.y,origin.position.z], dtype=np.float)

        print("real in m", self.latestGoal, self.__getCurrentPosition())
        goalPosition = (((self.latestGoal - mapOriginPosition)/resolution)[:2] - origin_dist).astype(np.uint)

        print("debug goal", (((self.latestGoal - mapOriginPosition)/resolution)[:2] - origin_dist))
        goalPosition = np.where(goalPosition <= 0, 0, goalPosition)
        goalPosition = np.where(goalPosition >= mapShape, (mapShape-1), goalPosition)

        # get current 2d position
        currentPosition = (((self.__getCurrentPosition()- mapOriginPosition)/resolution)[:2] - origin_dist).astype(np.uint)
        print("debug pos", (((self.__getCurrentPosition()- mapOriginPosition)/resolution)[:2] - origin_dist))
        currentPosition = np.where(currentPosition <= 0, 0, currentPosition)
        currentPosition = np.where(currentPosition >= mapShape, mapShape-1, currentPosition)

        print(currentPosition, goalPosition)
        start_node = grid.node(currentPosition[0], currentPosition[1])
        end_node = grid.node(goalPosition[0], goalPosition[1])


        path, runs = self.finder.find_path(start_node, end_node, grid)
        
        # convert to np
        path = np.array(path)

        #print(datetime.now(), "before map conv")
        print('operations:', runs, 'path length:', len(path))

        #Remove after debug is done
        image = self.__map2Image(map_array, currentPosition, goalPosition, path)
        image_message = self.bridge.cv2_to_imgmsg(image) 
        self.pub.publish(image_message) #publish the image

        for point in path:
            point += origin_dist

        path_transformed=np.empty((path.shape[0], 3), dtype=np.float)

        print(path)
        for (newPose, oldPose) in zip(path_transformed, path):
            oldPose = np.append(oldPose, 0).astype(np.float)
            
            print(oldPose, oldPose*resolution, mapOriginPosition)
            newPose = ((oldPose * resolution) + mapOriginPosition)
            print(newPose)
            
        print(path_transformed)


        #path_transformed = self.__transformPose(path, origin, resolution, toFrame="odom", fromFrame="map")
        
        self.publishPath(path_transformed)

        print("Fim: ", datetime.now())

        

    def run(self):  
        
        # main thread will now be used to update periodically A-star path

        sleepRate = rospy.Rate(500)

        while(not rospy.is_shutdown()):
            self.pathUpdateCallback()
            sleepRate.sleep()

    def __getCurrentPosition(self):
        if not (self.tfbuffer.can_transform(target_frame="map", source_frame="odom", time=rospy.Duration())):
            print("Error, can't obtain transform from 'odom' to 'map'")
            return None
        else:
            try:
                transform = self.tfbuffer.lookup_transform("map", "odom", rospy.Time(0))
                # get transform vectors
                return np.array([transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z], dtype=np.float)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error on transform")
                return None

    def __transformPose(self, poseMatrix, origin, resolution, fromFrame, toFrame):
        # check if we can transform to target frame
        if not (self.tfbuffer.can_transform(target_frame=toFrame, source_frame=fromFrame, time=rospy.Duration())):
            print("Error, can't obtain transform from '{}' to '{}'".format(fromFrame, toFrame))
            return None
        else:
            # get transform 
            translation_vector = None
            rotation_quarternion = None

            try:
                transform = self.tfbuffer.lookup_transform(toFrame, fromFrame, rospy.Time(0))
                # get transform vectors
                translation_vector=np.array([transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z], dtype=np.float)

                rotation_quarternion=np.array([transform.transform.rotation.x,
                    transform.transform.rotation.y, 
                    transform.transform.rotation.z, 
                    transform.transform.rotation.w])

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error on transform")
                return None

        newPoses = None
        print(poseMatrix.shape)
        shape = poseMatrix.shape

        if (fromFrame == "map" and toFrame=="odom"):
            shape = (shape[0], 3) # change to 3 dimensions
        elif (fromFrame == "odom" and toFrame=="map"):
            shape = (shape[0], 2) # change to 2 dimensions

        newPoses=np.empty(shape, dtype=np.float)

        print(poseMatrix)
        
        for (newPose, oldPose) in zip(newPoses, poseMatrix):
            print(origin)
            originPosition = np.array([origin.position.x,origin.position.y,origin.position.z])
            #print(pose)

            # pixel to meter
            if (fromFrame == "map" and toFrame=="odom"):
                # add dummy dimension 
                oldPose = np.append(oldPose, 0).astype(np.float)
                print(resolution, oldPose, originPosition, translation_vector)
                newPose = ((oldPose * resolution) + originPosition) + translation_vector
                print(newPose)
            # meter to pixel
            elif (fromFrame == "odom" and toFrame=="map"):
                newPose_stub = ((oldPose + translation_vector) - originPosition) / resolution
                newPose = newPose_stub[:2].astype(np.int)
            else:
                newPose = oldPose + translation_vector

            #print(pose)
            
        print(newPoses)
        
        return newPoses

    def publishPath(self, array):
        
        list_poses = []
        pose_stamped = PoseStamped()
        path = Path()

        for p in array:

            pose_stamped.header = Header()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = p[0]
            pose_stamped.pose.position.y = p[1]
            pose_stamped.pose.position.z = p[2]

            list_poses.append(pose_stamped)
        
        #Create the Header
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        path.header = h
        path.poses = list_poses

        self.pub_path.publish(path)



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
        result = tf.where(result == 100, x=-1, y=result)

        return result.numpy(), min_idx.numpy()



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