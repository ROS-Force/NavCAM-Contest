#!/usr/bin/env python3

from numpy.core.fromnumeric import diagonal
from pathfinding.core import diagonal_movement
import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2
import sys
from rospy.core import rospyerr
import tf2_ros

from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from nav_msgs.srv import GetMap

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.best_first import BestFirst
from pathfinding.finder.bi_a_star import BiAStarFinder
from pathfinding.finder.breadth_first import BreadthFirstFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from pathfinding.finder.ida_star import IDAStarFinder
from pathfinding.finder.msp import MinimumSpanningTree
from datetime import datetime

import tensorflow as tf


class PathFindingROS():

    def __init__(self):

        self.algorithm = rospy.get_param("~algorithm", "A*") #Name of the algorithm
        self.map_frame = rospy.get_param("~map_frame", "map") #Name of map frame
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        # Initialize variables

        self.intrinsics = None
        self.bridge = CvBridge()
        self.latestGoal = None
        self.stopPathFinding = False

        self.tfbuffer = tf2_ros.Buffer(debug=False)
        self.tflistener = tf2_ros.TransformListener(self.tfbuffer)
        

        if(self.algorithm == "A*"):
            self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "bi_A*"):
            self.finder = BiAStarFinder(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "best first"):
            self.finder = BestFirst(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "BFS"):
            self.finder = BreadthFirstFinder(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "dijkstra"):
            self.finder = DijkstraFinder(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "IDA*"):
            self.finder = IDAStarFinder(diagonal_movement=DiagonalMovement.always)
        elif(self.algorithm == "MST"):
            self.finder = MinimumSpanningTree(diagonal_movement=DiagonalMovement.always)
        else:
            rospyerr("Invalid path finding algorithm: %s", self.algorithm)
        
        self.currentMap = None
        # Subscriber
        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                                    self.goalCallback, queue_size=10, buff_size=2**24)

        # Publisher (update this )
        self.pub_path = rospy.Publisher("path", Path, queue_size=1)
        # remove after debug is done
        self.pub = rospy.Publisher("path_image", Image, queue_size=1)

    def goalCallback(self, pose_stamped):

        self.latestGoal = np.array(
            [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], dtype=np.float)
        print(pose_stamped)

    def getCurrentMap(self):

        rospy.wait_for_service('rtabmap/get_map')
        try:
            dynamicMap = rospy.ServiceProxy('rtabmap/get_map', GetMap)
            dynamicMapResult = dynamicMap()

            return dynamicMapResult.map
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None

# Callbacks
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
        map_array, origin_dist = PathFindingROS.__cropMap(
            latestGrid.data, map_width, map_height)

        # invert
        #origin_dist = origin_dist[::-1]

        grid = Grid(matrix=map_array)
        mapShape = np.array(map_array.shape).astype(np.uint)

        mapOriginPosition = np.array(
            [origin.position.x, origin.position.y, origin.position.z], dtype=np.float)

        goalPosition = (((self.latestGoal - mapOriginPosition) /
                        resolution)[:2] - origin_dist).astype(np.int)

        goalPosition = np.where(goalPosition <= 0, 0, goalPosition)
        goalPosition = np.where(goalPosition >= mapShape,
                                (mapShape-1), goalPosition)

        # get current 2d position
        currentPosition = (((self.__getCurrentPosition(
        ) - mapOriginPosition)/resolution)[:2] - origin_dist).astype(np.int)
        currentPosition = np.where(currentPosition <= 0, 0, currentPosition)
        currentPosition = np.where(
            currentPosition >= mapShape, mapShape-1, currentPosition)

        print(currentPosition, goalPosition)
        start_node = grid.node(
            int(currentPosition[0]), int(currentPosition[1]))
        end_node = grid.node(int(goalPosition[0]), int(goalPosition[1]))

        path, runs = self.finder.find_path(start_node, end_node, grid)

        # convert to np
        path = np.array(path)

        print('operations:', runs, 'path length:', len(path))

        # Remove after debug is done
        image = self.__map2Image(
            map_array, [int(currentPosition[1]), int(currentPosition[0])], [int(goalPosition[1]), int(goalPosition[0])], path)
        image_message = self.bridge.cv2_to_imgmsg(image)
        self.pub.publish(image_message)  # publish the image
        path_transformed = np.empty((path.shape[0], 3), dtype=np.float)

        for i in range(len(path)):
            path[i] += origin_dist

            oldPose = np.append(path[i], 0).astype(np.float)
            path_transformed[i] = ((oldPose * resolution) + mapOriginPosition)

        print(path)

        print(path_transformed)

        self.publishPath(path_transformed)
        print("Fim: ", datetime.now())

    def run(self):

        # main thread will now be used to update periodically

        sleepRate = rospy.Rate(1)

        while(not rospy.is_shutdown()):
            self.pathUpdateCallback()
            sleepRate.sleep()

    def __getCurrentPosition(self):
        if not (self.tfbuffer.can_transform(target_frame=self.map_frame, source_frame=self.base_frame, time=rospy.Duration())):
            print("Error, can't obtain transform from %s to %s", self.base_frame, self.map_frame)
            return None
        else:
            try:
                transform = self.tfbuffer.lookup_transform(
                    self.map_frame, self.base_frame, rospy.Time(0))
                # get transform vectors
                return np.array([transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z], dtype=np.float)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error on transform")
                return None

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
            pose_stamped.pose.orientation.w = 1.0

            list_poses.append(pose_stamped)

        # Create the Header
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
        free_space_idx = np.where(map == 1)

        for x, y in zip(free_space_idx[0], free_space_idx[1]):
            img[y, x, :] = 255

        # paint path
        for point in path:
            img[point[0], point[1], 0] = 0
            img[point[0], point[1], 1] = 255
            img[point[0], point[1], 2] = 0

        # Paint start
        img[start[1], start[0], 0] = 255
        img[start[1], start[0], 1] = 0
        img[start[1], start[0], 2] = 0

        # Paint end
        img[end[1], end[0], 0] = 0
        img[end[1], end[0], 1] = 0
        img[end[1], end[0], 2] = 255

        return img

    @staticmethod
    def __cropMap(data, map_width, map_height):
        print(datetime.now(), "Before tensor conv")
        map_tf = tf.convert_to_tensor(np.asarray(data))
        print(datetime.now(), "After tensor conv")
        map_tf = tf.reshape(map_tf, [map_height, map_width])
        print(datetime.now(), "Before searching and reducing columns")

        # sum columns
        #indexes = tf.where(map_tf >= 0)
        #min_idx = tf.reduce_min(indexes, axis=0)
        #max_idx = tf.reduce_max(indexes, axis=0)

        print(datetime.now(), "After summing columns")
        #result = map_tf[min_idx[0]:max_idx[0], min_idx[1]:max_idx[1]]
        result = map_tf[:]
        print(datetime.now(), "After slicing columns")

        result = tf.where(result == 0, x=1, y=result)
        result = tf.where(result == 100, x=-1, y=result)
        # min_inx.numpy()
        return result.numpy(), np.array([0, 0])


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
