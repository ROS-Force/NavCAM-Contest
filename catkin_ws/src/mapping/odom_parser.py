#!/usr/bin/env python3

import rospy
import numpy as np
import re, sys
from dateutil import parser as dateparser

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class OdomParserNode():
    # regex expression to parse odometry file
    REGEX_EXP="\[(\d{4}\-\d{2}\-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)Z)\] COORDS :: T\[(\d+)\] X\[(\-?\d+(?:\.\d+))\] Y\[(\-?\d+(?:\.\d+))\] H\[(\-?\d+(?:\.\d+))\]" 

    def __init__(self):

        #Publisher
        self.pub = rospy.Publisher("asa_odom", PoseStamped, queue_size=1)
        self.odomData = None
        odom_file_path = rospy.get_param("~odom_file_path", default=None)
        print(odom_file_path)

        if (odom_file_path is None):
            rospy.logerr("Error, the odom file path is not set. Closing...")
            sys.exit(0)
        else:
            with open(odom_file_path, 'r') as file:
                self.odomData = file.readlines()

        # we will assume the first pose to be 0 and the rest are offsets
        _, _, x, y, h = OdomParserNode.odomFromData(self.odomData[0])
        self.initial_pose = {'x' : x, 'y' : y, 'h' : h}

    @staticmethod
    def odomFromData(data):
        timestamp, timestamp_ms, x, y, h = re.search(OdomParserNode.REGEX_EXP, data).groups()
        return dateparser.isoparse(timestamp), timestamp_ms, x, y, h

    def run(self):
        msg_seq = 0 
        for odom_entry in self.odomData:
            msg_seq += 1
            timestamp, _, x, y, h = OdomParserNode.odomFromData(odom_entry)

            # sleep until publish time
            rospy.sleep(rospy.Duration.from_sec(rospy.get_time() - timestamp.timestamp() - 0.01))
            self.pub.publish(PoseStamped(
                header=Header(
                    seq = msg_seq,
                    stamp = rospy.Time.from_sec(timestamp.timestamp()), 
                    frame_id = 'map'), 
                pose=PoseStamped(
                    position=Point(x=(x - self.initial_pose['x']), y=(y-self.inital_pose['y']), z=0),
                    orientation=Quaternion(x=0,y=0,z=np.sin((h - self.inital_pose['h'])/2), w=np.cos((h - self.inital_pose['h'])/2))
                )))

def main():

    rospy.init_node('deeplab_segmentation')
    node = OdomParserNode()
    node.run()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")