#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from __future__ import print_function
##################################### Header ############################################
""" inclination_through_madgwick.py: Description of the node """
#__author__ = ""
#__credits__ = [""]
#__version__ = "0.0.0"
#__maintainer__ = ""
#__email__ = ""
#########################################################################################
# import any libraries necessary to run script
import roslib
import rospy
import math
import time 
import sys
import random
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class orientation():

	def __init__(self):

		#roslaunch inclination imu_filter_madgwick.launch is used to publish the following topic
		#/imu/data publishes the orientation of the camera in the quaternion format
		
		angles_subscriber=rospy.Subscriber("/imu/data", Imu, self.anglesCallback)
	
	#callback function 
	def anglesCallback(self,angles_message):


		self.x = angles_message.orientation.x
		self.y = angles_message.orientation.y
		self.z = angles_message.orientation.z
		self.w = angles_message.orientation.w

		#theta is rotation aroun the z axis by theta rad
		theta=math.asin(-2*(self.x*self.z-self.w*self.y))

		#around the y axis of the camera
		phi=math.asin(2*(self.w*self.z+self.x*self.y)/math.cos(theta))

		#psi and psi2 are complementary angles, 180-psi=psi2
		#psi=math.acos(2*((self.w**2)+(self.z**2)-0.5)/math.cos(theta))

		#around the x axis
		psi2=math.asin(2*(self.w*self.x+self.y*self.z)/math.cos(theta))

		#converting rad to degrees
		theta=(theta*180)/(math.pi)
		phi=(phi*180)/(math.pi)
		psi2=(psi2*180)/(math.pi)
		
		rospy.loginfo("Rotation around z=%s and y=%s and x=%s degrees"%(theta,phi,psi2))



def main():
	rospy.init_node('orientation', anonymous=True)
	obst = orientation()

	rospy.spin()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)

