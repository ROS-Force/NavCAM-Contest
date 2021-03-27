#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from __future__ import print_function
##################################### Header ############################################
""" obstruction.py: Description of the node """
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

		#acel_subscriber=rospy.Subscriber("/camera/accel/sample", Imu, self.acelCallback)
		
		#gyro_subscriber=rospy.Subscriber("/camera/gyro/sample", Imu, self.gyroCallback)

		#to subscribe to this topic you must use the magwick filter. So both accel and gyro topics must be united
		angles_subscriber=rospy.Subscriber("/imu/data", Imu, self.anglesCallback)
		
	def anglesCallback(self,angles_message):

		self.x = angles_message.orientation.x
		self.y = angles_message.orientation.y
		self.z = angles_message.orientation.z
		self.w = angles_message.orientation.w


		theta=math.asin(-2*(self.x*self.z-self.w*self.y))

		phi=math.asin(2*(self.w*self.z+self.x*self.y)/math.cos(theta))

		#por alguma razão eles são 180-psi=psi2 sao complementares mas devido ao facto de usar o inverso
		#do coseno em vez do inverso do seno em um deles

		#psi=math.acos(2*((self.w**2)+(self.z**2)-0.5)/math.cos(theta))

		psi2=math.asin(2*(self.w*self.x+self.y*self.z)/math.cos(theta))


		theta=(theta*180)/(math.pi)
		phi=(phi*180)/(math.pi)
		#psi=(psi*180)/(math.pi)
		psi2=(psi2*180)/(math.pi)
		#psi3=psi-psi2

		rospy.loginfo("Rotation around y=%s de z=%s de x=%s degrees"%(theta,phi,psi2))

		
	def acelCallback(self,accel_message):
		

		#self.header=accel_message.header.seq
		self.x_accel=accel_message.linear_acceleration.x
		self.y_accel=accel_message.linear_acceleration.y
		self.z_accel=accel_message.linear_acceleration.z


		##rospy.loginfo("sequency= %s"%(self.header))

	def gyroCallback(self,velocity_message):
		

		#self.header_gyro=accel_message.header.seq
		self.x_gyro=velocity_message.angular_velocity.x
		self.y_gyro=velocity_message.angular_velocity.y
		self.z_gyro=velocity_message.angular_velocity.z


		##rospy.loginfo("sequency= %s"%(self.header))


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

