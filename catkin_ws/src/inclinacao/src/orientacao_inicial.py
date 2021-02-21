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

		self.header=0
		self.x=0
		self.y=0
		self.z=0

		aceleracao_grav=-9.86

		acel_subscriber=rospy.Subscriber("/camera/accel/sample", Imu, self.acelCallback)
		
		self.rate = rospy.Rate(50)

		while not rospy.is_shutdown():

				if(abs(self.x)<9.86 and abs(self.y)<9.86 and abs(self.z)<9.86):

					self.angulo_x=math.acos(self.x/aceleracao_grav)
					self.angulo_y=math.acos(self.y/aceleracao_grav)
					self.angulo_z=math.acos(self.z/aceleracao_grav)

					self.angulo_x=(self.angulo_x*180)/(math.pi)-90
					self.angulo_y=(self.angulo_y*180)/(math.pi)
					self.angulo_z=(self.angulo_z*180)/(math.pi)-90

					rospy.loginfo("Inclinação em x=%s em y=%s em z=%s rad"%(self.angulo_x,self.angulo_y,self.angulo_z))
				else:
					rospy.loginfo("Terminado-->dispositivo em movimento")
					break

				self.rate.sleep()


			
	def acelCallback(self,accel_message):
		

		self.header=accel_message.header.seq
		self.x=accel_message.linear_acceleration.x
		self.y=accel_message.linear_acceleration.y
		self.z=accel_message.linear_acceleration.z

		##rospy.loginfo("sequência= %s"%(self.header))


def main():
	rospy.init_node('orientacao_inicial', anonymous=True)
	obst = orientation()

	rospy.spin()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)
