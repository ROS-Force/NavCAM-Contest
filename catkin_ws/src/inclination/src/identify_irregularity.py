#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from __future__ import print_function
##################################### Header ############################################
""" identify_irregularity.py: Description of the node """
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


class identify_irregularity():

	def __init__(self):

		self.header=0
		self.x=0
		self.y=0
		self.z=0

		acel_subscriber=rospy.Subscriber("/imu/data", Imu, self.acelCallback)

		#frequency of the while cicle 
		frequency=385

		self.rate = rospy.Rate(frequency)
		acel_array=[]
		standart_dev_array=[]
		inclination_identifier=1

		while not rospy.is_shutdown():

			#calibration time
			if(inclination_identifier==1):
				i1=0
				i2=0
				rospy.loginfo("recalibrating")

				while(i1<frequency*0.75):

					acel_array.append(self.y)

					i1=i1+1

					self.rate.sleep()

				acel_med=sum(acel_array)/len(acel_array)

				#rospy.loginfo("Acel med=%s"%(acel_med))

				while(i2<frequency*0.75):

					standart_dev_array=[(self.y-acel_med)**2]

					i2=i2+1

					self.rate.sleep()

				sum_med=sum(standart_dev_array)

				standart_dev=math.sqrt(sum_med/frequency*0.75)

				#rospy.loginfo("standart dev=%s"%(standart_dev))

				inclination_identifier=0;

			else:
				#check if theres any sudden variation in the vertical acelaration, that is 2 times over the standart deviation values calculated on calibration
				if(self.y>(acel_med+2*standart_dev) or self.y<(acel_med-2*standart_dev)):

					rospy.loginfo("A instability or a sudden change of inclination was found!")

					inclination_identifier=1

		self.rate.sleep()

	def acelCallback(self,accel_message):
		
		self.x=accel_message.linear_acceleration.x
		self.y=accel_message.linear_acceleration.y
		self.z=accel_message.linear_acceleration.z

def main():
	rospy.init_node('identify_irregularity', anonymous=True)
	obst = identify_irregularity()

	rospy.spin()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)
