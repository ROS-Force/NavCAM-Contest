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

		#aceleracao_grav=-9.86

		acel_subscriber=rospy.Subscriber("/camera/accel/sample", Imu, self.acelCallback)
		
		gyro_subscriber=rospy.Subscriber("/camera/gyro/sample", Imu, self.gyroCallback)

		frequency=250

		self.rate = rospy.Rate(frequency)
		i=0
		acel_grav_array=[]
		indicator=0;
		while not rospy.is_shutdown():

			#tempo de calibração
			while(i<frequency*1.2):

				while (self.y==0 and self.y==self.x):
					i=i;

				acel_grav=math.sqrt(((self.x**2)+(self.y**2))+(self.z**2))

				acel_grav_array.append(acel_grav)

				i=i+1

				self.rate.sleep()


			acel_grav_ref=sum(acel_grav_array)/len(acel_grav_array)

			while(indicator==0):

				rospy.loginfo("Acelera gravitica média é x=%s m/s²"%(acel_grav_ref))
				#calculo do angulo inicial
				if(abs(self.x)<acel_grav_ref and abs(self.y)<acel_grav_ref and abs(self.z)<acel_grav_ref):

						self.angulo_x=math.acos(self.x/acel_grav_ref)
						self.angulo_y=math.acos(self.y/acel_grav_ref)
						self.angulo_z=math.acos(self.z/acel_grav_ref)

						#self.angulo_x=(self.angulo_x*180)/(math.pi)
						#self.angulo_y=(self.angulo_y*180)/(math.pi)
						#self.angulo_z=(self.angulo_z*180)/(math.pi)

						indicator=1
			self.rate.sleep()
			#calculo da variação angulo seguinte
			self.angulo_z=self.angulo_x+self.x_gyro*frequency**(-1)

			self.angulo_y=self.angulo_y+self.y_gyro*frequency**(-1)

			self.angulo_x=self.angulo_z+self.z_gyro*frequency**(-1)

			#passagem para graus, posso fazer alguma coisa relativamente a dar uma valor precentual à rotação em torno de cada eixo mas isto necessitaria de varios calculos mais avançados sem certeza que de facto funcionaria
			self.angulo_x_graus=(self.angulo_x*180)/(math.pi)
			self.angulo_y_graus=(self.angulo_y*180)/(math.pi)
			self.angulo_z_graus=(self.angulo_z*180)/(math.pi)

			self.rate.sleep()

				#else:
				#	rospy.loginfo("Terminado-->dispositivo em movimento")
				#	break
			rospy.loginfo("Inclinação em x=%s em y=%s em z=%s rad"%(self.angulo_x_graus,self.angulo_y_graus,self.angulo_z_graus))




			


			
	def acelCallback(self,accel_message):
		

		#self.header=accel_message.header.seq
		self.x=accel_message.linear_acceleration.x
		self.y=accel_message.linear_acceleration.y
		self.z=accel_message.linear_acceleration.z


		##rospy.loginfo("sequência= %s"%(self.header))

	def gyroCallback(self,velocity_message):
		

		#self.header_gyro=accel_message.header.seq
		self.x_gyro=velocity_message.angular_velocity.x
		self.y_gyro=velocity_message.angular_velocity.y
		self.z_gyro=velocity_message.angular_velocity.z


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
