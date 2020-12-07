#!/usr/bin/python

from logging import disable
from numpy.core.fromnumeric import mean
import rospy
import math
import sys
import random as rand
import numpy as np
import panda as pd

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image



class Distances():

    def __init__(self):
        
        
        #Subscriber
        self.sensor_Subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dataCallback, queue_size=10)

        

    
    def dataCallback(self, new_data):


        roiSize = 5 #tamanho do lado que quero calcular a distancia (Area = 25pixeis)

        dist = np.fromstring(new_data.data, dtype=np.int16) # Converte os dados de String para num array de Numpy int16[]

        height = new_data.height
        width = new_data.width

        #transforma o array de 1D para 2D
        distances = dist.reshape(height, width)
        roiDistances = distances[height//2 - roiSize:roiSize + height//2, width//2 - roiSize:width//2]
        self.List = self.List.append(pd.reshape(distances, height*width))
        

        
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pass
            rate.sleep()

def main():

    rospy.init_node('distances', anonymous=True)

    n = Distances()

    n.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")