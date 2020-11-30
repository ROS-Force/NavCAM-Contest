#!/usr/bin/python

from logging import disable
from numpy.core.fromnumeric import mean
import rospy
import math
import sys
import random as rand
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image



class Distances():

    def __init__(self):
        
        self.roiSize = 5 #tamanho do lado que quero calcular a distancia (Area = 25pixeis)
        

        #Subscriber
        self.sensor_Subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dataCallback, queue_size=10)

        

    
    def dataCallback(self, new_data):

        #obtem as informacoes do tamanho do video
        self.height = new_data.height 
        self.width = new_data.width
        
        #transforma o array de 1D para 2D

        # Nao sei bem porque motivo o Python esta a considerar que new_data.data e uma String 
        # quando na verdade e um uint8[] (segundo rosmsg info sensor_msg/Image)
        
        self.distances = new_data.data
        self.distances.shape = (self.distances.size//self.width, self.width)

        #calcula a media das distancias da area de interesse
        roi_distance = mean(self.distances[int(round(self.height/2))-self.roiSize : int(round(self.height/2)) + self.roiSize][int(round(self.width/2))-self.roiSize : int(round(self.width/2)) + self.roiSize])
        
        #calcula a distancia minima e maxima
        min_distance = min(self.distances)
        max_distance = max(self.distances)

        print(roi_distance, min_distance, max_distance) #da um print

    #def calcRealDistance(self, d):
        
    #    real_distances = d * 0.0384

    #    return real_distances

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