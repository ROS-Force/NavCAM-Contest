#!/usr/bin/python

from logging import disable
from numpy.core.fromnumeric import mean
import rospy
import math
import sys
import os
import numpy as np
import pandas as pd
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image



class ExportData():

    def __init__(self):
        
        #Stamps para controlar a versao da Lista
        self.stampToCompare = 0
        self.stamp=0

        #Subscriber
        self.sensor_Subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dataCallback, queue_size=10)
        
        #Timer
        self.timer = rospy.Timer(period=rospy.Duration(secs=0.5), callback=self.timerCallback)# e chamada a cada 0.5s para verificar 
                                                                                              # se a lista de valores esta a ser atualizada 
                                                                                              # se a a lista nao esta a ser atualizada e pq o rosbag 
                                                                                              # acabou e podemos fechar o nodo

    def dataCallback(self, new_data):


        roiSize = 5 #tamanho do lado que quero calcular a distancia (Area = 2*25)

        dist = np.fromstring(new_data.data, dtype=np.int16) # Converte os dados de String para num array de Numpy int16[]
        height = new_data.height
        width = new_data.width

        self.stamp=self.stamp+1

        #transforma o array de 1D para 2D
        distances = dist.reshape(height, width)
        roiDistances = distances[height//2 - roiSize:roiSize + height//2, width//2 - roiSize:width//2]
        
        try:
            self.List = np.append(self.List, roiDistances.reshape(2*roiSize**2))
        except AttributeError as ae:
            self.List = roiDistances.reshape(2*roiSize**2)
        
    
    def timerCallback(self, stuff): #verifica se a dataCallback foi chamada, 
                                    #se nao for chamada encerra/(reset na lista, o que preferirem) este nodo e exporta os dados
        if (self.stamp == self.stampToCompare and not self.stamp == 0):

            self.exportFile();
            #if True:
            #    self.List = []
            #else:
            rospy.signal_shutdown("Exportado para um ficheiro CSV!")

        else:
            self.stampToCompare = self.stamp


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rate.sleep()

    def exportFile(self):
        
        #Nome do ficheiro consoante as horas (pode-se fazer roslaunch disto e do rosbag play, 
        # e fica com msm nome do rosbag, mas ja e tarde e fica para o proximo commit)
        now = rospy.get_rostime()
        t = time.localtime()
        current_time = time.strftime("%H_%M_%S", t)

        #directoria mae e paths
        directories = ["~/NavCAM-Contest", "/Resultados", "/Files"]
        path = "".join(directories)
        filename= current_time + ".csv"
        filepath= path + "/" + filename

        try:
            pd.DataFrame(self.List).to_csv(filepath, header=None, index=None)
        except IOError as io:
            print("Cria tu as directorias que isto ja me esta a dar dor de cabeca")
            p=""
            for str in directories:
                p = p + str
                try:
                    os.mkdir(p)
                    print(p)
                except OSError as F:
                    print("F" + " in the chat")
        finally:
            try:
                pd.DataFrame(self.List).to_csv(filepath, header=None, index=None)
            except:
                print("Hmmm... Fuck")

def main():

    rospy.init_node('export_node', anonymous=True)

    n = ExportData()

    n.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")