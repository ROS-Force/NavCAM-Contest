#!/usr/bin/env python

import rospy

class test_node():

    def __init__(self):

        #Model Variables
        list_param = rospy.get_param_names()
        #for param in list_param:
        #    print(param, " -> ", rospy.get_param(param))
        
                # check how many models exist
        defaultModelPrefix = "/yolo_model"
        lightModelPrefix = "/yolo_light_model"

        #param_list = [param for param in rospy.get_param_names() if (defaultModelPrefix in param or lightModelPrefix in param)]
        print(rospy.get_param(defaultModelPrefix))
        rospy.spin()

def main():
    rospy.init_node('yolo_detection')
    hd = test_node()
    return 0

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")