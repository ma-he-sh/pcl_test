#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy as rnp

class ObsDetector():
    def __init__(self):
        try:
            maxDistance = rospy.get_param('/max_distance')
            print(maxDistance)
            minDistance = rospy.get_param('/min_distance')
            print(minDistance)
        except ROSException:
            print("could not get param name")   
        
        while not rospy.is_shutdown():
            rospy.spin()     

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = ObsDetector()
        detect.run()
    except KeyboardInterrupt:
        print('closing')