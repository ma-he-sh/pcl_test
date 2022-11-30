#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy as rnp

class ObsDetector():
    def __init__(self):
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_handler)
        rospy.spin()

    def point_cloud_handler(self, point_cloud):
        pc_np = rnp.point_cloud2.pointcloud2_to_xyz_array(point_cloud,remove_nans=True)
        distance_arr = np.zeros(shape=(pc_np.shape[0],1))

        minDistance = 0.0
        maxDistance = 0.0
        
        if len(distance_arr) > 0:
            for i in range(pc_np.shape[0]):
                distance = math.sqrt(pc_np[i,0]**2 + pc_np[i,1]**2 + pc_np[i,2]**2)
                distance_arr[i,0] = distance

            minDistance = float(np.amin(distance_arr))
            maxDistance = float(np.amax(distance_arr))

        rospy.set_param("/max_distance", maxDistance)
        rospy.set_param("/min_distance", minDistance)

        # print(np.amin(distance_array))
        #print("Minimum distance index is: " + str(np.amin(distance_arr)))
        #print("Maximum distance index is: " + str(np.amax(distance_arr)))

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = ObsDetector()
    except KeyboardInterrupt:
        print('closing')