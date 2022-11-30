#!/usr/bin/env python3

import rospy
import struct

from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import ctypes
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header, ColorRGBA

class BoundingBox():
    def __init__(self):
        self.frame_id = "camera_link"
        self.publisher = rospy.Publisher("/observation/bounding", MarkerArray , queue_size=0)
        self.obsPublisher = rospy.Publisher("/observation/obstacles", ObstacleArrayMsg, queue_size=0)
        rospy.Subscriber("/voxel/only_objects", PointCloud2, self.pointcloudHandler)
        rospy.spin()

    def create_obstacle(self, index, center):
        obstacle = ObstacleMsg()
        obstacle.id = index
        
        # TODO add the points of the object detected
        v1 = Point32()
        v1.x = 0
        v1.y = 1
        v2 = Point32()
        v2.x = 0
        v2.y = 1
        v3 = Point32()
        v3.x = 0
        v3.y = 1
        v4 = Point32()
        v4.x = 0
        v4.y = 1
    
        obstacle.polygon.points = [v1, v2, v3, v4]
        return obstacles


    def create_box(self, index, center):
        box = Marker(
            type=Marker.CUBE,
            id=index,
            lifetime=rospy.Duration(1.5),
            pose=Pose(Point(center[0], center[1], center[2]), Quaternion(1.0, 1.0, 1.0, 1.0)),
            scale=Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id=self.frame_id),
            color=ColorRGBA( 255.0 , 0.0, 255.0, 0.65)
        )

        return box

    def create_label(self, index, text, center):
        label = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id= index,
            lifetime=rospy.Duration(1.5),
            pose=Pose(Point(center[0], center[1], center[2]), Quaternion(1.0, 1.0, 1.0, 1.0)),
            scale=Vector3(0.03, 0.03, 0.03),
            header=Header(frame_id=self.frame_id),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
            text=text)

        return label

    def bounding_box(self, points):
        x = points["x"]
        y = points["y"]
        z = points["z"]

        x_min = min( x )
        x_max = max( x )
        y_min = min( y )
        y_max = max( y )
        z_min = min( z )
        z_max = max( z )

        width = x_min - x_max
        height= y_min - y_max
        depth = z_min - z_max

        center = ( ( x_min + x_max ) / 2, ( y_min + y_max ) / 2, ( z_min + z_max ) / 2 )

        return center, width, height


    def pointcloudHandler(self, point_cloud):
        global iteration


        # obstacle Msg
        obstacleMsg = ObstacleArrayMsg()
        obstacleMsg.header.stamp = rospy.Time.now()
        obstacleMsg.frame_id = self.frame_id

        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])

        gen = pc2.read_points(point_cloud, skip_nans=True)
        int_data = list(gen)

        #print( point_cloud.width, point_cloud.height, point_cloud.point_step, point_cloud.row_step, point_cloud.fields )

        cluster = {}
        obj_id  = 0

        x_arr = []
        y_arr = []
        z_arr = []
        for x in int_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0) # x[2]
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

            if( b == 255 ):
                if r not in cluster:
                    cluster[r] = {
                        "points": {
                            "x" : [],
                            "y" : [],
                            "z" : []
                        },
                        "object": obj_id
                    }
                    obj_id+=1
                cluster[r]["points"]["x"].append(x[0])
                cluster[r]["points"]["y"].append(x[1])
                cluster[r]["points"]["z"].append(x[2])

            x_arr.append(x[0])
            y_arr.append(x[1])
            z_arr.append(x[2])

        #print(len(cluster))
        markers = []

        if len(cluster) > 0:
            _index = 0
            _obs_index = 0
            _marker_id = 0
            for objKey in cluster:
                #print(objKey)
                points = cluster[objKey]["points"]
                if len(points["x"]) <= 2: continue

                _bb_center, width, height = self.bounding_box( points )

                for idx, x in enumerate(points['x']):
                    center = (points['x'][idx], points['y'][idx], points['z'][idx])
                    box = self.create_box(_marker_id, center)
                    markers.append(box)

                    _marker_id+=1

                label = self.create_label(_index+1, "obs::" + str(_obs_index), _bb_center)
                markers.append(label)

                _index+=2
                _obs_index+=1

        if len(markers) != 0:
            self.publisher.publish(markers)

        iteration+=1

if __name__ == "__main__":
    global iteration
    iteration = 0

    rospy.init_node( "pcl_test", anonymous=True )
    try:
        boundingBox = BoundingBox()
    except KeyboardInterrupt:
        print("closed")
        vis.destroy_window()

