#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def filter_pointcloud(msg):
    """ Filtra i punti con is_dense=False """
    if msg.is_dense:
        return msg  # Nessun filtraggio necessario
    
    points = list(pc2.read_points(msg, field_names=None, skip_nans=True))
    filtered_msg = pc2.create_cloud(msg.header, msg.fields, points)
    filtered_msg.is_dense = True  # Dopo il filtraggio, i NaN sono rimossi
    return filtered_msg

def callback(msg):
    filtered_msg = filter_pointcloud(msg)
    pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_scan', anonymous=True)
    rospy.Subscriber('/carla/24/front', PointCloud2, callback)
    pub = rospy.Publisher('/carla/24/front/filtered', PointCloud2, queue_size=10)
    rospy.spin()
