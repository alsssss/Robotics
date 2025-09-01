#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2

def callback(msg):
    rospy.loginfo("Ricevuto PointCloud2 con {} punti.".format(msg.width * msg.height))
    
    # Leggi i primi 5 punti per verificarne i valori
    points = list(pcl2.read_points(msg, field_names=['x', 'y', 'z', 'time', 'ring'], skip_nans=True))
    
    for i, p in enumerate(points[:5]):  # Stampiamo solo i primi 5 punti
        print(f"Point {i}: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}, time={p[3]:.6f}, ring={int(p[4])}")

def listener():
    rospy.init_node('pointcloud_checker', anonymous=True)
    rospy.Subscriber('/lidar', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
