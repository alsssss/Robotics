#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
import struct

class PointCloudModifier:
    def __init__(self):
        rospy.init_node('pointcloud_modifier', anonymous=True)
        self.sub = rospy.Subscriber('/carla/base_link/lidar', PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/lidar', PointCloud2, queue_size=1)

    def callback(self, msg):
        # Estrarre i punti dalla nuvola
        points = list(pcl2.read_points(msg, field_names=None, skip_nans=True))
        
        # Controllare se i campi 'time' e 'ring' sono presenti
        field_names = [field.name for field in msg.fields]
        has_time = 'time' in field_names
        has_ring = 'ring' in field_names
        
        new_points = []
        for i, p in enumerate(points):
            x, y, z, intensity = p[:4]  # Supponiamo che i primi 4 campi siano x, y, z, intensity
            
            # Aggiungere il tempo relativo (ipotizziamo che il tempo massimo sia 0.1s)
            point_time = (i / len(points)) * 0.1 if not has_time else p[field_names.index('time')]
            
            # Aggiungere l'anello (ipotizziamo 16 anelli per un Velodyne VLP-16)
            ring_number = (i % 64) if not has_ring else p[field_names.index('ring')]
            
            new_points.append((x, y, z, intensity, point_time, ring_number))
        
        # Definizione dei nuovi campi
        new_fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('time', 16, PointField.FLOAT32, 1),
            PointField('ring', 20, PointField.UINT16, 1)
        ]
        
        # Creare un nuovo messaggio PointCloud2
        new_msg = pcl2.create_cloud(msg.header, new_fields, new_points)
        self.pub.publish(new_msg)
        
if __name__ == '__main__':
    try:
        pcm = PointCloudModifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
