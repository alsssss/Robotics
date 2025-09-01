#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        # Pubblicazione del path
        self.path_pub = rospy.Publisher('/drone_path', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "map"  # Imposta il frame di riferimento a "map"

        # Inizializza il subscriber per il topic /tracked_pose
        rospy.Subscriber("/tracked_pose", PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        # Aggiungi la nuova posa al path
        self.path.poses.append(msg)
        self.path.header.stamp = rospy.Time.now()
        # Pubblica il path aggiornato
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('path_publisher')
    PathPublisher()
    rospy.spin()
