#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    corrected_msg = PoseStamped()
    corrected_msg.header.stamp = rospy.Time.now()
    corrected_msg.header.frame_id = "map"  # Imposta il frame_id corretto
    corrected_msg.pose = msg.pose  # Copia la posa originale
    
    pub.publish(corrected_msg)


if __name__ == '__main__':
    rospy.init_node('fix_setpoint_frame_node')
    
    pub = rospy.Publisher('/goal_destination', PoseStamped, queue_size=10)
    sub = rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, callback)
    
    rospy.loginfo("Nodo fix_setpoint_frame_node avviato...")
    rospy.spin()