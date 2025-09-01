#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

def tf_callback():
    try:
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        pose_stamped = PoseStamped()
        pose_stamped.header = trans.header
        pose_stamped.pose.position.x = trans.transform.translation.x
        pose_stamped.pose.position.y = trans.transform.translation.y
        pose_stamped.pose.position.z = trans.transform.translation.z
        pose_stamped.pose.orientation = trans.transform.rotation
        pub.publish(pose_stamped)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Nessuna trasformata /tf ricevuta")

if __name__ == '__main__':
    rospy.init_node('tf_to_posestamped')
    pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        tf_callback()
        rate.sleep()
