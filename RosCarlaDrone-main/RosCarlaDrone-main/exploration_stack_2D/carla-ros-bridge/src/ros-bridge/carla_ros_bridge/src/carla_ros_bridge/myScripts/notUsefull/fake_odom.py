#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

def publish_odom():
    rospy.init_node('fake_odom_publisher')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_listener = tf.TransformListener()

    def tf_callback(msg):
        try:
            latest_time = tf_listener.getLatestCommonTime('/map', '/24')
            (trans, rot) = tf_listener.lookupTransform('/map', '/24', latest_time)
            (linear_vel, angular_vel) = tf_listener.lookupTwist('/map', '/24', latest_time, rospy.Duration(0.1))

            # Crea il messaggio di odometria
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = '24'

            # Posizione
            odom_msg.pose.pose.position.x = trans[0]
            odom_msg.pose.pose.position.y = trans[1]
            odom_msg.pose.pose.position.z = trans[2]
            odom_msg.pose.pose.orientation.x = rot[0]
            odom_msg.pose.pose.orientation.y = rot[1]
            odom_msg.pose.pose.orientation.z = rot[2]
            odom_msg.pose.pose.orientation.w = rot[3]

            # Velocità
            odom_msg.twist.twist.linear.x = linear_vel[0]
            odom_msg.twist.twist.linear.y = linear_vel[1]
            odom_msg.twist.twist.linear.z = linear_vel[2]
            odom_msg.twist.twist.angular.x = angular_vel[0]
            odom_msg.twist.twist.angular.y = angular_vel[1]
            odom_msg.twist.twist.angular.z = angular_vel[2]

            # Pubblica il messaggio
            odom_pub.publish(odom_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed, waiting for transform...")

    # Sottoscrizione al topic /tf
    rospy.Subscriber('/tf', TFMessage, tf_callback)

    # Mantieni il nodo attivo
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        pass
