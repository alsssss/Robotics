#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def tf_to_odometry():
    rospy.init_node('tf_to_odometry')
    
    # Inizializzazione del listener TF2
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # Publisher per il messaggio di odometria
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    # Loop di pubblicazione
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Ottieni la trasformazione da 'odom' a 'base_link'
            transform = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            
            # Crea un messaggio Odometry
            odom = Odometry()
            
            # Estrai la posizione e l'orientamento dalla trasformazione
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Posizione
            odom.pose.pose.position.x = transform.transform.translation.x
            odom.pose.pose.position.y = transform.transform.translation.y
            odom.pose.pose.position.z = transform.transform.translation.z
            
            # Orientamento
            odom.pose.pose.orientation = transform.transform.rotation
            
            # Velocità (opzionale, potrebbe essere calcolata separatamente)
            odom.twist.twist.linear.x = 0.0  # Velocità in x
            odom.twist.twist.linear.y = 0.0  # Velocità in y
            odom.twist.twist.linear.z = 0.0  # Velocità in z
            
            # Pubblica il messaggio di odometria
            odom_pub.publish(odom)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Trasformazione non disponibile!")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_odometry()
    except rospy.ROSInterruptException:
        pass
