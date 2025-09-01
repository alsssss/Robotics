#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped

def twist_callback(msg):
    """
    Callback per convertire il messaggio Twist in TwistStamped.
    """
    global stamped_pub
    
    # Creazione del messaggio TwistStamped
    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = rospy.Time.now()  # Timestamp corrente
    twist_stamped.header.frame_id = "map"          # Frame di riferimento
    twist_stamped.twist = msg                      # Copia del messaggio Twist
    
    # Pubblica il messaggio TwistStamped
    stamped_pub.publish(twist_stamped)

def twist_to_twist_stamped():
    """
    Nodo principale per la conversione di Twist in TwistStamped.
    """
    global stamped_pub
    
    rospy.init_node('twist_to_twist_stamped')
    
    # Publisher per /current_stamped_position
    stamped_pub = rospy.Publisher('/current_stamped_position', TwistStamped, queue_size=10)
    
    # Subscriber per /current_position
    rospy.Subscriber('/current_position', Twist, twist_callback)
    
    rospy.loginfo("Node 'twist_to_twist_stamped' started, listening on '/current_position'")
    
    # Loop infinito
    rospy.spin()

if __name__ == "__main__":
    try:
        twist_to_twist_stamped()
    except rospy.ROSInterruptException:
        pass

