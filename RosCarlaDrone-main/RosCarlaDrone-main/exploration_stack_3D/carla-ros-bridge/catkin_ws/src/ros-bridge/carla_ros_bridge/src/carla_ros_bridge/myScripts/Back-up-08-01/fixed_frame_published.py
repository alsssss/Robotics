#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish_fixed_frame():
    rospy.init_node('fixed_frame_publisher')
    
    # Nome del frame
    fixed_frame_id = "map"
    
    # Frequenza di pubblicazione
    rate = rospy.Rate(1)  
    
    # Creazione del broadcaster TF
    br = tf.TransformBroadcaster()
    
    rospy.loginfo("Publishing fixed frame: '%s' at (0, 0, 0)", fixed_frame_id)
    
    while not rospy.is_shutdown():
        # Pubblica il frame fisso in (0, 0, 0)
        br.sendTransform(
            (0.0, 0.0, 0.0),  # Traslazione
            (0.0, 0.0, 0.0, 1.0),  # Quaternione per nessuna rotazione
            rospy.Time.now(),
            fixed_frame_id,  # Nome del frame pubblicato
            "world"          # Nome del frame padre
        )
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_fixed_frame()
    except rospy.ROSInterruptException:
        pass

