#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import time
'''
## NODO UTILITY(per fare rostopic echo /current_position senza aprire un altro terminale): 
Fuori dall'anello di controllo, serve solo per printare su terminale la posizione attuale del drone a una frequenza "leggibile"
'''


class PositionLogger:
    def __init__(self):
        
        self.last_log_time = 0  # Timestamp dell'ultimo log
        self.log_frequency = 1  # Frequenza di log in Hz (1 Hz = 1 log al secondo)
        self.log_interval = 1 / self.log_frequency  # Intervallo di tempo minimo tra i log

        rospy.init_node('drone_position_logger', anonymous=True)
        rospy.loginfo("Nodo 'drone_position_logger' avviato. Frequenza di log: " + str(self.log_frequency) + " Hz")
        self.position_publisher = rospy.Publisher('/logged_position', Point, queue_size=10)

        rospy.Subscriber('/current_position', Point, self.position_callback)

        rospy.spin()

    def position_callback(self, msg: Point):
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            rospy.loginfo(f"Posizione attuale del drone: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
            self.position_publisher.publish(msg)
            self.last_log_time = current_time

if __name__ == '__main__':
    try:
        PositionLogger()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'drone_position_logger' interrotto.")
        pass
