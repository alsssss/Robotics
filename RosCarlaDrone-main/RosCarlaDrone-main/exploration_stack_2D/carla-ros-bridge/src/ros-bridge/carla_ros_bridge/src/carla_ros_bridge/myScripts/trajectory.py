#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
from math import sqrt

## Uso OOP perchè le variabili globali non si aggiornano (?)
## OK ## TODO: Tarare PID --> ki va aumentato perchè ci mette troppo a chiudere le distanze vicino il target (regime)
## TODO: Capire come funziona rviz per creare traiettorie
## TODO: Iniziare a capire come fare un Dockerfile

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        rospy.loginfo("Nodo 'trajectory_publisher' avviato.")
        self.pub = rospy.Publisher('/target_position', Point, queue_size=10)
        rospy.Subscriber('/current_position', Twist, self.position_callback)
        
        self.current_position = None  
        self.threshold = 5.0  
        self.rate = rospy.Rate(5)  
        
        # Square trajectory
        self.trajectory = [
            
            Point(0, 200, 5),
            Point(200, 200, 5),
            Point(0, 200, 5),
            Point(0, 0, 5)
            
            #Point(0, 0, 5),
            #Point(0, 200, 5)
        ]
        self.index = 0  


    def position_callback(self, msg: Twist):
        self.current_position = Point(msg.linear.x, msg.linear.y, msg.linear.z)

    def calculate_distance(self, point1, point2):
        return sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)

    # Praticamente il main
    def run(self):
        while not rospy.is_shutdown():
            if self.current_position:
                target = self.trajectory[self.index]
                distance = self.calculate_distance(self.current_position, target)
                #rospy.loginfo(f"Posizione corrente: ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}), \nTarget: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f}), \nDistanza: {distance:.2f}")
                
                if distance <= self.threshold:
                    self.index = (self.index + 1) % len(self.trajectory)
                    #rospy.loginfo(f"Nuovo target: {self.trajectory[self.index]}")
            else:
                rospy.loginfo("Attendere l'aggiornamento della posizione...")

            self.pub.publish(self.trajectory[self.index])
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = TrajectoryPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'trajectory_publisher' interrotto.")
        pass
