#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('spectator_velocity_publisher')

    pub = rospy.Publisher('/spectator_velocity_cmd', Twist, queue_size=10)
    rospy.loginfo("Nodo 'spectator_velocity_publisher' avviato.")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 1.0  # Esempio di velocità in avanti
        cmd.linear.y = 0.0
        cmd.linear.z = 0.5  # Salire lentamente
        cmd.angular.z = 0.1  # Rotazione sul proprio asse
        pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo '/spectator_velocity_cmd' interrotto.")
        pass
