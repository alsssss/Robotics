#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import carla

class Spectator:
    def __init__(self, client, world):
        self.spectator = world.get_spectator()
        self.velocity = carla.Vector3D(0, 0, 0)
        rospy.Subscriber('/spectator_velocity_cmd', Twist, self.velocity_callback)
        self.position_pub = rospy.Publisher('/current_position', Point, queue_size=10)

    def velocity_callback(self, msg: Twist):
        self.velocity.x = msg.linear.x
        self.velocity.y = msg.linear.y
        self.velocity.z = msg.linear.z

    def update_position(self, delta_time):
        # Ottieni la posizione corrente
        current_transform = self.spectator.get_transform()
        location = current_transform.location

        # Aggiorna la posizione in base alla velocità
        location.x += self.velocity.x * delta_time
        location.y += self.velocity.y * delta_time
        location.z += self.velocity.z * delta_time

        # Aggiorna lo spectator
        self.spectator.set_transform(carla.Transform(location, current_transform.rotation))

        # Pubblica la posizione corrente
        current_position = Point(location.x, location.y, location.z)
        self.position_pub.publish(current_position)

def main():
    rospy.init_node('spectator')
    rospy.loginfo("Nodo 'spectator' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    drone = Spectator(client, world)
    rate_Hz = 10
    rate = rospy.Rate(rate_Hz)  # 10 Hz

    while not rospy.is_shutdown():
        delta_time = float(1.0/rate_Hz) # 0.1s
        drone.update_position(delta_time)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'spectator' interrotto.")
        pass
