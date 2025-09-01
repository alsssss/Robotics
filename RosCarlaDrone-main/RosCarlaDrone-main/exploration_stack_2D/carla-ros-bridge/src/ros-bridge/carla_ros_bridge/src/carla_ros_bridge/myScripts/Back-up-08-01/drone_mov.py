#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import carla

class Drone:
    def __init__(self, client, world):
        self.world = world
        self.spectator = world.get_spectator()
        self.velocity = carla.Vector3D(0, 0, 0)
        self.angular_velocity = carla.Vector3D(0, 0, 0)
        rospy.Subscriber('/spectator_velocity_cmd', Twist, self.velocity_callback)
        self.position_pub = rospy.Publisher('/current_position', Twist, queue_size=10)

    def velocity_callback(self, msg: Twist):
        self.velocity.x = msg.linear.x
        self.velocity.y = msg.linear.y
        self.velocity.z = msg.linear.z
        self.angular_velocity.x = msg.angular.x
        self.angular_velocity.y = msg.angular.y
        self.angular_velocity.z = msg.angular.z

    def update_position(self, delta_time):
        current_transform = self.spectator.get_transform()
        location = current_transform.location
        rotation = current_transform.rotation

        # Aggiorna posizione
        location.x += self.velocity.x * delta_time
        location.y += self.velocity.y * delta_time
        location.z += self.velocity.z * delta_time

        # Aggiorna orientamento
        rotation.pitch += self.angular_velocity.x * delta_time
        rotation.yaw += self.angular_velocity.z * delta_time
        rotation.roll += self.angular_velocity.y * delta_time

        self.spectator.set_transform(carla.Transform(location, rotation))

        # Pubblica posizione e orientamento in un messaggio Twist
        current_state = Twist()
        current_state.linear.x = location.x
        current_state.linear.y = location.y
        current_state.linear.z = location.z
        current_state.angular.x = rotation.pitch
        current_state.angular.y = rotation.roll
        current_state.angular.z = rotation.yaw

        self.position_pub.publish(current_state)


def main():
    rospy.init_node('drone_mov')
    rospy.loginfo("Nodo 'drone_mov' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    ##
    settings = world.get_settings()
    settings.no_rendering_mode = True
    world.apply_settings(settings)
    ##


    drone = Drone(client, world)
    rate_Hz = 5
    rate = rospy.Rate(rate_Hz)

    try:
        while not rospy.is_shutdown():
            delta_time = float(1.0 / rate_Hz)
            drone.update_position(delta_time)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'drone_mov' interrotto.")


if __name__ == '__main__':
    main()
