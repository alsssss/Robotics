#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import carla

class SpectatorController:
    def __init__(self, client, world):
        self.spectator = world.get_spectator()
        self.velocity = carla.Vector3D(0, 0, 0)
        rospy.Subscriber('/spectator_velocity_cmd', Twist, self.velocity_callback)

    def velocity_callback(self, msg: Twist):
        # Convert ROS Twist to CARLA Vector3D
        self.velocity.x = msg.linear.x
        self.velocity.y = msg.linear.y
        self.velocity.z = msg.linear.z

    def update_position(self, delta_time):
        # Get current spectator transform
        current_transform = self.spectator.get_transform()
        # Update location based on velocity
        new_location = current_transform.location
        new_location.x += self.velocity.x * delta_time
        new_location.y += self.velocity.y * delta_time
        new_location.z += self.velocity.z * delta_time
        # Apply the new transform
        self.spectator.set_transform(carla.Transform(new_location, current_transform.rotation))

def main():
    rospy.init_node('spectator_controller')
    rospy.loginfo("Nodo 'spectator_controller' avviato.")
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    controller = SpectatorController(client, world)
    rate_Hz = 10
    rate = rospy.Rate(rate_Hz)  # 10 Hz

    while not rospy.is_shutdown():
        delta_time =  float(1.0/rate_Hz)  # Assuming 10 Hz
        controller.update_position(delta_time)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'spectator_controller' interrotto.")
        pass
