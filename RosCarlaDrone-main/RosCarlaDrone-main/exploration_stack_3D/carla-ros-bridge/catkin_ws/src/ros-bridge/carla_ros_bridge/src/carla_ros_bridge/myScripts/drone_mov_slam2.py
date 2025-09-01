#!/usr/bin/env python3

import rospy
import carla
import geometry_msgs.msg
import math
from geometry_msgs.msg import Twist

class Drone:
    def __init__(self, world):
        self.world = world
        self.spectator = world.get_spectator()
        
        self.velocity = carla.Vector3D(0, 0, 0)
        self.angular_velocity = carla.Vector3D(0, 0, 0)
        # For map01 = (200,190,3)
        # For map02 = (190,240,3)
        
        self.initial_position = carla.Vector3D(190, 240, 0.5)
        self.spectator.set_transform(carla.Transform(self.initial_position))
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.position_pub = rospy.Publisher('/current_position', Twist, queue_size=10)

    def add_lidar_sensor(self):
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50.0')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('points_per_second', '2000000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('upper_fov', '10')
        lidar_bp.set_attribute('lower_fov', '-60')
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        self.lidar_sensor = self.world.spawn_actor(lidar_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("LiDAR aggiunto con successo.")

    def add_imu_sensor(self):
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        self.imu_sensor = self.world.spawn_actor(imu_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("IMU aggiunto con successo.")

    def add_rgb_camera(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640') 
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(pitch=-15.0))
        self.rgb_camera = self.world.spawn_actor(camera_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("Camera RGB aggiunta allo spectator.")

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
        #rotation.pitch += self.angular_velocity.x * delta_time
        #rotation.yaw += self.angular_velocity.z * delta_time
        #rotation.roll += self.angular_velocity.y * delta_time

        rotation.yaw = 0
        rotation.pitch = 0
        rotation.roll = 0

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
    rospy.init_node('drone')
    rospy.loginfo("Nodo 'drone' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    drone = Drone(world)
    drone.add_lidar_sensor()
    drone.add_imu_sensor()
    #drone.add_rgb_camera()

    rate = rospy.Rate(10)  

    try:
        while not rospy.is_shutdown():
            delta_time = float(1.0 / 10)
            drone.update_position(delta_time)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'drone' interrotto.")

if __name__ == '__main__':
    main()