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

    def add_lidar_sensor(self):
        """Aggiunge un sensore LiDAR al drone"""
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')

        lidar_bp.set_attribute('range', '200.0')                # Portata massima
        lidar_bp.set_attribute('channels', '64')                # Numero di canali
        lidar_bp.set_attribute('points_per_second', '1000000')  # Densità di punti
        lidar_bp.set_attribute('rotation_frequency', '20')      # Frequenza di rotazione
        lidar_bp.set_attribute('upper_fov', '0')                # Campo visivo superiore
        lidar_bp.set_attribute('lower_fov', '-90')              # Campo visivo inferiore

        # Posizione relativa del LiDAR rispetto al drone (in questo caso lo spettatore)
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))  # Posizione sopra il drone

        self.lidar_sensor = self.world.spawn_actor(lidar_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("LiDAR aggiunto con successo.")


    def add_imu_sensor(self):
        """Aggiunge un sensore IMU al drone"""
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')

        # Posizione relativa dell'IMU rispetto al drone (in questo caso lo spettatore)
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))  # Posizione centrale sul drone
    
        self.imu_sensor = self.world.spawn_actor(imu_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("IMU aggiunto con successo.")


    def add_rgb_camera(self):
        """Aggiunge una camera RGB collegata allo spectator"""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
            
        # Configura parametri della camera
        camera_bp.set_attribute('image_size_x', '640') 
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')

        # Posizionamento della camera rispetto allo spectator
        spawn_point = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(pitch=-15.0))

        # Crea e collega la camera
        self.rgb_camera = self.world.spawn_actor(camera_bp, spawn_point, attach_to=self.spectator)
        rospy.loginfo("Camera RGB aggiunta allo spectator.")




def main():
    rospy.init_node('drone_mov')
    rospy.loginfo("Nodo 'drone_mov' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    # Disabilita il rendering per migliorare le prestazioni
    #settings = world.get_settings()
    #settings.no_rendering_mode = True
    #settings.synchronous_mode = True
    #world.apply_settings(settings)

    drone = Drone(client, world)

    # Aggiungi i sensori al drone
    #drone.add_lidar_sensor()
    #drone.add_imu_sensor()
    #drone.add_rgb_camera()

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
