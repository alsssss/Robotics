#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import carla
import tf
import math
from collections import deque

class IMU:
    def __init__(self, client, world):
        self.world = world
        self.spectator = world.get_spectator()

        self.imu_pub = rospy.Publisher('/sensor/imu_data', Imu, queue_size=10)

        n_samples=30

        # Storia dei dati per il filtro mediano
        self.angular_velocity_history = {'x': deque(maxlen=n_samples), 'y': deque(maxlen=n_samples), 'z': deque(maxlen=n_samples)}
        self.linear_acceleration_history = {'x': deque(maxlen=n_samples), 'y': deque(maxlen=n_samples), 'z': deque(maxlen=n_samples)}

        self.setup_imu()

    def setup_imu(self):
        blueprint_library = self.world.get_blueprint_library()
        imu_bp = blueprint_library.find('sensor.other.imu')

        # Posizionamento del sensore IMU
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.spectator)

        # Callback per il sensore IMU
        self.imu_sensor.listen(self.imu_callback)
        rospy.loginfo("Sensore IMU configurato e attaccato allo spectator.")

    def imu_callback(self, carla_imu_data):
        imu_msg = Imu()

        # Header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "map"  # Frame fisso per RViz

        # Orientation (quaternion)
        quaternion = self.carla_to_ros_quaternion(carla_imu_data.transform.rotation)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        # Angular velocity
        imu_msg.angular_velocity.x = self.validate_data(carla_imu_data.gyroscope.x, self.angular_velocity_history['x'], 20)
        imu_msg.angular_velocity.y = self.validate_data(carla_imu_data.gyroscope.y, self.angular_velocity_history['y'], 20)
        imu_msg.angular_velocity.z = self.validate_data(carla_imu_data.gyroscope.z, self.angular_velocity_history['z'], 20)

        # Linear acceleration
        imu_msg.linear_acceleration.x = self.validate_data(carla_imu_data.accelerometer.x, self.linear_acceleration_history['x'], 20)
        imu_msg.linear_acceleration.y = self.validate_data(carla_imu_data.accelerometer.y, self.linear_acceleration_history['y'], 20)
        imu_msg.linear_acceleration.z = self.validate_data(carla_imu_data.accelerometer.z, self.linear_acceleration_history['z'], 20)

        self.imu_pub.publish(imu_msg)

    @staticmethod
    def carla_to_ros_quaternion(carla_rotation):
        roll = math.radians(carla_rotation.roll)
        pitch = math.radians(carla_rotation.pitch)
        yaw = math.radians(carla_rotation.yaw)

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    @staticmethod
    def validate_data(value, history, threshold):
        history.append(value)
        if abs(value) > threshold:
            return sorted(history)[len(history) // 2]  # Mediana della finestra
        return value

    def destroy(self):
        if self.imu_sensor is not None:
            self.imu_sensor.destroy()

def main():
    rospy.init_node('imu')
    rospy.loginfo("Nodo 'imu' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    imu = IMU(client, world)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'imu' interrotto.")
        imu.destroy()

if __name__ == '__main__':
    main()
