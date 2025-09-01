#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
import carla
import numpy as np

class Lidar:
    def __init__(self, client, world):
        self.world = world
        self.spectator = world.get_spectator()

        # Publisher per il messaggio PointCloud2
        self.lidar_pub = rospy.Publisher('/sensor/lidar_data', PointCloud2, queue_size=10)

        # Configura il LIDAR
        self.setup_lidar()

    def setup_lidar(self):
        blueprint_library = self.world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

        # Configurazione del LIDAR
        lidar_bp.set_attribute('range', '100.0')                # Portata massima
        lidar_bp.set_attribute('channels', '64')                # Numero di canali
        lidar_bp.set_attribute('points_per_second', '100000')  # Densità di punti
        lidar_bp.set_attribute('rotation_frequency', '20')      # Frequenza di rotazione
        lidar_bp.set_attribute('upper_fov', '0')               # Campo visivo superiore
        lidar_bp.set_attribute('lower_fov', '-90')              # Campo visivo inferiore

        # Posizionamento del LIDAR
        lidar_transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(0, 0, 0))
        self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.spectator)

        # Callback per il LIDAR
        self.lidar.listen(self.lidar_callback)
        rospy.loginfo("LIDAR configurato e attaccato allo spectator.")

    def lidar_callback(self, data):
        """Callback per pubblicare i dati LIDAR come messaggio PointCloud2."""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        # Conversione dei dati del LIDAR
        lidar_points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        num_points = lidar_points.shape[0]

        # Creazione del messaggio PointCloud2
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1  # Indica che i dati sono organizzati in una riga
        cloud_msg.width = num_points
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # Ogni punto è composto da 4 campi FLOAT32 (4*4 bytes)
        cloud_msg.row_step = cloud_msg.point_step * num_points
        cloud_msg.data = np.asarray(lidar_points, dtype=np.float32).tobytes()
        cloud_msg.is_dense = True  # Assume che i dati non contengano NaN o inf

        # Pubblicazione del messaggio
        self.lidar_pub.publish(cloud_msg)

    def destroy(self):
        """Distrugge il LIDAR quando il nodo viene terminato."""
        if self.lidar is not None:
            self.lidar.destroy()


def main():
    rospy.init_node('lidar')
    rospy.loginfo("Nodo 'lidar' avviato.")

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    lidar = Lidar(client, world)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'lidar' interrotto.")
        lidar.destroy()


if __name__ == '__main__':
    main()
