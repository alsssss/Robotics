#!/usr/bin/env python3

import carla
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


class RgbCameraNode:
    def __init__(self):
        rospy.init_node('rgb_camera_node', anonymous=True)
        rospy.Subscriber('/current_position', Twist, None)  # Non serve a niente tranne che visivamente nel rqt_graph

        # Configurazione della connessione a CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        # Ottenere lo spectator
        self.spectator = self.world.get_spectator()

        # Configurazione della telecamera RGB
        self.blueprint_library = self.world.get_blueprint_library()
        self.camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '800')
        self.camera_bp.set_attribute('image_size_y', '600')
        self.camera_bp.set_attribute('fov', '90')

        # Attacco della telecamera allo spectator
        transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        self.camera = self.world.spawn_actor(self.camera_bp, transform, attach_to=self.spectator)

        # Publisher per il topic ROS
        self.image_publisher = rospy.Publisher('/rgb_camera/image_raw', Image, queue_size=10)

        # Inizializzazione di CvBridge per convertire le immagini
        self.bridge = CvBridge()

        # Callback per i dati della telecamera
        self.camera.listen(self.camera_callback)

        rospy.loginfo("RGB Camera configurata e attaccata allo spectator.")

    def camera_callback(self, carla_image):
        """Callback per i dati della telecamera CARLA."""
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = array.reshape((carla_image.height, carla_image.width, 4))
        rgb_image = array[:, :, :3]  # Rimuovere il canale alfa

        # Convertire l'immagine in un messaggio ROS
        ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")

        # Pubblicazione del messaggio
        self.image_publisher.publish(ros_image)

    def run(self):
        rospy.spin()

    def destroy(self):
        """Distruzione dell'oggetto e rilascio delle risorse."""
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
        

def main():
    try:
        camera = RgbCameraNode()
        camera.run()
    except rospy.ROSInterruptException:
        camera.destroy()
        rospy.loginfo("Nodo RGB Camera terminato.")
        pass


if __name__ == '__main__':
    main()
