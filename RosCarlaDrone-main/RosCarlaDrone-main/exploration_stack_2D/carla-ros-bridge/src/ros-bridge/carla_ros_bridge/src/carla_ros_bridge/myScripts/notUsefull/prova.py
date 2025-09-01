#!/usr/bin/env python3

import rospy
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def world_info_to_occupancy(msg):
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "map"

    # Configura i parametri della griglia (dimensioni, risoluzione, ecc.)
    occupancy_grid.info.resolution = 1.0  # Risoluzione in metri
    occupancy_grid.info.width = 100  # Larghezza della griglia
    occupancy_grid.info.height = 100  # Altezza della griglia
    occupancy_grid.info.origin.position.x = -50.0  # Offset della mappa
    occupancy_grid.info.origin.position.y = -50.0

    # Popola i dati della griglia (esempio semplificato: tutto libero)
    occupancy_grid.data = [-1] * (occupancy_grid.info.width * occupancy_grid.info.height)

    # Pubblica la mappa
    occupancy_pub.publish(occupancy_grid)

rospy.init_node('world_info_to_occupancy')
occupancy_pub = rospy.Publisher('/carla/map_occupancy', OccupancyGrid, queue_size=10)
rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_to_occupancy)
rospy.spin()