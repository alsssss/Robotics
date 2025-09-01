#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

def map_callback(new_map):
    global previous_map, map_pub

    if previous_map is None:
        previous_map = new_map
        return

    new_data = np.array(new_map.data, dtype=np.int8)
    prev_data = np.array(previous_map.data, dtype=np.int8)
    
    changed_indices = np.where(new_data != prev_data)[0]

    if len(changed_indices) > 0:
        # Trova i limiti del cambiamento per creare un sottoinsieme della mappa
        width = new_map.info.width
        height = new_map.info.height
        
        min_x = min(changed_indices % width)
        max_x = max(changed_indices % width)
        min_y = min(changed_indices // width)
        max_y = max(changed_indices // width)
        
        update = OccupancyGridUpdate()
        update.header = new_map.header
        update.x = min_x
        update.y = min_y
        update.width = max_x - min_x + 1
        update.height = max_y - min_y + 1

        # Estrarre solo la porzione aggiornata
        update_data = []
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                index = y * width + x
                update_data.append(new_data[index])
        
        update.data = update_data

        map_pub.publish(update)

    previous_map = new_map

if __name__ == "__main__":
    rospy.init_node("map_updates_publisher")

    previous_map = None

    rospy.Subscriber("/cmap", OccupancyGrid, map_callback)
    map_pub = rospy.Publisher("/map_updates", OccupancyGridUpdate, queue_size=10)

    rospy.spin()
