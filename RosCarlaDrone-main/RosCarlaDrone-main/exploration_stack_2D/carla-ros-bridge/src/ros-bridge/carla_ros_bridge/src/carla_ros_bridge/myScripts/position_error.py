#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

class PositionErrorNode:
    def __init__(self):
        rospy.init_node('position_error_node', anonymous=True)
        
        # Sottoscrizione ai topic
        rospy.Subscriber('/current_position', Twist, self.real_position_callback)
        rospy.Subscriber('/tf', TFMessage, self.estimated_position_callback)
        
        self.real_position = None
        self.estimated_position = None

    def real_position_callback(self, msg):
        self.real_position = (msg.linear.x, msg.linear.y, msg.linear.z)
        self.compute_error()
    
    def estimated_position_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "24":  
                self.estimated_position = (
                    transform.transform.translation.x,
                    -transform.transform.translation.y,
                    transform.transform.translation.z
                )
                self.compute_error()
    
    def compute_error(self):
        if self.real_position is not None and self.estimated_position is not None:
            error = [
                round(self.real_position[0] - self.estimated_position[0], 3),
                round(self.real_position[1] - self.estimated_position[1], 3),
                round(self.real_position[2] - self.estimated_position[2], 3)
            ]
            real_pos = [round(coord, 3) for coord in self.real_position]
            estimated_pos = [round(coord, 3) for coord in self.estimated_position]
            print(f"Real Position: {real_pos}, Estimated Position: {estimated_pos}, Position Error: {error}")
    
if __name__ == '__main__':
    try:
        node = PositionErrorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass