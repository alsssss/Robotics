#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd, max_output=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time == 0:
            delta_time = 1e-6

        p = error * self.kp
        self.integral += error * delta_time
        i = self.integral * self.ki
        derivative = (error - self.prev_error) / delta_time
        d = derivative * self.kd

        self.prev_error = error
        self.last_time = current_time

        output = p + i + d
        return max(-self.max_output, min(self.max_output, output))

class VelocityController:
    def __init__(self):
        self.current_position = Twist()
        self.target_position = None
        self.velocity_pub = rospy.Publisher('/spectator_velocity_cmd', Twist, queue_size=10)
        rospy.Subscriber('/target_position', Point, self.target_position_callback)
        rospy.Subscriber('/current_position', Twist, self.current_position_callback)

        max_speed = 1.0
        max_angle_rate = 4 * math.pi

        self.pid_x      = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)
        self.pid_y      = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)
        self.pid_z      = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)


    def target_position_callback(self, msg: Point):
        self.target_position = msg

    def current_position_callback(self, msg: Twist):
        self.current_position = msg
        self.publish_velocity()

    def compute_velocity_and_orientation(self):
        if not self.target_position:
            return None

        error = Point(
            self.target_position.x - self.current_position.linear.x,
            self.target_position.y - self.current_position.linear.y,
            self.target_position.z - self.current_position.linear.z
        )

        rospy.loginfo(f"Errore di posizione: x={error.x:.2f}, y={error.y:.2f}, z={error.z:.2f}")

        velocity_x = self.pid_x.compute(error.x)
        velocity_y = self.pid_y.compute(error.y)
        velocity_z = self.pid_z.compute(error.z)



        velocity = Twist()
        velocity.linear.x = velocity_x
        velocity.linear.y = velocity_y
        velocity.linear.z = velocity_z


        return velocity

    def publish_velocity(self):
        velocity = self.compute_velocity_and_orientation()
        if velocity:
            self.velocity_pub.publish(velocity)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('pid_velocity_controller')
    rospy.loginfo("Nodo 'pid_velocity_controller' avviato.")
    controller = VelocityController()
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo 'pid_velocity_controller' interrotto.")
