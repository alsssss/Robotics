#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
import time
from tf.transformations import euler_from_quaternion

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
        self.current_position = None
        self.target_position = None
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/goal_destination', PoseStamped, self.target_position_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)

        max_speed = 0.03
        max_angular_speed = 0.1 * 1/2 * math.pi
        # OK P = 1.5
        self.pid_x = PIDController(kp=0.75, ki=0.0000, kd=0.0, max_output=max_speed)
        self.pid_y = PIDController(kp=0.75, ki=0.0000, kd=0.0, max_output=max_speed)
        self.pid_z = PIDController(kp=0.75, ki=0.0000, kd=0.0, max_output=max_speed)
        # TESTING
        self.pid_roll = PIDController(kp=5.0, ki=0.00001, kd=0.0, max_output=max_angular_speed)
        self.pid_pitch = PIDController(kp=5.0, ki=0.00001, kd=0.0, max_output=max_angular_speed)
        self.pid_yaw = PIDController(kp=5.0, ki=0.00001, kd=0.0, max_output=max_angular_speed)

    def target_position_callback(self, msg: PoseStamped):
        self.target_position = msg

    def current_position_callback(self, msg: PoseStamped):
        self.current_position = msg

    def compute_velocity_and_orientation(self):
        if not self.target_position or not self.current_position:
            return None

        # Calcolo dell'errore di posizione
        error_x = self.target_position.pose.position.x - self.current_position.pose.position.x
        error_y = self.target_position.pose.position.y - self.current_position.pose.position.y
        error_z = self.target_position.pose.position.z - self.current_position.pose.position.z
        '''
        # Calcolo dell'errore di orientazione con trasf. quat -> euler
        target_quaternion = self.target_position.pose.orientation
        current_quaternion = self.current_position.pose.orientation

        target_euler = euler_from_quaternion([target_quaternion.x, target_quaternion.y, target_quaternion.z, target_quaternion.w])
        current_euler = euler_from_quaternion([current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w])

        error_roll = target_euler[0] - current_euler[0]
        error_pitch = target_euler[1] - current_euler[1]
        error_yaw = target_euler[2] - current_euler[2]

        '''

        error_roll = self.target_position.pose.orientation.x - self.current_position.pose.orientation.x
        error_pitch = self.target_position.pose.orientation.y - self.current_position.pose.orientation.y
        error_yaw = self.target_position.pose.orientation.z - self.current_position.pose.orientation.z


        #rospy.loginfo(f"Errore di posizione: x={error_x:.2f}, y={error_y:.2f}, z={error_z:.2f}")
        #rospy.loginfo(f"Errore di orientazione: roll={error_roll:.2f}, pitch={error_pitch:.2f}, yaw={error_yaw:.2f}")

        # Velocità lineari
        velocity_x = self.pid_x.compute(error_x)
        velocity_y = self.pid_y.compute(error_y)
        velocity_z = self.pid_z.compute(error_z)

        # Velocità angolari
        #angular_x = self.pid_roll.compute(error_roll)
        #angular_y = self.pid_pitch.compute(error_pitch)
        #angular_z = self.pid_yaw.compute(error_yaw)

        angular_x = 0
        angular_y = 0
        angular_z = 0

        # Costruzione del comando Twist
        velocity = Twist()
        velocity.linear.x = velocity_x
        velocity.linear.y = velocity_y
        velocity.linear.z = velocity_z
        velocity.angular.x = angular_x
        velocity.angular.y = angular_y
        velocity.angular.z = angular_z

        return velocity
    

    def convert_velocity_ros_to_carla(self, velocity_ros: Twist) -> Twist:
        """
        Converte una velocità dal sistema di riferimento di ROS a quello di CARLA.
        """
        velocity_carla = Twist()

        # Conversione velocità lineari
        velocity_carla.linear.x = velocity_ros.linear.x
        velocity_carla.linear.y = -velocity_ros.linear.y  # Asse Y invertito
        velocity_carla.linear.z = velocity_ros.linear.z

        # Conversione velocità angolari
        velocity_carla.angular.x = velocity_ros.angular.x
        velocity_carla.angular.y = -velocity_ros.angular.y  # Asse Y invertito
        velocity_carla.angular.z = -velocity_ros.angular.z  # Asse Z invertito

        return velocity_carla
    


    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            velocity = self.compute_velocity_and_orientation()
            if velocity:  # Verifica che velocity non sia None
                velocity_carla = self.convert_velocity_ros_to_carla(velocity)
                self.velocity_pub.publish(velocity_carla)
            rate.sleep()


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
