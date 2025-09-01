#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd, max_output=float('inf')):
        """
        Inizializza il controllore PID con parametri:
        - kp: guadagno proporzionale
        - ki: guadagno integrale
        - kd: guadagno derivativo
        - max_output: limite massimo per l'output
        """
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
        if delta_time == 0:  # Evita divisioni per zero
            delta_time = 1e-6

        # Componente proporzionale
        p = error * self.kp

        # Componente integrale
        self.integral += error * delta_time
        i = self.integral * self.ki

        # Componente derivativa
        derivative = (error - self.prev_error) / delta_time
        d = derivative * self.kd

        # Aggiorna gli stati precedenti
        self.prev_error = error
        self.last_time = current_time

        # Somma delle componenti
        output = p + i + d

        # Applica la saturazione
        output = max(-self.max_output, min(self.max_output, output))

        return output

class VelocityController:
    def __init__(self):
        self.current_position = Twist()
        self.target_position = None
        self.velocity_pub = rospy.Publisher('/spectator_velocity_cmd', Twist, queue_size=10)
        rospy.Subscriber('/target_position', Point, self.target_position_callback)
        rospy.Subscriber('/current_position', Twist, self.current_position_callback)

        max_speed = 5.0
        max_angle_rate = 4*math.pi

        self.pid_x = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)
        self.pid_y = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)
        self.pid_z = PIDController(kp=4.0, ki=0.075, kd=0.01, max_output=max_speed)


        self.pid_yaw   = PIDController(kp=100.0, ki=10, kd=0.1, max_output=max_angle_rate)
        self.pid_pitch = PIDController(kp=100.0, ki=10, kd=0.1, max_output=max_angle_rate)
        self.pid_roll  = PIDController(kp=100.0, ki=10, kd=0.1, max_output=max_angle_rate)

    def target_position_callback(self, msg: Point):
        self.target_position = msg

    def current_position_callback(self, msg: Twist):
        self.current_position = msg

    def compute_velocity_and_orientation(self):
        if not self.target_position:
            return None

        # Calcolo dell'errore di posizione
        error = Point(
            self.target_position.x - self.current_position.linear.x,
            self.target_position.y - self.current_position.linear.y,
            self.target_position.z - self.current_position.linear.z
        )

        # Log degli errori di posizione
        rospy.loginfo(f"Errore di posizione: x={error.x:.2f}, y={error.y:.2f}, z={error.z:.2f}")

        # Velocità lineari
        velocity_x = self.pid_x.compute(error.x)
        velocity_y = self.pid_y.compute(error.y)
        velocity_z = self.pid_z.compute(error.z)

        # Calcolo degli angoli desiderati
        desired_yaw = math.atan2(error.y, error.x)
        desired_pitch = -math.atan2(error.z, math.sqrt(error.x**2 + error.y**2))
        desired_roll = 0.0  # Roll target specifico 

        # Angoli attuali
        current_yaw = self.current_position.angular.z * math.pi / 180
        current_pitch = self.current_position.angular.y * math.pi / 180
        current_roll = self.current_position.angular.x * math.pi / 180

        # Calcolo degli errori angolari
        yaw_error = desired_yaw - current_yaw
        pitch_error = desired_pitch - current_pitch
        roll_error = desired_roll - current_roll

        # Normalizzazione degli errori per evitare discontinuità
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        pitch_error = math.atan2(math.sin(pitch_error), math.cos(pitch_error))
        roll_error = math.atan2(math.sin(roll_error), math.cos(roll_error))

        # Log degli errori di orientamento
        rospy.loginfo(f"Errore di orientamento: yaw={yaw_error:.2f}, pitch={pitch_error:.2f}, roll={roll_error:.2f}")

        # Velocità angolari
        angular_yaw = self.pid_yaw.compute(yaw_error)
        angular_pitch = self.pid_pitch.compute(pitch_error)
        angular_roll = self.pid_roll.compute(roll_error)

        # Costruzione del comando Twist
        velocity = Twist()
        velocity.linear.x = velocity_x
        velocity.linear.y = velocity_y
        velocity.linear.z = velocity_z
        velocity.angular.z = angular_yaw
        velocity.angular.y = angular_pitch
        velocity.angular.x = angular_roll

        return velocity


    def run(self):
        rate = rospy.Rate(5)  # Frequenza di aggiornamento (5 Hz)
        while not rospy.is_shutdown():
            velocity = self.compute_velocity_and_orientation()
            if velocity:
                self.velocity_pub.publish(velocity)
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
        pass
