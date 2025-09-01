#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
import time

class PIDController:
    def __init__(self, kp, ki, kd, max_output=float('inf')):
        """
        Inizializza il controllore PID con parametri:
        - kp: guadagno proporzionale
        - ki: guadagno integrale
        - kd: guadagno derivativo
        - max_output: limite massimo per l'output (le componenti asse xyz della velocità)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = Point(0, 0, 0)
        self.integral = Point(0, 0, 0)
        self.last_time = time.time()

    def compute(self, error):

        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time == 0:  # Evita divisioni per zero
            delta_time = 1e-6

        # Componente proporzionale
        p = Point(error.x * self.kp, error.y * self.kp, error.z * self.kp)

        # Componente integrale
        self.integral.x += error.x * delta_time
        self.integral.y += error.y * delta_time
        self.integral.z += error.z * delta_time
        i = Point(self.integral.x * self.ki, self.integral.y * self.ki, self.integral.z * self.ki)

        # Componente derivativa
        derivative = Point(
            (error.x - self.prev_error.x) / delta_time,
            (error.y - self.prev_error.y) / delta_time,
            (error.z - self.prev_error.z) / delta_time
        )
        d = Point(derivative.x * self.kd, derivative.y * self.kd, derivative.z * self.kd)

        # Aggiorna gli stati precedenti
        self.prev_error = error
        self.last_time = current_time

        # Somma delle componenti
        output = Point(p.x + i.x + d.x, p.y + i.y + d.y, p.z + i.z + d.z)

        # Applica la saturazione
        output.x = max(-self.max_output, min(self.max_output, output.x))
        output.y = max(-self.max_output, min(self.max_output, output.y))
        output.z = max(-self.max_output, min(self.max_output, output.z))

        return output

class VelocityController:
    def __init__(self):
        
        self.current_position = Point(0, 0, 2)  # Posizione iniziale
        self.target_position = None  # Posizione target inizialmente non disponibile
        self.velocity_pub = rospy.Publisher('/spectator_velocity_cmd', Twist, queue_size=10)
        rospy.Subscriber('/target_position', Point, self.target_position_callback)
        rospy.Subscriber('/current_position', Point, self.current_position_callback)

        # Limite massimo della velocità 
        max_speed = 5.0

        # Inizializza i PID con saturazione per ciascun asse
        self.pid_x = PIDController(kp=1.0, ki=0.1, kd=0.05, max_output=max_speed)
        self.pid_y = PIDController(kp=1.0, ki=0.1, kd=0.05, max_output=max_speed)
        self.pid_z = PIDController(kp=1.0, ki=0.1, kd=0.05, max_output=max_speed)

    def target_position_callback(self, msg: Point):
        self.target_position = msg

    def current_position_callback(self, msg: Point):
        self.current_position = msg

    def compute_velocity(self):

        if not self.target_position:
            return None

        # Calcolo dell'errore di posizione
        error = Point(
            self.target_position.x - self.current_position.x,
            self.target_position.y - self.current_position.y,
            self.target_position.z - self.current_position.z
        )

        # Calcolo della velocità usando i PID
        velocity_x = self.pid_x.compute(error).x
        velocity_y = self.pid_y.compute(error).y
        velocity_z = self.pid_z.compute(error).z

        # Creazione del messaggio Twist per pubblicare la velocità
        velocity = Twist()
        velocity.linear.x = velocity_x
        velocity.linear.y = velocity_y
        velocity.linear.z = velocity_z

        return velocity

    def run(self):
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            velocity = self.compute_velocity()
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
