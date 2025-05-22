import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class SupervisorController(Node):
    def __init__(self):
        super().__init__('supervisor_controller')

        # Parámetros
        self.angle_tol = 0.05  # Tolerancia angular (rad)
        self.k_angle = 1.5     # Ganancia para el giro
        self.state = "TURN"    # TURN o FOLLOW
        self.dist_tol = 0.01  # Tolerancia de distancia al objetivo

        # Estado actual
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Suscripciones y publicaciones
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose/local', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscribirse a los comandos de ambos controladores
        self.cmd_angle = Twist()
        self.cmd_linearization = Twist()
        self.angle_sub = self.create_subscription(Twist, '/cmd_vel/angle', self.cmd_angle_callback, 10)
        self.linearization_sub = self.create_subscription(Twist, '/cmd_vel/linearization', self.cmd_linearization_callback, 10)

        self.timer = self.create_timer(0.05, self.update)

    def cmd_angle_callback(self, msg):
        self.cmd_angle = msg

    def cmd_linearization_callback(self, msg):
        self.cmd_linearization = msg

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        self.goal_yaw = math.atan2(dy, dx)
        err_angle = self.normalize_angle(self.goal_yaw - self.current_yaw)
        # Cambia a modo GIRO solo si el error angular es mayor que la tolerancia
        if abs(err_angle) > self.angle_tol:
            self.state = "TURN"
            self.get_logger().info("Nuevo objetivo recibido. Cambiando a modo GIRO.")
        else:
            self.state = "FOLLOW"
            self.get_logger().info("Nuevo objetivo recibido. Ya alineado, modo SEGUIMIENTO.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update(self):
        err_angle = self.normalize_angle(self.goal_yaw - self.current_yaw)
        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

        # Si está cerca del objetivo, detenerse
        if dist < self.dist_tol:
            stop = Twist()
            self.cmd_pub.publish(stop)
            # self.get_logger().info("Objetivo alcanzado. Robot detenido.")
            return

        if self.state == "TURN":
            # Solo girar hasta estar alineado
            if abs(err_angle) > self.angle_tol:
                self.cmd_pub.publish(self.cmd_angle)
            else:
                self.get_logger().info("Orientación alcanzada. Cambiando a modo SEGUIMIENTO.")
                self.state = "FOLLOW"
        elif self.state == "FOLLOW":
            self.cmd_pub.publish(self.cmd_linearization)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()