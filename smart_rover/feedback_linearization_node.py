import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class FeedbackLinearizationController(Node):
    def __init__(self):
        super().__init__('feedback_linearization_controller')

        # Parámetros del robot
        self.h = 0.5
        k = 3
        self.K = np.diag([k, k])

        # Estado inicial
        self.q = np.array([0.0, 0.0, 0.0])  # x, y, theta

        # Tiempo de integración
        self.dt = 0.1

        # Subscripción a la posición deseada
        self.goal_sub = self.create_subscription(
            Point,
            '/goal_position',
            self.goal_callback,
            10
        )

        # Subscripción a la odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Temporizador para actualización periódica
        self.timer = self.create_timer(self.dt, self.update)

        # Objetivo actual
        self.qd = np.array([self.q[0], self.q[1]])
        self.qd_dot = np.array([0, -0])  # velocidad deseada constante

    def goal_callback(self, msg: Point):
        """Callback para actualizar la posición deseada."""
        self.qd = np.array([msg.x, msg.y])
        self.get_logger().info(f"Nueva posición deseada: x={msg.x:.2f}, y={msg.y:.2f}")

    def odom_callback(self, msg: Odometry):
        """Callback para actualizar el estado del robot usando la odometría."""
        # Posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convertir cuaternión a ángulo theta
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Actualizar el estado del robot
        self.q = np.array([x, y, theta])

    def update(self):
        """Actualizar el control y publicar comandos de velocidad."""
        x, y, theta = self.q
        e = np.array([x, y]) - self.qd

        c, s = np.cos(theta), np.sin(theta)
        D = np.array([
            [c, -self.h * s],
            [s,  self.h * c]
        ])

        try:
            # Resolver para v y w
            U = np.linalg.solve(D, -self.K @ e + self.qd_dot)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular matrix en el cálculo del control. Saltando esta actualización.")
            return

        v, w = U

        # Limitar las velocidades
        max_linear_speed = 0.5  # Velocidad lineal máxima (m/s)
        max_angular_speed = 1.0  # Velocidad angular máxima (rad/s)
        v = np.clip(v, -max_linear_speed, max_linear_speed)
        w = np.clip(w, -max_angular_speed, max_angular_speed)

        # Publicar en cmd_vel
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackLinearizationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
