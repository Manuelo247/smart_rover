import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker  # Agrega esta importación
import numpy as np
import math

class FeedbackLinearizationController(Node):
    def __init__(self):
        super().__init__('feedback_linearization_controller')

        # Parámetros del robot
        self.h = 0.5
        k1, k2 = (0.4, 0.6)
        self.K = np.array([[k1, 0], [0, k2]])

        # Estado inicial
        self.q = np.array([0.0, 0.0, 0.0])  # x, y, theta

        # Tiempo de integración
        self.dt = 0.01

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

        # Publicador para visualizar el objetivo en RViz
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Temporizador para actualización periódica
        self.timer = self.create_timer(self.dt, self.update)

        # Objetivo actual
        self.qd = np.array([self.q[0], self.q[1]])
        self.qd_prev = np.copy(self.qd)  # Guarda la posición deseada anterior
        self.qd_dot = np.array([0.0, 0.0])  # velocidad deseada inicial

        self.last_log_time = self.get_clock().now()
        self.log_interval = rclpy.duration.Duration(seconds=0.5)

    def goal_callback(self, msg: Point):
        """Callback para actualizar la posición deseada."""
        self.qd_prev = np.copy(self.qd)  # Guarda la posición deseada anterior antes de actualizar
        self.qd = np.array([msg.x, msg.y])
        self.get_logger().info(f"Nueva posición deseada: x={msg.x:.2f}, y={msg.y:.2f}")
        self.publish_goal_marker(msg.x, msg.y)

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "odom"  # Asegúrate que coincida con el frame de RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

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

        self.get_logger().debug(f"ODOM: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

        # Actualizar el estado del robot
        self.q = np.array([x, y, theta])

    def update(self):
        """Actualizar el control y publicar comandos de velocidad."""
        # Calcular la derivada numérica de la trayectoria deseada
        self.qd_dot = (self.qd - self.qd_prev) / self.dt
        self.qd_prev = np.copy(self.qd)

        x, y, theta = self.q
        e = np.array([x, y]) - self.qd

        # Publicar la distancia al objetivo solo cada 0.5 segundos
        now = self.get_clock().now()
        if (now - self.last_log_time) > self.log_interval:
            self.get_logger().info(f"Distancia al objetivo: {np.linalg.norm(e):.2f}")
            self.last_log_time = now

        c, s = np.cos(theta), np.sin(theta)
        D = np.array([
            [c, -self.h * s],
            [s,  self.h * c]
        ])

        try:
            # Resolver para v y w
            U = np.linalg.solve(D, -self.K @ e + self.qd_dot)
        except np.linalg.LinAlgError:
            self.get_logger().warn(f"Singularidad con theta={theta:.2f}, e={e}")
            return

        v, w = U

        # Frenado progresivo si está cerca del objetivo
        #distance = np.linalg.norm(e)
        if np.linalg.norm(e) < 0.1:
            v = 0.0
            w = 0.0
            self.get_logger().info("Objetivo alcanzado.")
        #if distance < 0.2:
        #    v *= distance / 0.2

        # Limitar las velocidades
        #max_linear_speed = 0.2  # o incluso 0.1 para empezar
        #max_angular_speed = 0.5
        #v = np.clip(v, -max_linear_speed, max_linear_speed)
        #w = np.clip(w, -max_angular_speed, max_angular_speed)

        self.get_logger().debug(f"CMD: v={v:.2f}, w={w:.2f}")

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