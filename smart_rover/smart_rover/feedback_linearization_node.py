import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
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

        # Subscripción a la posición deseada (ahora PoseStamped)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose/local',
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

        # Subscripción a la velocidad deseada
        self.cmd_desired_sub = self.create_subscription(
            Twist,
            '/cmd_vel/desired',
            self.cmd_desired_callback,
            10
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/linearization', 10)

        # Publicador para visualizar el objetivo en RViz
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Temporizador para actualización periódica
        self.timer = self.create_timer(self.dt, self.update)

        # Objetivo actual
        self.qd = np.array([self.q[0], self.q[1]])
        self.qd_prev = np.copy(self.qd)  # Guarda la posición deseada anterior
        self.qd_dot = np.array([0.0, 0.0])  # velocidad deseada inicial

        self.last_log_time = self.get_clock().now()
        self.log_interval = rclpy.duration.Duration(seconds=1)
        self.last_goal_reached_log_time = self.get_clock().now()  # Nueva variable
        self._last_info_time = self.get_clock().now()

    def info_throttled(self, msg):
        """Publica info solo si ha pasado al menos 0.5s desde el último info."""
        now = self.get_clock().now()
        if (now - self._last_info_time).nanoseconds * 1e-9 > 0.5:
            self.get_logger().info(msg)
            self._last_info_time = now

    def goal_callback(self, msg: PoseStamped):
        """Callback para actualizar la posición deseada."""
        self.qd_prev = np.copy(self.qd)
        self.qd = np.array([msg.pose.position.x, msg.pose.position.y])
        self.info_throttled(f"Nueva posición deseada: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.publish_goal_marker(msg.pose.position.x, msg.pose.position.y)

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # Asegúrate que coincida con el frame de RViz
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

    def cmd_desired_callback(self, msg: Twist):
        """Actualiza la velocidad deseada (qd_dot) usando /cmd_vel/desired."""
        self.qd_dot = np.array([msg.linear.x, msg.angular.z])

    def update(self):
        """Actualizar el control y publicar comandos de velocidad."""
        # Calcular la derivada numérica de la trayectoria deseada
        self.qd_dot = (self.qd - self.qd_prev) / self.dt
        self.qd_prev = np.copy(self.qd)

        x, y, theta = self.q
        e = np.array([x, y]) - self.qd

        # Publicar la distancia al objetivo solo cada 0.5 segundos
        self.info_throttled(f"Distancia al objetivo: {np.linalg.norm(e):.2f}")

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
        # if np.linalg.norm(e) < 0.1:
        #     v = 0.0
        #     w = 0.0
        #     self.info_throttled("Objetivo alcanzado.")

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