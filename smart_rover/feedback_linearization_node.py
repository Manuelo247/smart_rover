import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import numpy as np

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
        self.subscription = self.create_subscription(
            Point,
            '/goal_position',
            self.goal_callback,
            10
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Temporizador para actualización periódica
        self.timer = self.create_timer(self.dt, self.update)

        # Objetivo actual
        self.qd = np.array([self.q[0], self.q[1]])
        self.qd_dot = np.array([-3.0, -3.0])  # velocidad deseada constante por ahora

    def goal_callback(self, msg: Point):
        self.qd = np.array([msg.x, msg.y])
        self.get_logger().info(f"Nueva posición deseada: x={msg.x:.2f}, y={msg.y:.2f}")

    def update(self):
        x, y, theta = self.q
        e = np.array([x, y]) - self.qd

        c, s = np.cos(theta), np.sin(theta)
        D = np.array([
            [c, -self.h * s],
            [s,  self.h * c]
        ])

        # Resolver para v y w
        U = np.linalg.solve(D, -self.K @ e + self.qd_dot)
        v, w = U

        # Publicar en cmd_vel
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        # Simular avance (sin odometría real, circuito abierto)
        x_dot = v * np.cos(theta) - self.h * np.sin(theta) * w
        y_dot = v * np.sin(theta) + self.h * np.cos(theta) * w
        theta_dot = w
        self.q += np.array([x_dot, y_dot, theta_dot]) * self.dt

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackLinearizationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
