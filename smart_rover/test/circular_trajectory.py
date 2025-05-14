import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circular_trajectory_publisher')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.radius = 1.0  # Radio del círculo en metros
        self.center_x = 0.0
        self.center_y = 0.0
        self.center_initialized = False  
        self.period = 40 # Tiempo para completar una vuelta (segundos)
        self.timer_period = 5.0  # Publicar cada 2 segundos
        self.t = 0.0
        self.timer = self.create_timer(self.timer_period, self.publish_next_point)

    def publish_next_point(self):
        # No publicar hasta que el centro esté inicializado
        if not self.center_initialized:
            self.get_logger().info("Esperando primera lectura de odometria...")
            return

        # Calcular el ángulo actual
        omega = 2 * math.pi / self.period  # Velocidad angular
        angle = omega * self.t

        # Calcular la siguiente posición en la circunferencia
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)

        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0

        self.publisher.publish(point)
        self.get_logger().info(f"Publicando punto circular: x={x:.2f}, y={y:.2f}")

        self.t += self.timer_period
        
    def odom_callback(self, msg: Odometry):
        """Callback para inicializar el centro solo una vez."""
        if not self.center_initialized:
            self.center_x = msg.pose.pose.position.x
            self.center_y = msg.pose.pose.position.y
            self.center_initialized = True
            self.get_logger().info(f"Centro inicializado en: x={self.center_x:.2f}, y={self.center_y:.2f}")

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

def main(args=None):
    rclpy.init(args=args)
    # Lanzar ambos nodos: el controlador y el generador de trayectoria
    trajectory_node = CircularTrajectoryPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(trajectory_node)
    try:
        executor.spin()
    finally:
        trajectory_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()