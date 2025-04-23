import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circular_trajectory_publisher')
        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.radius = 1.0  # Radio del círculo en metros
        self.center_x = 0.0
        self.center_y = 0.0
        self.period = 20.0  # Tiempo para completar una vuelta (segundos)
        self.timer_period = 2.0  # Publicar cada 2 segundos
        self.t = 0.0
        self.timer = self.create_timer(self.timer_period, self.publish_next_point)

    def publish_next_point(self):
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