import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Configuración del escaneo
        self.angle_min = -math.pi  # 180 grados hacia la izquierda
        self.angle_max = math.pi   # 180 grados hacia la derecha
        self.angle_increment = math.radians(1)  # 1 grado por muestra
        self.range_min = 0.2
        self.range_max = 10.0

        self.get_logger().info("Simulador de LiDAR inicializado.")

    def timer_callback(self):
        scan = LaserScan()
        now = self.get_clock().now().to_msg()
        scan.header.stamp = now
        scan.header.frame_id = 'lidar_link'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0  # Se puede calcular si necesitas precisión de tiempo
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        scan.ranges = []

        # Generar datos simulados: un círculo vacío con algunos obstáculos
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment

            # Obstáculo simulado entre 2 y 3 metros en ciertos sectores
            if -0.5 < angle < 0.5:
                distance = random.uniform(2.0, 3.0)
            elif math.fabs(angle - math.pi / 2) < 0.1:
                distance = random.uniform(1.0, 2.0)
            else:
                distance = self.range_max  # libre

            scan.ranges.append(distance)

        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
