import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SimpleSafetyNode(Node):
    def __init__(self):
        super().__init__('simple_safety_node')

        self.fov_degrees = 80
        self.min_safe = 0.16   # en metros
        self.max_safe = 0.25   # en metros

        self.latest_dist = float('inf')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/control', self.control_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        print("Nodo iniciado")
    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) * 180.0 / np.pi
        angles = (angles + 360) % 360  # normaliza a [0, 360)

        # Solo interesa el frente: 0° ± 40° = [320°, 40°]
        center_angle = 0.0
        fov = self.fov_degrees
        lower = (center_angle - fov / 2.0) % 360
        upper = (center_angle + fov / 2.0) % 360

        if lower < upper:
            indices = np.where((angles >= lower) & (angles <= upper))[0]
        else:
            indices = np.where((angles >= lower) | (angles <= upper))[0]

        valid = ranges[indices]
        valid = valid[(valid > 0.0) & np.isfinite(valid)]

        self.latest_dist = np.min(valid) if valid.size > 0 else float('inf')

    def control_callback(self, msg: Twist):
        factor = self.calculate_factor(self.latest_dist)

        original_vx = msg.linear.x
        # Aplica factor solo si va hacia adelante
        adjusted_vx = original_vx * factor if original_vx > 0 else original_vx

        new_cmd = Twist()
        new_cmd.linear.x = adjusted_vx
        new_cmd.linear.y = msg.linear.y
        new_cmd.linear.z = msg.linear.z
        new_cmd.angular = msg.angular

        print(f"[LiDAR]: {self.latest_dist:6.3f} m    [Velocidad] Recibida: {original_vx:5.2f} m/s | Enviada: {adjusted_vx:5.2f} m/s | Factor: {factor:4.2f}")

        self.cmd_pub.publish(new_cmd)

    def calculate_factor(self, distance):
        if distance >= self.max_safe:
            return 1.0
        elif distance <= self.min_safe:
            return 0.0
        else:
            return (distance - self.min_safe) / (self.max_safe - self.min_safe)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
