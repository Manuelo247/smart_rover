import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import threading
import sys
import termios
import tty
from collections import deque

class SimpleSafetyNode(Node):
    def __init__(self):
        super().__init__('simple_safety_node')

        # Parámetros
        self.fov_degrees = 70
        self.min_safe = 0.2
        self.max_safe = 0.3
        self.saturate = True
        self.emergency_stop = False
        self.emergency_timer = None

        self.latest_dist = float('inf')
        self.dist_history = deque(maxlen=5)  # Para el filtro de promedio móvil

        # Subscripciones y publicaciones
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/control', self.control_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Hilo para entrada de teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) * 180.0 / np.pi
        angles = (angles + 360) % 360

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

        min_dist = np.min(valid) if valid.size > 0 else float('inf')

        # Aplicar filtro de promedio móvil
        self.dist_history.append(min_dist)
        self.latest_dist = np.mean(self.dist_history) if self.dist_history else float('inf')

    def control_callback(self, msg: Twist):
        factor = self.calculate_factor(self.latest_dist)

        original_vx = msg.linear.x
        adjusted_vx = original_vx * factor if original_vx > 0 else original_vx

        if self.emergency_stop:
            # No publica desde aquí, el timer se encarga
            return

        # Saturación opcional
        if self.saturate:
            adjusted_vx = max(min(adjusted_vx, 0.3), -0.3)
            msg.linear.y = max(min(msg.linear.y, 0.3), -0.3)
            msg.linear.z = max(min(msg.linear.z, 0.3), -0.3)
            msg.angular.x = max(min(msg.angular.x, 1.5), -1.5)
            msg.angular.y = max(min(msg.angular.y, 1.5), -1.5)
            msg.angular.z = max(min(msg.angular.z, 1.5), -1.5)

        # Publica velocidad ajustada
        new_cmd = Twist()
        new_cmd.linear.x = adjusted_vx
        new_cmd.linear.y = msg.linear.y
        new_cmd.linear.z = msg.linear.z
        new_cmd.angular = msg.angular

        status = " [PARO]" if self.emergency_stop else ""
        print(f"\r[LiDAR]:{self.latest_dist:6.3f} m  [Vel_lineal]{original_vx:5.2f} --> {adjusted_vx:5.2f} | [Vel_angular]:{msg.angular.z:5.2f}  Factor: {factor:4.2f}{status}", flush=True)

        self.cmd_pub.publish(new_cmd)

    def calculate_factor(self, distance):
        if distance >= self.max_safe:
            return 1.0
        elif distance <= self.min_safe:
            return 0.0
        else:
            return (distance - self.min_safe) / (self.max_safe - self.min_safe)

    def publish_zero_velocity(self):
        zero_cmd = Twist()
        self.cmd_pub.publish(zero_cmd)

    def keyboard_listener(self):
        def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                return sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

        print("[TECLADO] Espacio = paro emergencia | q = salir")
        while rclpy.ok():
            char = getch()
            if char == ' ':
                self.emergency_stop = not self.emergency_stop
                estado = "ACTIVADO" if self.emergency_stop else "DESACTIVADO"
                print(f"\r[EMERGENCIA] Paro de emergencia {estado}")

                if self.emergency_stop:
                    self.emergency_timer = self.create_timer(0.5, self.publish_zero_velocity)
                else:
                    if self.emergency_timer:
                        self.emergency_timer.cancel()
                        self.emergency_timer = None

            elif char == 'q' or char =="Q":
                print("[SALIDA] Cerrando nodo por teclado...")
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSafetyNode()
    rclpy.spin(node)

    # Limpieza
    if node.emergency_timer:
        node.emergency_timer.cancel()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
