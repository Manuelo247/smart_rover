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
        self.min_safe = 0.19
        self.max_safe = 0.28
        self.saturate = True
        self.emergency_stop = False
        self.debug = False
        self.emergency_timer = None

        self.latest_front_dist = float('inf')
        self.latest_back_dist = float('inf')
        self.front_history = deque(maxlen=5)
        self.back_history = deque(maxlen=5)

        # Subscripciones y publicaciones
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, 'control', self.control_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Hilo para entrada de teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 15.0  # Reemplazar inf por 15.0
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) * 180.0 / np.pi
        angles = (angles + 360) % 360

        def get_sector_min(center_angle):
            lower = (center_angle - self.fov_degrees / 2.0) % 360
            upper = (center_angle + self.fov_degrees / 2.0) % 360
            if lower < upper:
                indices = np.where((angles >= lower) & (angles <= upper))[0]
            else:
                indices = np.where((angles >= lower) | (angles <= upper))[0]
            valid = ranges[indices]
            valid = valid[(valid > 0.0) & np.isfinite(valid)]
            return np.min(valid) if valid.size > 0 else 0.15  # Reemplazo consistente

        # Obtener distancias mínima delantera y trasera con filtro
        front_min = get_sector_min(0.0)
        back_min = get_sector_min(180.0)

        self.front_history.append(front_min)
        self.back_history.append(back_min)

        self.latest_front_dist = np.mean(self.front_history) if self.front_history else float('inf')
        self.latest_back_dist = np.mean(self.back_history) if self.back_history else float('inf')

    def control_callback(self, msg: Twist):
        front_factor = self.calculate_factor(self.latest_front_dist)
        back_factor = self.calculate_factor(self.latest_back_dist)

        original_vx = msg.linear.x
        if original_vx > 0:
            adjusted_vx = original_vx * front_factor
        elif original_vx < 0:
            adjusted_vx = original_vx * back_factor
        else:
            adjusted_vx = 0.0

        if self.emergency_stop:
            # No publica desde aquí, el timer se encarga
            return

        # Saturación opcional
        if self.saturate:
            adjusted_vx = max(min(adjusted_vx, 0.25), -0.25)
            msg.linear.y = max(min(msg.linear.y, 0.25), -0.25)
            msg.linear.z = max(min(msg.linear.z, 0.25), -0.25)
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
        if self.debug:
            print(f"\r[LiDAR]: F: {self.latest_front_dist:5.2f} m | B: {self.latest_back_dist:5.2f} m  [Velocidad] R: {original_vx:5.2f} m/s | E: {adjusted_vx:5.2f} m/s | [Angular]: {msg.angular.z:4.2f} | [Factor]: F: {front_factor:4.2f} | B: {back_factor:4.2f} {status}")

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

            elif char == 'q':
                print("[SALIDA] Cerrando nodo por teclado...")
                rclpy.shutdown()
                break
            elif char == 'd':
                if self.debug:
                    self.debug = False
                else:
                    self.debug = True
                print("[DEBUG]:",self.debug)

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
