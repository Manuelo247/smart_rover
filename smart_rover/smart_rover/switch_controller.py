import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class SupervisorController(Node):
    def __init__(self):
        super().__init__('supervisor_controller')
        print("DECLARANDO NODO")
        # Parámetros
        self.angle_tol = 0.05  # Tolerancia angular (rad)
        self.state = "TURN"    # TURN o FOLLOW
        self.dist_tol = 0.01  # Tolerancia de distancia al objetivo
        self.angle_tol_turn = 0.06  # Cambia a GIRO si el error es mayor a esto
        self.angle_tol_follow = 0.05 # Cambia a SEGUIMIENTO si el error es menor a esto

        # Estado actual
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Suscripciones y publicaciones
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose/local', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/control', 10)

        # Suscribirse a los comandos de ambos controladores
        self.cmd_angle = Twist()
        self.cmd_linearization = Twist()
        self.angle_sub = self.create_subscription(Twist, '/cmd_vel/angle', self.cmd_angle_callback, 10)
        self.linearization_sub = self.create_subscription(Twist, '/cmd_vel/linearization', self.cmd_linearization_callback, 10)

        self.timer = self.create_timer(0.05, self.update)
        self._last_info_time = self.get_clock().now()
        self.last_state_change_time = self.get_clock().now()  # Para el delay de cambio de estado
        self._pending_state = None
        self._pending_state_time = None
        print("NODO DECLARADO")

    def info_throttled(self, msg):
        """Publica info solo si ha pasado al menos 0.5s desde el último info."""
        now = self.get_clock().now()
        if (now - self._last_info_time).nanoseconds * 1e-9 > 0.5:
            self.get_logger().debug(msg)
            self._last_info_time = now
    
    def cmd_angle_callback(self, msg):
        self.cmd_angle = msg
        #print("Angle callback")
        
    def cmd_linearization_callback(self, msg):
        # Solo toma la componente angular, ignora la velocidad lineal
        self.cmd_linearization = Twist()
        self.cmd_linearization.linear = msg.linear
        #print("FL callback")
        
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        self.goal_yaw = math.atan2(dy, dx)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def update(self):
        err_angle = self.normalize_angle(self.goal_yaw - self.current_yaw)
        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        print("UPDATE")
        if dist < self.dist_tol:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().debug("Objetivo alcanzado. Robot detenido.")
            self._pending_state = None
            self._pending_state_time = None
            return

        now = self.get_clock().now()
        delay = 0.1  # segundos

        # Lógica de cambio de estado con delay al detectar condición de cambio
        next_state = None
        if self.state == "TURN" and abs(err_angle) < self.angle_tol_follow:
            next_state = "FOLLOW"
        elif self.state == "FOLLOW" and abs(err_angle) > self.angle_tol_turn:
            next_state = "TURN"

        if next_state is not None:
            if self._pending_state != next_state:
                self._pending_state = next_state
                self._pending_state_time = now
            elif (now - self._pending_state_time).nanoseconds * 1e-9 > delay:
                self.state = self._pending_state
                self.last_state_change_time = now
                if self.state == "FOLLOW":
                    self.get_logger().info("Orientación alcanzada. Cambiando a modo SEGUIMIENTO.")
                else:
                    self.get_logger().info("Desalineado. Cambiando a modo GIRO.")
                self._pending_state = None
                self._pending_state_time = None
        else:
            self._pending_state = None
            self._pending_state_time = None

        if self.state == "TURN":
            self.get_logger().info(f"modo GIRO activo. err_angle: {err_angle}")
            self.cmd_pub.publish(self.cmd_angle)
        elif self.state == "FOLLOW":
            self.get_logger().info(f"modo SEGUIMIENTO activo. err_angle: {err_angle}")
            self.cmd_pub.publish(self.cmd_linearization)

def main(args=None):
    print("INICIO")
    rclpy.init(args=args)
    node = SupervisorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()