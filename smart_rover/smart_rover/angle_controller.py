#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class YawController(Node):
    def __init__(self):
        super().__init__('yaw_controller')

        # --- Parámetros PID y límites ---
        self.declare_parameter('k_p', 0.5)             # reduje P respecto al ejemplo anterior
        self.declare_parameter('k_d', 0.1)             # ganancia derivativa
        self.declare_parameter('angle_tolerance', 0.01)
        self.declare_parameter('max_ang_speed', 1.0)   # [rad/s]
        self.declare_parameter('max_ang_accel', 0.5)   # [rad/s²]

        self.k_p           = self.get_parameter('k_p').value
        self.k_d           = self.get_parameter('k_d').value
        self.angle_tol     = self.get_parameter('angle_tolerance').value
        self.max_w         = self.get_parameter('max_ang_speed').value
        self.max_accel     = self.get_parameter('max_ang_accel').value

        # estado interno para D y ramp
        self.prev_err      = 0.0
        self.prev_w        = 0.0

        self.current_yaw   = 0.0
        self.goal_yaw      = 0.0
        self.dt            = 0.05  # 20 Hz

        # Subscripciones y publicador
        self.create_subscription(Float64, '/goal_yaw', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(self.dt, self.update)
        self.get_logger().info('YawController con PD y ramp iniciado.')

    def normalize_angle(self, a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def goal_callback(self, msg: Float64):
        self.goal_yaw = self.normalize_angle(msg.data)
        self.get_logger().info(f'Nuevo goal_yaw: {self.goal_yaw:.2f} rad')

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1-2*(q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def update(self):
        # error
        err = self.normalize_angle(self.goal_yaw - self.current_yaw)

        # derivada de error
        err_dot = (err - self.prev_err) / self.dt
        self.prev_err = err

        # PD sin límites aún
        w_unlim = self.k_p * err + self.k_d * err_dot

        # Limitar aceleración angular (ramp-up)
        delta = w_unlim - self.prev_w
        max_delta = self.max_accel * self.dt
        if abs(delta) > max_delta:
            w_ramped = self.prev_w + math.copysign(max_delta, delta)
        else:
            w_ramped = w_unlim

        # Limitar velocidad angular
        w = max(-self.max_w, min(self.max_w, w_ramped))
        self.prev_w = w

        # Si ya estamos dentro de la tolerancia, frenar
        if abs(err) < self.angle_tol:
            w = 0.0

        # Publicar (sin componente lineal)
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        self.get_logger().debug(f"err={err:.3f}, err_dot={err_dot:.3f}, w={w:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = YawController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
