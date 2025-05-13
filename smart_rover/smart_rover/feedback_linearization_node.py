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

        # Ganancia y tolerancia (puedes cambiarlo vía ros2 param:set si lo deseas)
        self.declare_parameter('k_ang', 0.8)
        self.declare_parameter('angle_tolerance', 0.01)  # radianes

        self.k_ang      = self.get_parameter('k_ang').value
        self.angle_tol  = self.get_parameter('angle_tolerance').value

        self.current_yaw = 0.0
        self.goal_yaw    = 0.0

        # Suscripción al goal_yaw (en radianes)
        self.create_subscription(
            Float64,
            'goal_yaw',
            self.goal_callback,
            10
        )

        # Suscripción a la odometría
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer de control a 20 Hz
        self.create_timer(0.05, self.update)

        self.get_logger().info('YawController iniciado. Esperando /goal_yaw…')

    def normalize_angle(self, angle: float) -> float:
        """Reduce un ángulo a (-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_callback(self, msg: Float64):
        """Actualiza el objetivo de orientación."""
        self.goal_yaw = self.normalize_angle(msg.data)
        self.get_logger().info(f'Nuevo goal_yaw: {self.goal_yaw:.2f} rad')

    def odom_callback(self, msg: Odometry):
        """Extrae yaw de la odometría."""
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def update(self):
        """Calcula error yaw y publica solo angular.z."""
        err = self.normalize_angle(self.goal_yaw - self.current_yaw)

        twist = Twist()
        twist.linear.x = 0.0

        if abs(err) > self.angle_tol:
            twist.angular.z = float(self.k_ang * err)
        else:
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = YawController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()