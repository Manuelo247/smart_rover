#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from visualization_msgs.msg import Marker

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

        # Subscripción a la posición deseada (ahora PoseStamped)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/local_goal_pose',
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

        self.last_log_time = self.get_clock().now()
        self.log_interval = rclpy.duration.Duration(seconds=0.5)

    def goal_callback(self, msg: PoseStamped):
        """Callback para actualizar la posición deseada."""
        self.qd_prev = np.copy(self.qd)  # Guarda la posición deseada anterior antes de actualizar
        self.qd = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"Nueva posición deseada: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.publish_goal_marker(msg.pose.position.x, msg.pose.position.y)

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "odom"  # Asegúrate que coincida con el frame de RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

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