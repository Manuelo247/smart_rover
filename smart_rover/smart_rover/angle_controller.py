#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class AngleController(Node):
    def __init__(self):
        super().__init__('angle_controller')

        # Parámetros
        self.declare_parameter('k', 1.5)
        self.declare_parameter('angle_tolerance', 0.05)  # radianes

        self.k           = self.get_parameter('k').value
        self.angle_tol   = self.get_parameter('angle_tolerance').value
        self.goal_yaw    = 0.0  # Inicialmente en 0

        self.current_yaw = 0.0
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.dt          = 0.05  # 20 Hz

        # Suscripciones y publicaciones
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.local_goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose/local', self.local_goal_pose_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel/angle', 10)

        self.timer = self.create_timer(self.dt, self.update)

    def local_goal_pose_callback(self, msg):
        print("[DEBUG] Callback /local_goal_pose recibido")
        # Calcula el ángulo objetivo a partir de la posición GLOBAL del mensaje PoseStamped
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        self.goal_yaw = math.atan2(dy, dx)
        print(f"[GOAL] Nueva pose global recibida: x={goal_x:.2f}, y={goal_y:.2f} -> Yaw objetivo: {math.degrees(self.goal_yaw):.2f}°")

    def odom_callback(self, msg):
        # Guarda la posición actual y extrae el ángulo yaw desde el quaternion de la odometría
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        # print(f"[ODOM] Posición actual: x={self.current_x:.2f}, y={self.current_y:.2f} | Yaw actual: {math.degrees(self.current_yaw):.2f}°")

    def normalize_angle(self, angle):
        # Normaliza el ángulo a [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update(self):
        # Error de ángulo
        err = self.normalize_angle(self.current_yaw - self.goal_yaw)

        # Ley de control: U = -k * e
        w = -self.k * err

        # Si el error es pequeño, detenerse
        if abs(err) < self.angle_tol:
            w = 0.0

        # Prints para depuración en grados
        print(f"[CONTROL] Yaw actual: {math.degrees(self.current_yaw):.2f}° | Yaw objetivo: {math.degrees(self.goal_yaw):.2f}° | Error: {math.degrees(err):.2f}° | w: {w:.3f} rad/s")

        # Publicar comando de velocidad angular
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AngleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
