#!/usr/bin/env python3
"""
Nodo ROS2: Kalman 1D para posición y velocidad X

– Para priorizar un resultado más estable y sin picos:
    • Varianza de proceso Q = 1e-4 (muy baja, confía en el modelo)
    • Varianza de medición R = 0.1  (más alta, suaviza mediciones ruidosas)

– Comandos para ver datos en terminal:
    ros2 topic echo /odom_raw
    ros2 topic echo /odom_filtered

– Para visualizar en RViz2:
    1) rviz2
    2) Add → Odometry → Topic: /odom_raw     (color: rojo)
    3) Add → Odometry → Topic: /odom_filtered (color: verde)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class KalmanFilter1D:
    def __init__(self, dt, process_var, meas_var):
        # Estado [posición; velocidad]
        self.x = np.zeros((2,1))
        # Covarianza inicial
        self.P = np.eye(2)
        # Modelo de transición (2×2)
        self.A = np.array([[1.0, dt],
                           [0.0, 1.0]])
        # Observamos solo posición
        self.C = np.array([[1.0, 0.0]])
        # Ruido de proceso (suaviza cambios súbitos)
        self.Q = np.array([[process_var, 0.0],
                           [0.0,       process_var]])
        # Ruido de medición (suaviza la salida)
        self.R = np.array([[meas_var]])

    def predict(self):
        # Predice estado y covarianza
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # Corrección con la medición z (posición)
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        y = np.array([[z]]) - (self.C @ self.x)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.C) @ self.P

class KalmanOdomNode(Node):
    def __init__(self):
        super().__init__('kalman_odom_node')
        # Parámetros
        rate     = self.declare_parameter('rate', 50).value
        dt       = 1.0 / rate
        q_var    = self.declare_parameter('process_variance',    1e-4).value
        r_var    = self.declare_parameter('measurement_variance', 0.1).value

        # Filtro de Kalman
        self.kf = KalmanFilter1D(dt, q_var, r_var)

        # Suscriptor a odometría cruda
        self.sub_raw = self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            10
        )
        # Publicador de odometría filtrada
        self.pub_filt = self.create_publisher(
            Odometry,
            'odom_filtered',
            10
        )

    def odom_cb(self, msg: Odometry):
        # 1) Extraer posición medida en X
        z = msg.pose.pose.position.x

        # 2) Paso de Kalman
        self.kf.predict()
        self.kf.update(z)

        # 3) Leer estado estimado
        est_pos = float(self.kf.x[0])
        est_vel = float(self.kf.x[1])

        # 4) Construir mensaje de salida
        out = Odometry()
        out.header         = msg.header
        out.child_frame_id = msg.child_frame_id

        # Asignar posición y velocidad filtradas
        out.pose.pose.position.x   = est_pos
        out.twist.twist.linear.x   = est_vel

        # (Opcional) publicar covarianzas en pose & twist
        cov_pose = [0.0]*36
        cov_pose[0] = self.kf.P[0,0]  # var(pos)
        out.pose.covariance = cov_pose

        cov_twist = [0.0]*36
        cov_twist[0] = self.kf.P[1,1] # var(vel)
        out.twist.covariance = cov_twist

        # 5) Publicar odometría filtrada
        self.pub_filt.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
