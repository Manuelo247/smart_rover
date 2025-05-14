#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class KalmanFilter1D:
    def __init__(self, dt, process_var, meas_var):
        # Estado [posición; velocidad]
        self.x = np.array([[0.0], [0.0]])
        # Covarianza inicial
        self.P = np.eye(2)
        # Modelo de transición de estado
        self.A = np.array([[1.0, dt],
                           [0.0, 1.0]])
        # Modelo de observación (medimos posición)
        self.C = np.array([[1.0, 0.0]])
        # Ruido de proceso
        self.Q = np.array([[process_var, 0.0],
                           [0.0, process_var]])
        # Ruido de medición
        self.R = np.array([[meas_var]])

    def predict(self):
        # Predicción de estado y covarianza
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # z: medición escalar de posición
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        y = np.array([[z]]) - (self.C @ self.x)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.C) @ self.P

class KalmanOdomNode(Node):
    def __init__(self):
        super().__init__('kalman_odom_node')
        # Parámetros: frecuencia, varianzas de proceso y medición
        rate         = self.declare_parameter('rate', 50).value
        dt           = 1.0 / rate
        process_var  = self.declare_parameter('process_variance',    0.001).value
        meas_var     = self.declare_parameter('measurement_variance', 0.01).value

        # Instanciar filtro
        self.kf = KalmanFilter1D(dt, process_var, meas_var)

        # Suscriptor a odom “cruda”
        self.sub_raw  = self.create_subscription(
            Odometry, 'odom_raw', self.odom_raw_cb, 10)
        # Publicador de odom filtrada
        self.pub_filt = self.create_publisher(
            Odometry, 'odom_filtered', 10)

    def odom_raw_cb(self, msg: Odometry):
        # 1) Medición de posición en X
        z = msg.pose.pose.position.x
        # 2) Ciclo Kalman
        self.kf.predict()
        self.kf.update(z)
        est_pos, est_vel = float(self.kf.x[0]), float(self.kf.x[1])
        # 3) Armar mensaje de salida
        out = Odometry()
        out.header         = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x      = est_pos
        out.twist.twist.linear.x      = est_vel
        # (puedes copiar orientación y covarianzas si lo deseas)
        self.pub_filt.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
