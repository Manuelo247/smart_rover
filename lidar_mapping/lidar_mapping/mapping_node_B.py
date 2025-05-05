#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import math

from lidar_mapping.occupancy_grid_B import OccupancyGrid

class GridMappingNode(Node):
    def __init__(self):
        super().__init__('grid_mapping_node')
        # Parámetros del mapa
        width = 400       # celdas
        height = 400      # celdas
        resolution = 0.05 # m/celda (20 celdas por 1 m)
        origin = (-10.0, -10.0)  # esquina inferior izquierda en el mundo
        self.map = OccupancyGrid(width, height, resolution,
                                 origin_x=origin[0], origin_y=origin[1])
        # Suscriptores
        self.create_subscription(LaserScan, 'scan',
                                 self.lidar_callback, 10)
        self.create_subscription(Odometry, 'odom',
                                 self.odom_callback, 10)

        # Publicador del mapa como Float32MultiArray (opcional)
        self.map_pub = self.create_publisher(Float32MultiArray,
                                             'occupancy_map', 10)

        self.latest_scan = None
        self.latest_odom = None

        # Timer para procesar y publicar a 5 Hz
        self.create_timer(0.2, self.timer_callback)

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # convertir quaternion a yaw
        _, _, yaw = np.array([0,0,0]), np.array([0,0,0]), \
            2 * math.atan2(ori.z, ori.w)
        self.latest_odom = (pos.x, pos.y, yaw)

    def timer_callback(self):
        if self.latest_scan is None or self.latest_odom is None:
            return

        # Extraer rangos y ángulos
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min,
                             self.latest_scan.angle_max,
                             len(ranges))

        # Actualizar mapa
        self.map.update(ranges, angles, self.latest_odom)

        # Publicar como array a otros nodos (si se desea)
        prob_map = self.map.get_probability_map().flatten()
        msg = Float32MultiArray(data=prob_map.tolist())
        self.map_pub.publish(msg)

        # Visualizar rápido con matplotlib (solo para debugging)
        plt.clf()
        plt.imshow(self.map.get_probability_map(),
                   cmap='gray', origin='lower')
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
