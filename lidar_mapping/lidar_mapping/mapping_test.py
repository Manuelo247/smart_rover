#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import math

from lidar_mapping.occupancy_grid_B import OccupancyGridMap

class GridMappingNode(Node):
    def __init__(self):
        super().__init__('grid_mapping_node')
        self.get_logger().info('Inicializando nodo de mapeo...')

        # Parámetros del mapa
        width = 400
        height = 400
        resolution = 0.05
        origin = (-10.0, -10.0)
        origin_x, origin_y = origin
        self.map = OccupancyGridMap(width, height, resolution, origin_x=origin[0], origin_y=origin[1])
        self.get_logger().info(f'Mapa creado: {width}x{height} celdas, resolución {resolution}, origen: ({origin_x}, {origin_y})')

        # Suscriptores
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('Suscripciones a /scan y /odom creadas.')

        # Publicador del mapa
        self.map_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.get_logger().info('Publicador de /occupancy_grid creado.')

        self.latest_scan = None
        self.latest_odom = None

        self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Timer iniciado para publicar a 5 Hz.')

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.get_logger().debug('Mensaje de LiDAR recibido.')

    #def get_yaw_from_quaternion(q):
    #    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #    return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        def get_yaw_from_quaternion(q):
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        #yaw = get_yaw_from_quaternion(ori)
        yaw = get_yaw_from_quaternion(ori)
        self.latest_odom = (pos.x, pos.y, yaw)

    def timer_callback(self):
        if self.latest_scan is None or self.latest_odom is None:
            # Parámetros del mapa
            width = 400
            height = 400
            resolution = 0.05
            origin = (-10.0, -10.0)
            origin_x, origin_y = origin
            self.map = OccupancyGridMap(width, height, resolution, origin_x=origin[0], origin_y=origin[1])
            self.get_logger().info(f'Mapa creado: {width}x{height} celdas, resolución {resolution}, origen: ({origin_x}, {origin_y})')


            self.get_logger().warn('Esperando datos de LiDAR y odometría...')
            return

        self.get_logger().info('Actualizando mapa con nueva lectura.')
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))

        self.map.update(ranges, angles, self.latest_odom)

        prob_map = self.map.get_probability_map()
        data = np.full(prob_map.shape, -1, dtype=np.int8)
        data[prob_map > 0.55] = 100
        data[prob_map < 0.45] = 0

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = MapMetaData()
        msg.info.resolution = self.map.res
        msg.info.width = self.map.width
        msg.info.height = self.map.height
        msg.info.origin.position.x = self.map.origin_x
        msg.info.origin.position.y = self.map.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.flatten().tolist()

        self.map_pub.publish(msg)
        self.get_logger().info('Mapa publicado.')

def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario.')
    finally:
        node.get_logger().info('Cerrando nodo...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
