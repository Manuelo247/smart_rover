#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
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

        # Publicador del mapa como OccupancyGrid para RViz2
        self.map_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

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

        # --- Publicar OccupancyGrid para RViz2 ---
        prob_map = self.map.get_probability_map()
        # Convertir a formato OccupancyGrid: 0=libre, 100=ocupado, -1=desconocido
        data = np.full(prob_map.shape, -1, dtype=np.int8)
        data[prob_map > 0.55] = 100
        data[prob_map < 0.45] = 0
        # Rellenar mensaje
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
        
        # Puedes eliminar o comentar la visualización matplotlib
        # plt.clf()
        # plt.imshow(self.map.get_probability_map(),
        #            cmap='gray', origin='lower')
        # plt.pause(0.001)

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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
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

        # Publicador del mapa como OccupancyGrid para RViz2
        self.map_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

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

        # --- Publicar OccupancyGrid para RViz2 ---
        prob_map = self.map.get_probability_map()
        # Convertir a formato OccupancyGrid: 0=libre, 100=ocupado, -1=desconocido
        data = np.full(prob_map.shape, -1, dtype=np.int8)
        data[prob_map > 0.55] = 100
        data[prob_map < 0.45] = 0
        # Rellenar mensaje
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
        
        # Puedes eliminar o comentar la visualización matplotlib
        # plt.clf()
        # plt.imshow(self.map.get_probability_map(),
        #            cmap='gray', origin='lower')
        # plt.pause(0.001)

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
