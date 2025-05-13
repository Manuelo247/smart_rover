import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import os

MAP_YAML = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    'resource',
    'ros2_custom_map.yaml'
)

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 1)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.map_msg = self.load_map(MAP_YAML)
        self.get_logger().info('Publicando mapa en /map')

    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)
        image_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])
        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise RuntimeError(f"No se pudo cargar la imagen del mapa: {image_path}")
        # Convertir a formato OccupancyGrid (0: libre, 100: ocupado, -1: desconocido)
        grid = np.where(img == 0, 0, 100).astype(np.int8)
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(map_metadata['resolution'])
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        origin = map_metadata['origin']
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()