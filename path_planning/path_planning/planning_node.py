import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
import cv2
import os

from path_planning.a_star import a_star 
from path_planning.theta_star import theta_star

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])
    grid = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    grid = np.where(grid == 0, 0, 100)  # 0 libre, 100 ocupado
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    return grid, resolution, origin

def world_to_map(x, y, origin, resolution):
    mx = int((x - origin[0]) / resolution)
    my = int((y - origin[1]) / resolution)
    return (my, mx)  # Nota: fila, columna

def map_to_world(my, mx, origin, resolution):
    x = mx * resolution + origin[0]
    y = my * resolution + origin[1]
    return x, y

def inflate_obstacles(grid, robot_radius_cells):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*robot_radius_cells+1, 2*robot_radius_cells+1))
    inflated = cv2.dilate((grid > 0).astype(np.uint8), kernel)
    return np.where(inflated > 0, 100, 0)

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('planner')
        # Obtén la ruta absoluta del paquete
        pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        yaml_path = os.path.join(pkg_path, 'resource', 'ros2_custom_map.yaml')
        self.grid, self.resolution, self.origin = load_map(yaml_path)
        # Inflar obstáculos según el radio del robot
        robot_radius_m = 0.25  # Cambia esto según el radio de tu robot (en metros)
        robot_radius_cells = int(robot_radius_m / self.resolution)
        self.grid = inflate_obstacles(self.grid, robot_radius_cells)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.current_pose = None
        self.pose_sub = self.create_subscription(PoseStamped, 'current_pose', self.pose_callback, 10)
        self.get_logger().info('A* / Theta* planner node ready!')

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn('Esperando posición actual...')
            return
        # Convertir posiciones a índices de mapa
        start = world_to_map(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.origin, self.resolution
        )
        goal = world_to_map(
            msg.pose.position.x,
            msg.pose.position.y,
            self.origin, self.resolution
        )
        # Elige el algoritmo aquí:
        # path_idx = a_star(start, goal, self.grid)
        path_idx = theta_star(start, goal, self.grid)
        if path_idx is None:
            self.get_logger().warn('No se encontró ruta')
            return
        # Convertir índices a Path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for idx in path_idx:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            x, y = map_to_world(idx[0], idx[1], self.origin, self.resolution)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info('Ruta publicada')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()