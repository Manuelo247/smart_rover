import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
import cv2
import os
import math
from scipy.ndimage import distance_transform_edt

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

def cost(d, alpha=10.0, beta=2.0):
    return 1 + alpha * math.exp(-beta * d)

def generate_costmap(grid, alpha=10.0, beta=2.0):
    # grid: 0 = libre, >0 = ocupado
    obstacle_mask = (grid > 0)
    # Distancia en celdas a obstáculo más cercano
    dist = distance_transform_edt(~obstacle_mask)
    # Aplica la función de coste a cada celda
    vectorized_cost = np.vectorize(cost)
    costmap = vectorized_cost(dist, alpha, beta)
    # Opcional: pon coste muy alto en obstáculos
    costmap[obstacle_mask] = np.inf
    return costmap

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
        self.costmap = generate_costmap(self.grid)
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
        # NUEVO: Verifica si el objetivo está dentro del mapa
        if (goal[0] < 0 or goal[0] >= self.grid.shape[0] or
            goal[1] < 0 or goal[1] >= self.grid.shape[1]):
            self.get_logger().warn('El objetivo está FUERA del mapa. No se puede planificar.')
            return
        # Verifica si el objetivo está en un obstáculo
        if (self.grid[goal] != 0) or np.isinf(self.costmap[goal]):
            self.get_logger().warn('El objetivo está en una zona prohibida: dentro de una pared o demasiado cerca.')
            return
        # Elige el algoritmo aquí:
        # path_idx = a_star(start, goal, self.grid)
        path_idx = theta_star(start, goal, self.grid, self.costmap)
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