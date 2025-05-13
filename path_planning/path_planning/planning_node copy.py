import os
import math
import yaml
import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from path_planning.a_star import a_star
from path_planning.theta_star import theta_star

# =========================
# PARÁMETROS AJUSTABLES
# =========================
ROBOT_RADIUS_M = 0.15      # Radio de seguridad del robot en metros
COST_ALPHA = 10.0          # Parámetro alpha para la función de coste
COST_BETA = 5.0           # Parámetro beta para la función de coste
MAP_FILENAME = 'ros2_custom_map.yaml'  # Nombre del archivo de mapa

# =========================
# FUNCIONES DE UTILIDAD
# =========================

def load_map(yaml_path):
    """Carga el mapa y metadatos desde archivos .yaml y .pgm"""
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])
    grid = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    grid = np.where(grid == 0, 0, 100)  # 0 libre, 100 ocupado
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    return grid, resolution, origin

def world_to_map(x, y, origin, resolution):
    """Convierte coordenadas reales a índices de la matriz del mapa"""
    mx = int((x - origin[0]) / resolution)
    my = int((y - origin[1]) / resolution)
    return (my, mx)  # fila, columna

def map_to_world(my, mx, origin, resolution):
    """Convierte índices de la matriz a coordenadas reales"""
    x = mx * resolution + origin[0]
    y = my * resolution + origin[1]
    return x, y

def inflate_obstacles(grid, robot_radius_cells):
    """Infla los obstáculos en forma cuadrada según el radio del robot"""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2*robot_radius_cells+1, 2*robot_radius_cells+1))
    inflated = cv2.dilate((grid > 0).astype(np.uint8), kernel)
    return np.where(inflated > 0, 100, 0)

def cost(d, alpha=COST_ALPHA, beta=COST_BETA):
    """Función de coste según la distancia a obstáculos"""
    return 1 + alpha * math.exp(-beta * d)

def generate_costmap(grid, alpha=COST_ALPHA, beta=COST_BETA):
    """Genera un costmap ponderado a partir del grid binario"""
    obstacle_mask = (grid > 0)
    dist = distance_transform_edt(~obstacle_mask)
    vectorized_cost = np.vectorize(cost)
    costmap = vectorized_cost(dist, alpha, beta)
    costmap[obstacle_mask] = np.inf  # Obstáculos: coste infinito
    return costmap

# =========================
# NODO DE PLANIFICACIÓN
# =========================

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('planner')

        # --- Inicialización de mapa y costmap ---
        pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        yaml_path = os.path.join(pkg_path, 'resource', MAP_FILENAME)
        self.grid, self.resolution, self.origin = load_map(yaml_path)

        robot_radius_cells = math.ceil(ROBOT_RADIUS_M / self.resolution)
        self.grid = inflate_obstacles(self.grid, robot_radius_cells)
        self.costmap = generate_costmap(self.grid)

        # --- Subscripciones y publicaciones ---
        self.current_pose = None
        self.pose_sub = self.create_subscription(PoseStamped, 'current_pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        self.get_logger().info('A* / Theta* planner node listo.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn('Esperando posición actual...')
            return

        # --- Conversión de coordenadas ---
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

        # --- Validación del objetivo ---
        if (goal[0] < 0 or goal[0] >= self.grid.shape[0] or
            goal[1] < 0 or goal[1] >= self.grid.shape[1]):
            self.get_logger().warn('El objetivo está FUERA del mapa. No se puede planificar.')
            return
        if (self.grid[goal] != 0) or np.isinf(self.costmap[goal]):
            self.get_logger().warn('El objetivo está en una zona prohibida: dentro de una pared o demasiado cerca según el tamaño del robot.')
            return

        # --- Planificación (elige el algoritmo aquí) ---
        # path_idx = a_star(start, goal, self.grid)
        path_idx = theta_star(start, goal, self.grid, self.costmap)

        if path_idx is None:
            self.get_logger().warn('No se encontró ruta')
            return

        # --- Publicación de la ruta ---
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

# =========================
# MAIN
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()