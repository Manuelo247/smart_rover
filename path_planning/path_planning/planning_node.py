import os
import math
import yaml
import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from path_planning.a_star import a_star
from path_planning.theta_star import theta_star

# =========================
# PARÁMETROS AJUSTABLES
# =========================
ROBOT_RADIUS_M = 0.09
COST_ALPHA = 10.0
COST_BETA = 5.0
MAP_FILENAME = 'mapa_ines.yaml'

# =========================
# FUNCIONES DE UTILIDAD
# =========================

def world_to_map(x, y, origin, resolution):
    mx = int((x - origin[0]) / resolution)
    my = int((y - origin[1]) / resolution)
    return (my, mx)

def map_to_world(my, mx, origin, resolution):
    x = mx * resolution + origin[0]
    y = my * resolution + origin[1]
    return x, y

def inflate_obstacles(grid, robot_radius_cells):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * robot_radius_cells + 1, 2 * robot_radius_cells + 1))
    inflated = cv2.dilate((grid > 0).astype(np.uint8), kernel)
    return np.where(inflated > 0, 100, 0)

def cost(d, alpha=COST_ALPHA, beta=COST_BETA):
    return 1 + alpha * math.exp(-beta * d)

def generate_costmap(grid, alpha=COST_ALPHA, beta=COST_BETA):
    obstacle_mask = (grid > 0)
    dist = distance_transform_edt(~obstacle_mask)
    vectorized_cost = np.vectorize(cost)
    costmap = vectorized_cost(dist, alpha, beta)
    costmap[obstacle_mask] = np.inf
    return costmap

# =========================
# NODO DE PLANIFICACIÓN
# =========================

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('planner')

        self.grid = None
        self.resolution = None
        self.origin = None

        # Subscripciones y publicaciones
        self.current_pose = None
        self.current_pose_sub = self.create_subscription(PoseStamped, 'odom/localization_pose', self.update_current_pose, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose/global', self.goal_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.path_pub = self.create_publisher(Path, 'planning/planned_path', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, 'planning/inflated_map', 1)

        self.get_logger().info('Planner iniciado (esperando mapa y pose).')

    def update_current_pose(self, msg):
        self.current_pose = msg

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        grid = np.where(data == 0, 0, 100)
        grid[data == -1] = 100
        self.grid = grid
        self.resolution = msg.info.resolution
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        robot_radius_cells = math.ceil(ROBOT_RADIUS_M / self.resolution)
        self.grid = inflate_obstacles(self.grid, robot_radius_cells)
        self.get_logger().info('Mapa recibido y procesado.')

        # Publicar el mapa inflado
        inflated_msg = OccupancyGrid()
        inflated_msg.header.stamp = self.get_clock().now().to_msg()
        inflated_msg.header.frame_id = 'map'
        inflated_msg.info.width = self.grid.shape[1]
        inflated_msg.info.height = self.grid.shape[0]
        inflated_msg.info.resolution = self.resolution
        inflated_msg.info.origin.position.x = self.origin[0]
        inflated_msg.info.origin.position.y = self.origin[1]
        inflated_msg.info.origin.position.z = 0.0
        inflated_msg.info.origin.orientation.w = 1.0
        inflated_msg.data = self.grid.astype(np.int8).flatten().tolist()
        self.inflated_map_pub.publish(inflated_msg)

    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn('Esperando posición actual...')
            return
        if self.grid is None:
            self.get_logger().warn('Esperando mapa...')
            return

        # Convertir coordenadas
        start = world_to_map(self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.origin, self.resolution)
        goal = world_to_map(msg.pose.position.x, msg.pose.position.y, self.origin, self.resolution)

        if (goal[0] < 0 or goal[0] >= self.grid.shape[0] or goal[1] < 0 or goal[1] >= self.grid.shape[1]):
            self.get_logger().warn('El objetivo está fuera del mapa.')
            return
        if (self.grid[goal] != 0):
            self.get_logger().warn('El objetivo está en una zona ocupada o peligrosa.')
            return

        # Planificación
        path_idx = theta_star(start, goal, self.grid, None)

        if path_idx is None:
            self.get_logger().warn('No se encontró ruta.')
            return

        # Publicar ruta
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
        self.get_logger().info('Ruta publicada.')

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
