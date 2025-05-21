#!/usr/bin/env python3
import numpy as np
import math

def prob_to_log_odds(p):
    """Convierte probabilidad [0,1] a log-odds."""
    return np.log(p / (1 - p))

def log_odds_to_prob(l):
    """Convierte log-odds a probabilidad [0,1]."""
    return 1 - 1/(1 + np.exp(l))

def bresenham(x0, y0, x1, y1):
    """Implementación de Bresenham para trazar una línea de celdas entre dos puntos."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells

class OccupancyGridMap:
    def __init__(self, width, height, resolution, origin_x=0.0, origin_y=0.0,
                 p0=0.5, p_occ=0.7, p_free=0.3, lo_max=20.0, lo_min=-20.0):
        """
        width, height: tamaño en número de celdas
        resolution: metros por celda
        origin: coordenadas del voxel (0,0) en el marco del mundo
        p0: probabilidad inicial (0.5 = desconocido)
        p_occ/free: probabilidades modelo sensor
        lo_max/min: límites de log-odds
        """
        self.width = width
        self.height = height
        self.res = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        # inicializa matriz de log-odds con log_odds(p0)
        self.log_odds = np.ones((height, width)) * prob_to_log_odds(p0)
        self.lo_max = lo_max
        self.lo_min = lo_min
        self.lo_occ = prob_to_log_odds(p_occ)
        self.lo_free = prob_to_log_odds(p_free)

    def world_to_map(self, x, y):
        """Convierte coordenadas del mundo a índices de grilla (i, j)."""
        j = int((x - self.origin_x) / self.res)
        i = int((y - self.origin_y) / self.res)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def inverse_sensor_model(self, i, j, endpoint):
        """
        Calcula delta log-odds para la celda (i,j) dado que 'endpoint' 
        es la celda final del rayo.
        """
        if (i, j) == endpoint:
            return self.lo_occ
        else:
            return self.lo_free

    def update_ray(self, robot_xy, endpoint_xy):
        """
        Actualiza todas las celdas por donde pasa el rayo y el endpoint.
        robot_xy, endpoint_xy: tuplas de índices (i,j).
        """
        ray = bresenham(robot_xy[1], robot_xy[0], endpoint_xy[1], endpoint_xy[0])
        for idx, (j, i) in enumerate(ray):
            if not self.in_bounds(i, j):
                continue
            # endpoint: última celda
            if idx == len(ray) - 1:
                delta = self.inverse_sensor_model((i, j), (i, j), endpoint_xy)
            else:
                delta = self.lo_free
            self.log_odds[i, j] = np.clip(
                self.log_odds[i, j] + delta,
                self.lo_min, self.lo_max
            )

    def update(self, scan_ranges, scan_angles, robot_pose):
        """
        scan_ranges: lista de distancias
        scan_angles: lista de ángulos (relativos al robot)
        robot_pose: (x, y, theta) en coordenadas del mundo
        """
        rx, ry, rtheta = robot_pose
        robot_i, robot_j = self.world_to_map(rx, ry)

        for r, ang in zip(scan_ranges, scan_angles):
            # filtrar lecturas inválidas
            if np.isinf(r) or np.isnan(r):
                continue
            # punto en el mundo
            wx = rx + r * math.cos(rtheta + ang)
            wy = ry + r * math.sin(rtheta + ang)
            end_i, end_j = self.world_to_map(wx, wy)
            # actualizar ray
            self.update_ray((robot_i, robot_j), (end_i, end_j))

    def get_probability_map(self):
        """Devuelve matriz de probabilidades (0–1)."""
        return log_odds_to_prob(self.log_odds)
