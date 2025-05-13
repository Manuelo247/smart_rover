import numpy as np
import heapq

# =========================
# CLASES Y FUNCIONES BÁSICAS
# =========================

class Node:
    """Nodo para el algoritmo Theta*"""
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position  # (fila, columna)
        self.g = g                # Coste acumulado
        self.h = h                # Heurística al objetivo
        self.f = g + h            # Coste total estimado
        self.parent = parent      # Nodo padre

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    """Distancia euclidiana entre dos puntos"""
    return np.linalg.norm(np.array(a) - np.array(b))

def line_of_sight(grid, p0, p1):
    """
    Algoritmo de Bresenham para comprobar si hay línea de visión libre entre dos puntos.
    Devuelve True si no hay obstáculos en la línea recta entre p0 y p1.
    """
    x0, y0 = p0
    x1, y1 = p1
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if not (0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]) or grid[x, y] != 0:
            return False
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return True

def bresenham(x0, y0, x1, y1):
    """Generador de puntos entre (x0, y0) y (x1, y1) usando el algoritmo de Bresenham."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            yield (x, y)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        yield (x, y)
    else:
        err = dy / 2.0
        while y != y1:
            yield (x, y)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        yield (x, y)

def line_cost(costmap, p0, p1):
    """Suma el coste de todas las celdas a lo largo de la línea de visión entre p0 y p1."""
    x0, y0 = p0
    x1, y1 = p1
    points = list(bresenham(x0, y0, x1, y1))
    return sum(costmap[x, y] for x, y in points)

# =========================
# ALGORITMO THETA*
# =========================

def theta_star(start, goal, grid, costmap):
    """
    Algoritmo Theta* para planificación de rutas con costmap.
    Minimiza el coste acumulado, considerando la cercanía a obstáculos.
    """
    open_list = []
    closed_set = set()
    start_node = Node(start, 0, heuristic(start, goal), parent=None)
    heapq.heappush(open_list, start_node)
    nodes = {start: start_node}

    while open_list:
        current = heapq.heappop(open_list)
        if current.position == goal:
            return reconstruct_path(current)
        closed_set.add(current.position)

        # Movimientos en 8 direcciones
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            neighbor_pos = (current.position[0] + dx, current.position[1] + dy)
            if (0 <= neighbor_pos[0] < grid.shape[0] and
                0 <= neighbor_pos[1] < grid.shape[1] and
                grid[neighbor_pos] == 0 and
                neighbor_pos not in closed_set):

                # Línea de visión desde el padre
                if current.parent and line_of_sight(grid, current.parent.position, neighbor_pos):
                    parent = current.parent
                    g = parent.g + line_cost(costmap, parent.position, neighbor_pos)
                else:
                    parent = current
                    g = current.g + costmap[neighbor_pos]

                if neighbor_pos not in nodes or g < nodes[neighbor_pos].g:
                    h = heuristic(neighbor_pos, goal)
                    neighbor_node = Node(neighbor_pos, g, h, parent)
                    nodes[neighbor_pos] = neighbor_node
                    heapq.heappush(open_list, neighbor_node)
    return None

def reconstruct_path(node):
    """Reconstruye el camino desde el nodo objetivo hasta el inicio"""
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]