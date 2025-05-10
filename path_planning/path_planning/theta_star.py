import numpy as np
import heapq

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def line_of_sight(grid, p0, p1):
    """Bresenham's line algorithm to check if path is free."""
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

def theta_star(start, goal, grid, costmap):
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

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            neighbor_pos = (current.position[0] + dx, current.position[1] + dy)
            if (0 <= neighbor_pos[0] < grid.shape[0] and
                0 <= neighbor_pos[1] < grid.shape[1] and
                grid[neighbor_pos] == 0 and
                neighbor_pos not in closed_set):

                if current.parent and line_of_sight(grid, current.parent.position, neighbor_pos):
                    parent = current.parent
                    # Suma el coste de la celda destino
                    g = parent.g + costmap[neighbor_pos]
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
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]