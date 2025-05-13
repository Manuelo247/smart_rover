import numpy as np
import heapq

class Node:
    def __init__(self, position, g=0, h=0):
        self.position = position  # (fila, columna)
        self.g = g  # Costo desde el inicio
        self.h = h  # Heurística al objetivo
        self.f = g + h

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star(start, goal, grid):
    open_list = []
    closed_set = set()
    start_node = Node(start, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    came_from = {}
    g_score = {start: 0}

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal:
            return reconstruct_path(came_from, current_node.position)

        closed_set.add(current_node.position)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current_node.position[0] + dx, current_node.position[1] + dy)

            if (0 <= neighbor[0] < grid.shape[0] and
                0 <= neighbor[1] < grid.shape[1] and
                grid[neighbor] == 0 and
                neighbor not in closed_set):

                tentative_g_score = g_score[current_node.position] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node.position
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_list, Node(neighbor, tentative_g_score, heuristic(neighbor, goal)))

    return None  # No se encontró ruta

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]