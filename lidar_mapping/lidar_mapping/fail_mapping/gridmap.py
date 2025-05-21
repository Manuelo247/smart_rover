import numpy as np

class OccupancyGridMap:
    def __init__(self, width, height, resolution, log_odds_min=-5, log_odds_max=5):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.log_odds_min = log_odds_min
        self.log_odds_max = log_odds_max

        self.grid = np.zeros((height, width), dtype=np.float32)

    def world_to_map(self, x, y):
        mx = int(x / self.resolution + self.width // 2)
        my = int(y / self.resolution + self.height // 2)
        return mx, my

    def update_cell(self, x, y, delta_log_odds):
        mx, my = self.world_to_map(x, y)
        if 0 <= mx < self.width and 0 <= my < self.height:
            self.grid[my, mx] += delta_log_odds
            self.grid[my, mx] = np.clip(self.grid[my, mx], self.log_odds_min, self.log_odds_max)

    def get_probability_map(self):
        return 1 - 1 / (1 + np.exp(self.grid))  # Sigmoide invertida
