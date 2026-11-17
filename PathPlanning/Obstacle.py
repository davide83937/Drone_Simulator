import math
import heapq
from obstacle_config import ObstacleConfig


class ObstacleAABB:
    def __init__(self, x, y, z, width, depth, height, drone_radius=None):
        if drone_radius is None:
            drone_radius = ObstacleConfig.get_default_aabb_drone_radius()
        self.min_x = x - (width / 2) - drone_radius
        self.max_x = x + (width / 2) + drone_radius
        self.min_y = y - (depth / 2) - drone_radius
        self.max_y = y + (depth / 2) + drone_radius
        self.min_z = z - (height / 2) - drone_radius
        self.max_z = z + (height / 2) + drone_radius




