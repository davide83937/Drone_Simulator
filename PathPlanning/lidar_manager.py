import math
import struct
import numpy as np
from obstacle_config import ObstacleConfig
from Obstacle import ObstacleAABB



class LidarManager:
    def __init__(self, max_points=30000):
        self.max_points = max_points if max_points is not None else ObstacleConfig.get_max_accumulated_points()
        self.accumulated_points = np.zeros((0, 3))
        self.tracked_obstacles = []
        self.next_obstacle_id = 0
        self.R_yaw = np.eye(3)

        # Lista degli ostacoli statici noti


    def update_rotation_matrix(self, theta):
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        self.R_yaw = np.array([
            [cos_t, -sin_t, 0],
            [sin_t,  cos_t, 0],
            [    0,      0, 1]
        ])
        return self.R_yaw

    def lidar_points_extracting(self, num_floats, data, drone_pos):
        if num_floats % 3 == 0:
            floats = struct.unpack(f'<{num_floats}f', data)
            local_points = np.array(floats).reshape(-1, 3)

            rotated_points = local_points.dot(self.R_yaw.T)
            absolute_points = rotated_points + drone_pos
            self.accumulated_points = np.vstack((self.accumulated_points, absolute_points))

            if len(self.accumulated_points) > self.max_points:
                self.accumulated_points = self.accumulated_points[-self.max_points:]
        return self.accumulated_points

