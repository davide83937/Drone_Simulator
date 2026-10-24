import math
import struct
import numpy as np
from obstacle_config import ObstacleConfig
from Obstacle import ObstacleAABB
from PathPlanning.getObstacles import get_obstacles_lidar_manager


class LidarManager:
    def __init__(self, max_points=30000):
        self.max_points = max_points if max_points is not None else ObstacleConfig.get_max_accumulated_points()
        self.accumulated_points = np.zeros((0, 3))
        self.tracked_obstacles = []
        self.next_obstacle_id = 0
        self.R_yaw = np.eye(3)

        # Lista degli ostacoli statici noti
        self.static_obstacles = get_obstacles_lidar_manager()

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

    def get_center_and_dimension_from_cluster(self, cluster_pcd):
        aabb = cluster_pcd.get_axis_aligned_bounding_box()
        center = aabb.get_center()
        extent = aabb.get_extent()
        c_x, c_y, c_z = center[0], center[1], center[2]
        exp = ObstacleConfig.get_cluster_xy_expansion_factor()
        d_x, d_y, d_z = extent[0] + extent[0] * exp, extent[1] + extent[1] * exp, extent[2]
        return c_x, c_y, c_z, d_x, d_y, d_z, aabb

    def is_obstacle_inside(self, c_x, c_y, c_z):
        for static_obs in self.static_obstacles:
            if static_obs.is_inside(c_x, c_y, c_z):
                return True
        return False

    def is_obstacles_mobile(self, c_x, c_y, c_z, min_dist=None):
        if min_dist is None:
            min_dist = ObstacleConfig.get_tracking_max_match_distance()
        matched_tracker = None
        for tracker in self.tracked_obstacles:
            dist = math.sqrt((c_x - tracker['center'][0]) ** 2 +
                             (c_y - tracker['center'][1]) ** 2 +
                             (c_z - tracker['center'][2]) ** 2)
            if dist < min_dist:
                min_dist = dist
                matched_tracker = tracker
        return matched_tracker, min_dist

    def detect_movement(self, matched_tracker, current_time, aabb, c_x, c_y, c_z, d_x, d_y, d_z):
        matched_tracker['stationary_since'] = current_time
        matched_tracker['center'] = (c_x, c_y, c_z)
        matched_tracker['extent'] = (d_x, d_y, d_z)
        aabb.color = (1, 0, 0)
        print(f"[TRACKING] ID {matched_tracker['id']} in movimento -> Pos X:{c_x:.1f}, Y:{c_y:.1f}")

    def new_fixed_obstacle_detected(self, current_time, matched_tracker, aabb):
        """Restituisce i dati dell'ostacolo (obs_data) se viene promosso, altrimenti None"""
        tempo_fermo = current_time - matched_tracker['stationary_since']
        if tempo_fermo >= ObstacleConfig.get_static_promotion_time_sec():
            print(f"[PROMOZIONE] L'intruso ID {matched_tracker['id']} è fermo. Aggiunto alla mappa statica.")
            width = matched_tracker['extent'][0]
            depth = matched_tracker['extent'][1]
            height = matched_tracker['extent'][2]

            nuovo_ostacolo = ObstacleAABB(
                x=matched_tracker['center'][0],
                y=matched_tracker['center'][1],
                z=matched_tracker['center'][2],
                width=width,
                depth=depth,
                height=height,
                drone_radius=ObstacleConfig.get_drone_safety_radius()
            )
            self.static_obstacles.append(nuovo_ostacolo)
            matched_tracker['promosso'] = True
            aabb.color = (0, 0, 1)

            obs_data = {
                'x': matched_tracker['center'][0],
                'y': matched_tracker['center'][1],
                'z': matched_tracker['center'][2],
                'w': matched_tracker['extent'][0],
                'd': matched_tracker['extent'][1],
                'h': matched_tracker['extent'][2]
            }
            return obs_data
        else:
            aabb.color = (1, 0, 0)
            return None

    def new_obstacle_detected(self, aabb, current_time, c_x, c_y, c_z, d_x, d_y, d_z):
        print(f"[NUOVO OSTACOLO] Intruso ID {self.next_obstacle_id} in X:{c_x:.1f}, Y:{c_y:.1f} | Analisi stabilità...")
        self.tracked_obstacles.append({
            'id': self.next_obstacle_id,
            'center': (c_x, c_y, c_z),
            'extent': (d_x, d_y, d_z),
            'last_seen': current_time,
            'stationary_since': current_time,
            'promosso': False
        })
        self.next_obstacle_id += 1
        aabb.color = (1, 0, 0)

    def clear_obstacles(self, current_time):
        ostacoli_rimasti = []
        for tracker in self.tracked_obstacles:
            if tracker.get('promosso', False):
                continue
            if current_time - tracker['last_seen'] < ObstacleConfig.get_obstacle_forget_time_sec():
                ostacoli_rimasti.append(tracker)
            else:
                print(f"[-] OSTACOLO ID {tracker['id']} PERSO. Eliminato dalla memoria.")
        self.tracked_obstacles = ostacoli_rimasti