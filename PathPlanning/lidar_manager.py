import math
import struct
import numpy as np
from Obstacle import ObstacleAABB


class LidarManager:
    def __init__(self, max_points=30000):
        self.max_points = max_points
        self.accumulated_points = np.zeros((0, 3))
        self.tracked_obstacles = []
        self.next_obstacle_id = 0
        self.R_yaw = np.eye(3)

        # Lista degli ostacoli statici noti
        self.static_obstacles = [
            # Dritto 1
            ObstacleAABB(x=-46.0, y=-75.0, z=35.0, width=13.0, depth=200.0, height=60.0, drone_radius=5.0),
            # Rovesciato 2
            ObstacleAABB(x=-126.0, y=-173.0, z=35.0, width=200.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 3
            ObstacleAABB(x=45.0, y=-75.0, z=35.0, width=13.0, depth=200.0, height=60.0, drone_radius=5.0),
            # Rovesciato 4
            ObstacleAABB(x=-68.0, y=-233.0, z=35.0, width=200.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 5
            ObstacleAABB(x=-243.0, y=-269.0, z=35.0, width=13.0, depth=200.0, height=60.0, drone_radius=5.0),
            # Rovesciato 6
            ObstacleAABB(x=-151.0, y=-390.0, z=35.0, width=200.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Rovesciato 7
            ObstacleAABB(x=-73.0, y=-315.0, z=35.0, width=200.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 8
            ObstacleAABB(x=-161.0, y=-270.0, z=35.0, width=13.0, depth=60.0, height=60.0, drone_radius=5.0),
            # Rovesciato 9
            ObstacleAABB(x=84.0, y=-160.0, z=35.0, width=60.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 10
            ObstacleAABB(x=104.0, y=-202.0, z=35.0, width=13.0, depth=60.0, height=60.0, drone_radius=5.0),
            # Rovesciato 11
            ObstacleAABB(x=63.0, y=-239.0, z=35.0, width=60.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 12
            ObstacleAABB(x=-48.0, y=-427.0, z=35.0, width=13.0, depth=60.0, height=60.0, drone_radius=5.0),
            # Rovesciato 13
            ObstacleAABB(x=-9.0, y=-458.0, z=35.0, width=60.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Rovesciato 14
            ObstacleAABB(x=60.0, y=-458.0, z=35.0, width=60.0, depth=13.0, height=60.0, drone_radius=5.0),
            # Dritto 15
            ObstacleAABB(x=104.0, y=-427.0, z=35.0, width=13.0, depth=60.0, height=60.0, drone_radius=5.0),
            # Dritto 16
            ObstacleAABB(x=104.0, y=-356.0, z=35.0, width=13.0, depth=60.0, height=60.0, drone_radius=5.0),
            # Rovesciato 17
            ObstacleAABB(x=67.0, y=-315.0, z=35.0, width=60.0, depth=13.0, height=60.0, drone_radius=5.0),
        ]

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
        d_x, d_y, d_z = extent[0]+extent[0]*0.1, extent[1]+extent[1]*0.1, extent[2]#*extent[2]*0.1
        return c_x, c_y, c_z, d_x, d_y, d_z, aabb

    def is_obstacle_inside(self, c_x, c_y, c_z):
        for static_obs in self.static_obstacles:
            if static_obs.is_inside(c_x, c_y, c_z):
                return True
        return False

    def is_obstacles_mobile(self, c_x, c_y, c_z, min_dist=4.0):
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
        if tempo_fermo >= 0.9:
            print(f"[PROMOZIONE] L'intruso ID {matched_tracker['id']} è fermo. Aggiunto alla mappa statica.")

            nuovo_ostacolo = ObstacleAABB(
                x=matched_tracker['center'][0],
                y=matched_tracker['center'][1],
                z=matched_tracker['center'][2],
                width=matched_tracker['extent'][0],
                depth=matched_tracker['extent'][1],
                height=matched_tracker['extent'][2],
                drone_radius=7.0
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
            if current_time - tracker['last_seen'] < 2.0:
                ostacoli_rimasti.append(tracker)
            else:
                print(f"[-] OSTACOLO ID {tracker['id']} PERSO. Eliminato dalla memoria.")
        self.tracked_obstacles = ostacoli_rimasti