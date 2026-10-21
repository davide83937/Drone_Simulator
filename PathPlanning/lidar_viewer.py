import os
import socket
import numpy as np
import time
import json
from std_msgs.msg import String

from PathPlanning.lidar_graphics import (
    start_window,
    render_obstacles,
    render_drone,
    update_drone_position,
    render_points,
    render_new_obstacles,
    clear_points,
)
from lidar_manager import LidarManager

# --- SETTINGS AMBIENTE WINDOWS ROS 2 ---
extra_dll_dirs = [
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\Library\bin",
]

for directory in extra_dll_dirs:
    if os.path.isdir(directory):
        os.add_dll_directory(directory)
        os.environ["PATH"] = directory + os.pathsep + os.environ.get("PATH", "")

prefix_path = (
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces;"
    r"C:\Users\david\OneDrive\Desktop\ros2-windows"
)
os.environ.setdefault("AMENT_PREFIX_PATH", prefix_path)
# ----------------------------------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


# --- NODO ROS 2 PER POSIZIONE, ROTAZIONE E ALLARMI ---
class DroneStateSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_viewer_state_sub')
        self.create_subscription(Vector3, 'drone_position', self.pos_callback, 10)
        self.create_subscription(Vector3, 'drone_rotation', self.rot_callback, 10)
        self.create_subscription(String, 'blocking_obstacle', self.blocking_obs_callback, 10)

        self.static_obs_pub = self.create_publisher(String, 'new_static_obstacle', 10)
        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_yaw = 0.0
        self.blocking_obstacles_queue = []

    def pos_callback(self, msg):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])

    def rot_callback(self, msg):
        self.drone_yaw = np.radians(msg.z)

    def blocking_obs_callback(self, msg):
        self.blocking_obstacles_queue.append(json.loads(msg.data))

    def publish_static_obstacle(self, obs_data):
        """Invia al PathManager ROS 2 i dati dell'ostacolo promosso dal LidarManager"""
        msg = String()
        msg.data = json.dumps(obs_data)
        self.static_obs_pub.publish(msg)
        print("-> Notifica inviata al Navigatore ROS 2")


# --- CONFIGURAZIONE LIDAR UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 9999
MAX_POINTS = 30000


def main():
    rclpy.init()
    state_node = DroneStateSubscriber()
    lidar = LidarManager(max_points=MAX_POINTS)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    # GRAFICA ------------------------------
    vis, pcd = start_window()
    drone_mesh, base_drone_vertices = render_drone(vis)
    render_obstacles(lidar.static_obstacles, vis)
    # --------------------------------------

    first_points_received = False
    last_print_time = time.time()
    current_bboxes = []
    last_cluster_time = time.time()

    print(f"In ascolto su {UDP_IP}:{UDP_PORT} per il LiDAR...")

    while True:
        rclpy.spin_once(state_node, timeout_sec=0.0)
        current_time = time.time()

        if current_time - last_print_time >= 1.0:
            deg_yaw = np.degrees(state_node.drone_yaw)
            print(f"[DRONE] Pos: X:{state_node.drone_pos[0]:.2f} | Y:{state_node.drone_pos[1]:.2f} | Z:{state_node.drone_pos[2]:.2f} || Yaw: {deg_yaw:.1f}°")
            last_print_time = current_time

        # --- AGGIORNAMENTO MATRICE DI ROTAZIONE E GRAFICA DRONE ---
        R_yaw = lidar.update_rotation_matrix(state_node.drone_yaw)

        # GRAFICA -------------------------------------------------------------------------
        update_drone_position(base_drone_vertices, drone_mesh, R_yaw, state_node, vis)
        render_new_obstacles(state_node, vis)
        # ---------------------------------------------------------------------------------

        try:
            data, addr = sock.recvfrom(65535)
            num_floats = len(data) // 4

            accumulated_points = lidar.lidar_points_extracting(num_floats, data, state_node.drone_pos)

            # GRAFICA ---------------------------------------------------------------------
            first_points_received = render_points(vis, pcd, accumulated_points, first_points_received)
            # -----------------------------------------------------------------------------

        except BlockingIOError:
            pass
        except KeyboardInterrupt:
            break
        except Exception:
            pass

        # --- LOGICA DI FILTRAGGIO OSTACOLI ---
        if current_time - last_cluster_time >= 0.3 and len(np.asarray(pcd.points)) > 50:
            last_cluster_time = current_time

            # GRAFICA ---------------------------------------------------------------------
            clear_points(current_bboxes, vis)
            # -----------------------------------------------------------------------------

            downpcd = pcd.voxel_down_sample(voxel_size=0.5)
            labels = np.array(downpcd.cluster_dbscan(eps=4.5, min_points=45, print_progress=False))

            if len(labels) > 0:
                max_label = labels.max()

                for i in range(max_label + 1):
                    cluster_indices = np.where(labels == i)[0]
                    cluster_pcd = downpcd.select_by_index(cluster_indices)
                    c_x, c_y, c_z, d_x, d_y, d_z, aabb = lidar.get_center_and_dimension_from_cluster(cluster_pcd)

                    if lidar.is_obstacle_inside(c_x, c_y, c_z):
                        aabb.color = (0, 0, 1)
                    else:
                        matched_tracker, min_dist = lidar.is_obstacles_mobile(c_x, c_y, c_z, min_dist=4.0)

                        if matched_tracker:
                            matched_tracker['last_seen'] = current_time
                            if min_dist > 2.5:
                                lidar.detect_movement(matched_tracker, current_time, aabb, c_x, c_y, c_z, d_x, d_y, d_z)
                            else:
                                promoted_obs_data = lidar.new_fixed_obstacle_detected(current_time, matched_tracker, aabb)
                                if promoted_obs_data is not None:
                                    state_node.publish_static_obstacle(promoted_obs_data)
                        else:
                            lidar.new_obstacle_detected(aabb, current_time, c_x, c_y, c_z, d_x, d_y, d_z)

                    vis.add_geometry(aabb, reset_bounding_box=False)
                    current_bboxes.append(aabb)

            lidar.clear_obstacles(current_time)

        time.sleep(0.005)

    vis.destroy_window()
    state_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()