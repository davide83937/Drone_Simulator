import os
import socket
import struct
import numpy as np
import open3d as o3d
import time

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


# --- NODO ROS 2 PER POSIZIONE E ROTAZIONE ---
class DroneStateSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_viewer_state_sub')
        self.create_subscription(Vector3, 'drone_position', self.pos_callback, 10)

        # NUOVO SOTTOSCRITTORE PER LA ROTAZIONE
        self.create_subscription(Vector3, 'drone_rotation', self.rot_callback, 10)

        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_yaw = 0.0  # Memorizziamo lo yaw in radianti

    def pos_callback(self, msg):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])

    def rot_callback(self, msg):
        # Assumiamo che lo yaw sia pubblicato in msg.z (convenzione standard ROS)
        self.drone_yaw = msg.z
        self.drone_yaw = np.radians(msg.z)



# --- CONFIGURAZIONE LIDAR UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 9999
MAX_POINTS = 30000


def main():
    rclpy.init()
    state_node = DroneStateSubscriber()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Godot LiDAR Viewer", width=800, height=600)

    opt = vis.get_render_option()
    opt.point_size = 5.0
    opt.background_color = np.asarray([0.1, 0.1, 0.1])

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

    drone_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.5)
    drone_sphere.paint_uniform_color([1.0, 0.0, 0.0])
    vis.add_geometry(drone_sphere)
    current_sphere_pos = np.array([0.0, 0.0, 0.0])

    accumulated_points = np.zeros((0, 3))
    first_points_received = False
    last_print_time = time.time()

    current_bboxes = []
    last_cluster_time = time.time()

    print(f"In ascolto su {UDP_IP}:{UDP_PORT} per il LiDAR...")

    while True:
        rclpy.spin_once(state_node, timeout_sec=0.0)
        current_time = time.time()

        if current_time - last_print_time >= 1.0:
            # Stampa anche l'angolo per debug (convertito in gradi)
            deg_yaw = np.degrees(state_node.drone_yaw)
            print(
                f"[DRONE] Pos: X:{state_node.drone_pos[0]:.2f} | Y:{state_node.drone_pos[1]:.2f} | Z:{state_node.drone_pos[2]:.2f} || Yaw: {deg_yaw:.1f}°")
            last_print_time = current_time

        vis.poll_events()
        vis.update_renderer()

        translation = state_node.drone_pos - current_sphere_pos
        if np.linalg.norm(translation) > 0.001:
            drone_sphere.translate(translation)
            current_sphere_pos = np.copy(state_node.drone_pos)
            vis.update_geometry(drone_sphere)

        try:
            data, addr = sock.recvfrom(65535)
            num_floats = len(data) // 4

            if num_floats % 3 == 0:
                floats = struct.unpack(f'<{num_floats}f', data)
                local_points = np.array(floats).reshape(-1, 3)

                # --- 1. APPLICA LA MATRICE DI ROTAZIONE (YAW) ---
                theta = state_node.drone_yaw
                cos_t = np.cos(theta)
                sin_t = np.sin(theta)

                # Matrice di rotazione attorno all'asse verticale (Y nel nostro visualizzatore)
                R_yaw = np.array([
                    [cos_t, -sin_t, 0],
                    [sin_t, cos_t, 0],
                    [0, 0, 1]
                ])

                # Ruotiamo i punti locali
                rotated_points = local_points.dot(R_yaw.T)

                # --- 2. TRASFORMA IN ASSOLUTO (Somma la posizione) ---
                absolute_points = rotated_points + state_node.drone_pos

                accumulated_points = np.vstack((accumulated_points, absolute_points))

                if len(accumulated_points) > MAX_POINTS:
                    accumulated_points = accumulated_points[-MAX_POINTS:]

                pcd.points = o3d.utility.Vector3dVector(accumulated_points)
                pcd.paint_uniform_color([0.0, 1.0, 0.0])
                vis.update_geometry(pcd)

                if not first_points_received and len(accumulated_points) > 0:
                    vis.reset_view_point(True)
                    first_points_received = True

        except BlockingIOError:
            pass
        except KeyboardInterrupt:
            break
        except Exception as e:
            pass

        if current_time - last_cluster_time >= 1.0 and len(np.asarray(pcd.points)) > 50:
            last_cluster_time = current_time

            for bbox in current_bboxes:
                vis.remove_geometry(bbox, reset_bounding_box=False)
            current_bboxes.clear()

            downpcd = pcd.voxel_down_sample(voxel_size=0.5)
            labels = np.array(downpcd.cluster_dbscan(eps=4.5, min_points=30, print_progress=False))

            if len(labels) > 0:
                max_label = labels.max()
                print(f"\n[VISIONE] Trovati {max_label + 1} ostacoli!")

                for i in range(max_label + 1):
                    cluster_indices = np.where(labels == i)[0]
                    cluster_pcd = downpcd.select_by_index(cluster_indices)

                    aabb = cluster_pcd.get_axis_aligned_bounding_box()
                    aabb.color = (1, 0, 0)

                    center = aabb.get_center()
                    extent = aabb.get_extent()

                    c_x, c_y, c_z = center[0], center[1], center[2]
                    d_x, d_y, d_z = extent[0], extent[1], extent[2]

                    print(
                        f"  -> Ostacolo {i}: Centro X:{c_x:.1f}, Y:{c_y:.1f}, Z:{c_z:.1f} | Dim: {d_x:.1f}x{d_y:.1f}x{d_z:.1f}m")

                    vis.add_geometry(aabb, reset_bounding_box=False)
                    current_bboxes.append(aabb)

        time.sleep(0.005)

    vis.destroy_window()
    state_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()