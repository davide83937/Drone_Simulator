# --- 1. CONFIGURAZIONE AMBIENTE ROS 2 (DEVE ESSERE LA PRIMA COSA IN ABSOLUTE) ---
import os

from PathPlanning.lidar_processor import Lidar_Processor

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
os.environ["AMENT_PREFIX_PATH"] = prefix_path  # Usa os.environ invece di setdefault per forzarlo
# -----------------------------------------------------------------------------


import threading

import numpy as np
from rclpy.executors import MultiThreadedExecutor
import rclpy

from PathPlanning.PantoPlanning import PantoPlanning, vertex
from PathPlanning.WaypointSender import WaypointSender
from PathPlanning.getObstacles import get_obstacles_path_manager
from PathPlanning.lidar_graphics import Graphic_Visualizer
from PathPlanning.lidar_manager import Lidar_Server


class PathManager:
    def __init__(self):
        self.vertexes = []
        self.min_bound_x = -300
        self.max_bound_x = 200
        self.min_bound_y = -500
        self.max_bound_y = 100
        self.step = 5



    def fill_vertexes_list(self):
        for i in range(self.min_bound_x, self.max_bound_x, self.step):
            for j in range(self.min_bound_y, self.max_bound_y, self.step):
                self.vertexes.append(vertex(i, 0,j))

    def check_vertexes(self):
        for vertex in self.vertexes:
            for obstacle in get_obstacles_path_manager():
                if vertex.x >= obstacle.min_x-10 and vertex.x <= obstacle.max_x+10 and vertex.z >= obstacle.min_z-10 and vertex.z <= obstacle.max_z+10:
                    #print("Vertice bloccato")
                    #print(f"Vertici: {vertex.x}, {vertex.y}, {vertex.z}")
                    #print(f"Obstacle min: {obstacle.min_x}, {obstacle.min_y}, {obstacle.min_z}")
                    #print(f"Obstacle max: {obstacle.max_x}, {obstacle.max_y}, {obstacle.max_z}")
                    #print("\n")
                    vertex.free = False

    def update_vertex_list_cylinder(self, cylinder: dict, margin: float = 10.0):
        """
        Marca come non liberi (free = False) i vertici della griglia
        che cadono dentro il cerchio (center_xz, radius) + margine di sicurezza.
        Restituisce la lista dei vertici appena bloccati.
        """
        cx, cz = cylinder["center_xz"]
        max_dist = float(cylinder["radius"]) + float(margin)
        max_dist_sq = max_dist ** 2

        change = False
        for vertex in self.vertexes:
            if vertex.free:
                dx = vertex.x - cx
                dz = vertex.z - cz
                # Confrontiamo i quadrati per evitare la radice quadrata a ogni punto
                if (dx * dx + dz * dz) <= max_dist_sq:
                    vertex.free = False
                    change = True
        return change



    def update_vertex_list(self, new_obstacle):
        for vertex in self.vertexes:
            if vertex.free:
                    if vertex.x >= new_obstacle.min_x-10 and vertex.x <= new_obstacle.max_x+10 and vertex.z >= new_obstacle.min_z-10 and vertex.z <= new_obstacle.max_z+10:
                        vertex.free = False


    def search_vertexes(self, x, z):
        v = None
        for vertex in self.vertexes:
            if vertex.x == x and vertex.z == z:
                return vertex
        return v

    def increment_step_positive_x(self, current:vertex):
        next_x = current.x + self.step
        next_vertex = self.search_vertexes(next_x, current.z)
        return next_vertex

    def increment_step_negative_x(self, current:vertex):
        next_x = current.x - self.step
        next_vertex = self.search_vertexes(next_x, current.z)
        return next_vertex

    def increment_step_positive_z(self, current:vertex):
        next_z = current.z + self.step
        next_vertex = self.search_vertexes(current.x, next_z)
        return next_vertex

    def increment_step_negative_z(self, current:vertex):
        next_z = current.z - self.step
        next_vertex = self.search_vertexes(current.x, next_z)
        return next_vertex





if __name__ == "__main__":
    pm = PathManager()
    start = vertex(0, 0, 0)
    end = vertex(25, 0, -405)
    pp = PantoPlanning(pm, start, end)
    pm.fill_vertexes_list()
    pm.check_vertexes()
    gv = Graphic_Visualizer(pp)
    gv.insert_poligons(get_obstacles_path_manager())
    pp.run()
    pcd = gv.insert_point_cloud(pm.vertexes, point_size=2.0)
    gv.insert_path_lines(pp.list_vertex_path, color=(0.0, 0.0, 1.0), line_width=5.0)
    gv.set_top_down_view()
    print(f"LUNGHEZZA: {len(pp.list_vertex_path)}")
    i = 0
    for v in pp.list_vertex_path:
        gv.add_object(gv.create_sphere(v.x, v.y, v.z, radius=1, color=(0.0, 0.0, 0.0)))
        gv.draw_text_3d(str(i), v.x+3, v.y, v.z, color=(0.0, 0.0, 0.0), scale=0.1)
        #print(i)
        i=i+1


    #gv.select_points(pm.vertexes)
    #gv.set_top_down_view()
    rclpy.init()
    sender = WaypointSender(pp)

    lidar_processor = Lidar_Processor()
    lidar_server = Lidar_Server(sender, lidar_processor, pm)
    lidar_server.start()

    executor = MultiThreadedExecutor()
    executor.add_node(sender)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    sender.send_next_waypoint()

    gv.loop(lidar_server.my_queue, lidar_server.change, pm.vertexes)
    gv.set_top_down_view()
