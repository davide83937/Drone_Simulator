import socket

import numpy as np

from PathPlanning.PantoPlanning import PantoPlanning, vertex
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
    end = vertex(0, 0, -380)
    pp = PantoPlanning(pm, start, end)
    pm.fill_vertexes_list()
    pm.check_vertexes()
    gv = Graphic_Visualizer()

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
    gv.set_top_down_view()
    Lidar_Server = Lidar_Server()
    Lidar_Server.start()
    gv.loop(Lidar_Server.my_queue)