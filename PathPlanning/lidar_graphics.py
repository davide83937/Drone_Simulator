from queue import Empty

import numpy as np
import open3d as o3d
import Obstacle as ob



class Graphic_Visualizer:
    def __init__(self, pp):
        self.vis = o3d.visualization.Visualizer()
        self.size = 5000
        self.origin =[0,0,0]
        self.geometries = []
        self.vis.create_window(window_name="Map", width=1280, height=720)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=30.0, origin=[0, 0, 0])
        self.vis.add_geometry(axes)
        self.path_geometries = []
        self.panto_planning = pp


    def clear_path(self):
        for geom in self.path_geometries:
            self.vis.remove_geometry(geom, reset_bounding_box=False)
        self.path_geometries.clear()
        self.refresh()

    def create_poligon(self, obs:ob.ObstacleAABB, color=(0.8, 0.2, 0.2)):
        dx = obs.max_x - obs.min_x
        dy = obs.max_y - obs.min_y
        dz = obs.max_z - obs.min_z

        # 2. Crea il parallelepipedo (in Open3D gli argomenti sono sempre nell'ordine X, Y, Z)
        mesh = o3d.geometry.TriangleMesh.create_box(width=dx, height=dy, depth=dz)

        # 3. Sposta l'origine del box (0,0,0) nel vertice minimo dell'ostacolo
        mesh.translate((obs.min_x, obs.min_y, obs.min_z))
        color = (0.8, 0.2, 0.2)
        # 4. Applica il colore
        mesh.paint_uniform_color(color)

        return mesh

    def draw_point(self, x, y, z, color=(1, 1, 0), radius=2):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.translate((x, y, z))
        sphere.paint_uniform_color(color)

        self.vis.add_geometry(sphere)
        self.vis.update_geometry(sphere)
        self.vis.poll_events()
        self.vis.update_renderer()

        return sphere

    def draw_text_3d(self, text, x, y, z, color=(0.0, 0.0, 0.0), scale=0.5):
        # 1. Crea la mesh 3D del testo (usa l'API tensoriale o3d.t)
        text_mesh_t = o3d.t.geometry.TriangleMesh.create_text(text, depth=0.5)

        # 2. Converte nel formato legacy compatibile con self.vis.add_geometry
        text_mesh = text_mesh_t.to_legacy()

        # 3. Ridimensiona il testo (di default i caratteri sono grandi ~10-15 unità)
        text_mesh.scale(scale, center=(0, 0, 0))

        # 4. Ruota il testo se lo guardi dall'alto (piano XZ, telecamera su +Y)
        # Di default il testo giace sul piano XY; ruotandolo di -90° su X diventa leggibile dall'alto
        R = text_mesh.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
        text_mesh.rotate(R, center=(0, 0, 0))

        # 5. Posiziona e colora (nero = (0.0, 0.0, 0.0))
        text_mesh.translate((x, y, z))
        text_mesh.paint_uniform_color(color)

        self.add_object(text_mesh)
        return text_mesh

    def insert_point_cloud(self, vertexes, point_size=4.0):
        color_free = (0.0, 1.0, 0.0)
        color_blocked = (0.8, 0.2, 0.2)
        # 1. Estrae le coordinate [x, y, z] dai tuoi oggetti vertex in un array NumPy (N, 3)
        coords = np.array([[v.x, v.y, v.z] for v in vertexes], dtype=np.float64)
        # 2. Crea l'array dei colori [r, g, b] in base allo stato v.free di ogni vertice
        colors = np.ascontiguousarray(
            [color_free if v.free else color_blocked for v in vertexes],
            dtype=np.float64
        ).reshape(-1, 3)

        # 2. Crea l'oggetto PointCloud e carica i punti
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # 3. Regola la grandezza dei punti in pixel sullo schermo
        self.vis.get_render_option().point_size = point_size

        # 4. Aggiunge la nuvola di punti alla scena (un solo oggetto, zero lag)
        self.add_object(pcd)
        self.grid_pcd = pcd
        self.grid_coords = coords
        self.grid_colors = colors
        return pcd

    def highlight_grid_points_near(self, center_xyz, radius=10.0, color=(0.0, 0.5, 1.0), reset_others=False):
        """Colora i punti della griglia entro un certo raggio da center_xyz (es. posizione del drone)."""
        if not hasattr(self, "grid_pcd"):
            return
        if reset_others:
            # Ripristina verde/rosso base se vuoi solo evidenziare la zona attuale
            pass

        c = np.asarray(center_xyz, dtype=np.float64).reshape(1, 3)
        dists = np.linalg.norm(self.grid_coords - c, axis=1)
        mask = dists <= radius
        if np.any(mask):
            self.grid_colors[mask] = np.asarray(color, dtype=np.float64)
            self.grid_pcd.colors = o3d.utility.Vector3dVector(self.grid_colors)
            self.vis.update_geometry(self.grid_pcd)

    def insert_path_lines(self, vertex_path, color=(0.0, 0.0, 1.0), line_width=5.0):
        if len(vertex_path) < 2:
            return None

        # 1. Estrae le coordinate [x, y, z] dei punti del percorso
        points = np.array([[v.x, v.y, v.z] for v in vertex_path], dtype=np.float64)

        # 2. Crea le coppie di indici consecutivi: [0, 1], [1, 2], ...
        lines = np.array([[i, i + 1] for i in range(len(vertex_path) - 1)], dtype=np.int32)

        # 3. Assegna il colore RGB a ogni segmento
        colors = np.tile(color, (len(lines), 1)).astype(np.float64)

        # 4. Crea il LineSet
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        # 5. Imposta lo spessore della linea e aggiunge alla scena
        self.vis.get_render_option().line_width = line_width
        self.add_object(line_set)

        return line_set

    def create_sphere(self, x, y, z, radius=1.0, color=(1.0, 1.0, 0.0)):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        mesh.translate((x, y, z))
        mesh.paint_uniform_color(color)
        return mesh

    def insert_spheres(self, points):
        for point in points:
            mesh = self.create_sphere(point.x, point.y, point.z)
            self.add_object(mesh)

    def set_top_down_view(self):
        vc = self.vis.get_view_control()
        # Il vettore 'front' indica dove si trova la telecamera rispetto al target (+Y = dall'alto)
        vc.set_front([0.0, -1.0, 0.0])
        # Il vettore 'up' indica cosa sta in alto sullo schermo (-Z = avanti nella mappa)
        vc.set_up([0.0, -1.0, 0.0])
        # Punto centrale verso cui guarda la telecamera (centro approssimativo dei tuoi ostacoli)
        #vc.set_lookat([-50.0, 15.0, -250.0])
        vc.set_zoom(0.5)

    def update_lidar_points(self, points_np, color=(0.0, 0.0, 0.0)):
        """Aggiorna i punti del LiDAR sostituendo quelli precedenti."""
        if not hasattr(self, "lidar_pcd"):
            self.lidar_pcd = o3d.geometry.PointCloud()
            self.lidar_pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))
            colors = np.tile(color, (len(points_np), 1)).astype(np.float64)
            self.lidar_pcd.colors = o3d.utility.Vector3dVector(colors)
            self.vis.add_geometry(self.lidar_pcd, reset_bounding_box=False)
        else:
            self.lidar_pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))
            colors = np.tile(color, (len(points_np), 1)).astype(np.float64)
            self.lidar_pcd.colors = o3d.utility.Vector3dVector(colors)
            self.vis.update_geometry(self.lidar_pcd)

    def update_point_cloud(self, vertexes, point_size=4.0):
        color_free = (0.0, 1.0, 0.0)
        color_blocked = (0.8, 0.2, 0.2)

        coords = np.array([[v.x, v.y, v.z] for v in vertexes], dtype=np.float64)
        colors = np.ascontiguousarray(
            [color_free if v.free else color_blocked for v in vertexes],
            dtype=np.float64
        ).reshape(-1, 3)

        if not hasattr(self, "grid_pcd") or self.grid_pcd is None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(coords)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            self.vis.get_render_option().point_size = point_size
            self.grid_pcd = pcd
            self.add_object(pcd)
        else:
            self.grid_pcd.points = o3d.utility.Vector3dVector(coords)
            self.grid_pcd.colors = o3d.utility.Vector3dVector(colors)
            self.vis.get_render_option().point_size = point_size
            self.vis.update_geometry(self.grid_pcd)

    def loop(self, queue, change, list_vertex):
        while self.vis.poll_events():
            while True:
                try:
                    lidar_pts = queue.get_nowait()
                    if len(lidar_pts) > 0:
                        self.update_lidar_points(
                            lidar_pts,
                            color=(0.0, 0.0, 0.0)
                        )
                    if change[0]:
                        self.update_point_cloud(list_vertex)
                        change[0] = False
                    if self.panto_planning.change_path[0]:
                        self.clear_path()

                        line_set = self.insert_path_lines(self.panto_planning.list_vertex_path, color=(1.0, 0.0, 0.0), line_width=5.0)
                        if line_set is not None:
                            self.path_geometries.append(line_set)

                        for idx, v in enumerate(self.panto_planning.list_vertex_path):
                            sphere = self.create_sphere(v.x, v.y, v.z, radius=1.0, color=(1.0, 0.0, 0.0))
                            self.add_object(sphere)
                            self.path_geometries.append(sphere)

                            text_mesh = self.draw_text_3d(str(idx), v.x + 3, v.y, v.z, color=(1.0, 0.0, 0.0), scale=0.1)
                            self.path_geometries.append(text_mesh)
                        self.panto_planning.change_path[0]=False

                except Empty:
                    break
            self.vis.update_renderer()
        self.close()


    def add_object(self, mesh):
        self.vis.add_geometry(mesh)
        self.refresh()

    def insert_poligons(self, obstacles):
        for obstacle in obstacles:
            mesh = self.create_poligon(obstacle)
            self.add_object(mesh)

    def refresh(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def close(self):
        self.vis.destroy_window()

    def open_scene(self):
        self.vis.run()
        self.vis.close()





