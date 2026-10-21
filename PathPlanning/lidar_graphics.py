import numpy as np
import open3d as o3d
import math


def create_drone_mesh(scale=1.2):
    drone = o3d.geometry.TriangleMesh()

    # 1. Corpo centrale (fusoliera)
    body = o3d.geometry.TriangleMesh.create_box(width=1.6 * scale, height=2.2 * scale, depth=0.5 * scale)
    body.translate((-0.8 * scale, -1.1 * scale, -0.25 * scale))
    body.paint_uniform_color([0.25, 0.25, 0.3])
    drone += body

    # 2. Bracci e Rotori (4 eliche agli angoli)
    arm_Offsets = [
        (-1.8 * scale, -1.8 * scale, [1.0, 0.25, 0.0]),  # Anteriore SX (Arancione/Rosso)
        ( 1.8 * scale, -1.8 * scale, [1.0, 0.25, 0.0]),  # Anteriore DX (Arancione/Rosso)
        (-1.8 * scale,  1.8 * scale, [0.0, 0.85, 1.0]),  # Posteriore SX (Ciano)
        ( 1.8 * scale,  1.8 * scale, [0.0, 0.85, 1.0]),  # Posteriore DX (Ciano)
    ]

    for rx, ry, rotor_color in arm_Offsets:
        # Braccio dal centro al rotore
        arm = o3d.geometry.TriangleMesh.create_box(width=0.25 * scale, height=2.6 * scale, depth=0.2 * scale)
        arm.translate((-0.125 * scale, -1.3 * scale, -0.1 * scale))
        angle = math.atan2(rx, ry)
        R_arm = arm.get_rotation_matrix_from_xyz((0, 0, -angle))
        arm.rotate(R_arm, center=(0, 0, 0))
        arm.translate((rx / 2.0, ry / 2.0, 0))
        arm.paint_uniform_color([0.5, 0.5, 0.55])
        drone += arm

        # Disco del rotore
        rotor = o3d.geometry.TriangleMesh.create_cylinder(radius=0.95 * scale, height=0.15 * scale, resolution=20)
        rotor.translate((rx, ry, 0.2 * scale))
        rotor.paint_uniform_color(rotor_color)
        drone += rotor

    # 3. Freccia direzionale anteriore (punta verso -Y quando Yaw=0)
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.2 * scale,
        cone_radius=0.45 * scale,
        cylinder_height=1.8 * scale,
        cone_height=0.9 * scale
    )
    # Di default la freccia punta su +Z; la ruotiamo di +90° attorno a X per puntare verso -Y
    R_arrow = arrow.get_rotation_matrix_from_xyz((math.pi / 2.0, 0, 0))
    arrow.rotate(R_arrow, center=(0, 0, 0))
    arrow.translate((0, -1.0 * scale, 0.35 * scale))
    arrow.paint_uniform_color([1.0, 0.9, 0.0])  # Giallo acceso
    drone += arrow

    drone.compute_vertex_normals()
    return drone


# --- GENERATORE SCRITTA 3D "New Obstacle Detected" (VISTA DALL'ALTO) ---
# --- GENERATORE SCRITTA 3D ROSSO BOLD (VISTA DALL'ALTO) ---
def create_3d_text_label(text, position, scale=2.2, thickness=0.65, color=(1.0, 0.0, 0.0)):
    """
    Crea una scritta 3D piena e Bold sul piano XY (leggibile dall'alto)
    usando parallelepipedi 3D continui invece di linee tratteggiate.
    """
    strokes = {
        'N': [((0,0),(0,2)), ((0,2),(1,0)), ((1,0),(1,2))],
        'E': [((0,0),(0,2)), ((0,2),(1,2)), ((0,1),(0.85,1)), ((0,0),(1,0))],
        'W': [((0,2),(0.2,0)), ((0.2,0),(0.5,1.1)), ((0.5,1.1),(0.8,0)), ((0.8,0),(1,2))],
        'O': [((0,0),(0,2)), ((0,2),(1,2)), ((1,2),(1,0)), ((1,0),(0,0))],
        'B': [((0,0),(0,2)), ((0,2),(0.85,2)), ((0.85,2),(1,1.5)), ((1,1.5),(0.85,1)),
              ((0,1),(0.85,1)), ((0.85,1),(1,0.5)), ((1,0.5),(0.85,0)), ((0.85,0),(0,0))],
        'S': [((1,2),(0.1,2)), ((0.1,2),(0,1.5)), ((0,1.5),(0.1,1)), ((0.1,1),(0.9,1)),
              ((0.9,1),(1,0.5)), ((1,0.5),(0.9,0)), ((0.9,0),(0,0))],
        'T': [((0,2),(1,2)), ((0.5,2),(0.5,0))],
        'A': [((0,0),(0.5,2)), ((0.5,2),(1,0)), ((0.2,0.75),(0.8,0.75))],
        'C': [((1,2),(0,2)), ((0,2),(0,0)), ((0,0),(1,0))],
        'L': [((0,2),(0,0)), ((0,0),(1,0))],
        'D': [((0,0),(0,2)), ((0,2),(0.75,2)), ((0.75,2),(1,1.4)),
              ((1,1.4),(1,0.6)), ((1,0.6),(0.75,0)), ((0.75,0),(0,0))],
    }

    text_mesh = o3d.geometry.TriangleMesh()
    lines = text.upper().split('\n')
    char_step = 1.55 * scale
    line_step = 2.8 * scale
    z_depth = 0.8  # Spessore verticale del testo lungo Z

    for row_idx, line in enumerate(lines):
        total_w = len(line) * char_step
        start_x = position[0] - (total_w / 2.0)
        # Dispone eventuali righe multiple dall'alto (+Y) verso il basso (-Y)
        base_y = position[1] + ((len(lines) - 1) / 2.0 - row_idx) * line_step

        for col_idx, ch in enumerate(line):
            if ch not in strokes:
                continue
            x_off = start_x + col_idx * char_step

            for (x1, y1), (x2, y2) in strokes[ch]:
                p1_x = x_off + x1 * scale
                p1_y = base_y + y1 * scale
                p2_x = x_off + x2 * scale
                p2_y = base_y + y2 * scale

                dx = p2_x - p1_x
                dy = p2_y - p1_y
                length = math.hypot(dx, dy) + thickness  # Sovrappone gli spigoli senza buchi
                angle = math.atan2(dy, dx)

                bar = o3d.geometry.TriangleMesh.create_box(width=length, height=thickness, depth=z_depth)
                bar.translate((-length / 2.0, -thickness / 2.0, -z_depth / 2.0))
                R = bar.get_rotation_matrix_from_xyz((0.0, 0.0, angle))
                bar.rotate(R, center=(0.0, 0.0, 0.0))
                bar.translate(((p1_x + p2_x) / 2.0, (p1_y + p2_y) / 2.0, position[2]))

                text_mesh += bar

    # NON calcoliamo le normali: così Open3D lo disegna in ROSSO PURO piatto (sempre luminoso)
    text_mesh.paint_uniform_color(color)
    return text_mesh


def _create_fallback_lineset_text(text, position, scale=0.8, color=(1.0, 0.85, 0.0)):
    strokes = {
        'N': [((0,0),(0,2)), ((0,2),(1,0)), ((1,0),(1,2))],
        'E': [((1,2),(0,2)), ((0,2),(0,0)), ((0,0),(1,0)), ((0,1),(0.8,1))],
        'W': [((0,2),(0.2,0)), ((0.2,0),(0.5,1)), ((0.5,1),(0.8,0)), ((0.8,0),(1,2))],
        'O': [((0,0),(0,2)), ((0,2),(1,2)), ((1,2),(1,0)), ((1,0),(0,0))],
        'B': [((0,0),(0,2)), ((0,2),(0.9,2)), ((0.9,2),(0.9,1)), ((0,1),(0.9,1)), ((0.9,1),(1,0)), ((1,0),(0,0))],
        'S': [((1,2),(0,2)), ((0,2),(0,1)), ((0,1),(1,1)), ((1,1),(1,0)), ((1,0),(0,0))],
        'T': [((0,2),(1,2)), ((0.5,2),(0.5,0))],
        'A': [((0,0),(0.5,2)), ((0.5,2),(1,0)), ((0.2,0.8),(0.8,0.8))],
        'C': [((1,2),(0,2)), ((0,2),(0,0)), ((0,0),(1,0))],
        'L': [((0,2),(0,0)), ((0,0),(1,0))],
        'D': [((0,0),(0,2)), ((0,2),(0.8,2)), ((0.8,2),(1,1)), ((1,1),(0.8,0)), ((0.8,0),(0,0))],
    }
    points = []
    lines = []
    total_w = len(text) * 1.4 * scale
    start_x = position[0] - total_w / 2.0

    for idx, ch in enumerate(text):
        if ch not in strokes:
            continue
        x_off = start_x + idx * 1.4 * scale
        for (x1, y1), (x2, y2) in strokes[ch]:
            # Sviluppa l'altezza dei caratteri lungo l'asse Y anziché Z (piatta vista dall'alto)
            p1 = [x_off + x1 * scale, position[1] + y1 * scale, position[2]]
            p2 = [x_off + x2 * scale, position[1] + y2 * scale, position[2]]
            points.extend([p1, p2])
            lines.append([len(points) - 2, len(points) - 1])

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.paint_uniform_color(color)
    return ls


def start_window():
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Godot LiDAR Viewer", width=900, height=700)
    opt = vis.get_render_option()
    opt.point_size = 5.0
    opt.background_color = np.asarray([0.1, 0.1, 0.1])
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))
    return vis, pcd

def render_drone(vis):
    drone_mesh = create_drone_mesh(scale=1.2)
    base_drone_vertices = np.asarray(drone_mesh.vertices).copy()
    vis.add_geometry(drone_mesh)
    return drone_mesh, base_drone_vertices

def update_drone_position(base_drone_vertices, drone_mesh, R_yaw, state_node, vis):
    transformed_verts = base_drone_vertices.dot(R_yaw.T) + state_node.drone_pos
    drone_mesh.vertices = o3d.utility.Vector3dVector(transformed_verts)
    vis.update_geometry(drone_mesh)

def render_obstacles(static_obstacles, vis):
    for obs in static_obstacles:
        min_b = np.array([obs.min_x, obs.min_y, obs.min_z])
        max_b = np.array([obs.max_x, obs.max_y, obs.max_z])
        box_expanded = o3d.geometry.AxisAlignedBoundingBox(min_b, max_b)
        box_expanded.color = (0.0, 0.4, 1.0)
        vis.add_geometry(box_expanded)

def render_new_obstacles(state_node, vis):
    # --- DISEGNA "NEW OBSTACLE DETECTED" SOLO SUGLI OSTACOLI CHE INTRALCIANO ---
    while state_node.blocking_obstacles_queue:
        b_obs = state_node.blocking_obstacles_queue.pop(0)
        bx, by, bz = b_obs['x'], b_obs['y'], b_obs['z']
        bw, bd, bh = b_obs['w'], b_obs['d'], b_obs['h']

        # 1. Bounding box dell'ostacolo bloccante in GIALLO
        alert_min = np.array([bx - bw / 2.0 - 0.4, by - bd / 2.0 - 0.4, bz - bh / 2.0 - 0.4])
        alert_max = np.array([bx + bw / 2.0 + 0.4, by + bd / 2.0 + 0.4, bz + bh / 2.0 + 0.4])
        alert_box = o3d.geometry.AxisAlignedBoundingBox(alert_min, alert_max)
        alert_box.color = (1.0, 0.85, 0.0)  # Giallo
        vis.add_geometry(alert_box, reset_bounding_box=False)

        # 2. Scritta 3D ROSSO BOLD sospesa sopra l'ostacolo
        label_pos = np.array([bx, by, bz + (bh / 2.0) + 5.0])
        text_geom = create_3d_text_label(
            "NEW OBSTACLE\nDETECTED",
            label_pos,
            scale=2.2,  # Grandezza dei caratteri
            thickness=0.70,  # Spessore del Bold
            color=(1.0, 0.0, 0.0)  # Rosso acceso
        )
        vis.add_geometry(text_geom, reset_bounding_box=False)
        print(f"[ALERT 3D] Scritta ROSSO BOLD aggiunta sopra X:{bx:.1f}, Y:{by:.1f}")

    vis.poll_events()
    vis.update_renderer()

def render_points(vis, pcd, accumulated_points, first_points_received):
    pcd.points = o3d.utility.Vector3dVector(accumulated_points)
    pcd.paint_uniform_color([0.0, 1.0, 0.0])
    vis.update_geometry(pcd)
    if not first_points_received and len(accumulated_points) > 0:
        vis.reset_view_point(True)
        first_points_received = True
    return first_points_received


def clear_points(current_bboxes, vis):
    for bbox in current_bboxes:
        vis.remove_geometry(bbox, reset_bounding_box=False)
    current_bboxes.clear()