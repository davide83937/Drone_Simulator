from PathPlanning.Obstacle import ObstacleAABB

# --- COSTANTI GLOBALI ---
# Quote e altezze fisse
PATH_Y = 20.0          # Quota costante del drone per Dijkstra
LIDAR_Z = 35.0         # Quota costante per il LiDAR
HEIGHT_OBSTACLE = 60.0 # Altezza verticale dei muri (Y per Dijkstra, Z per Lidar)
DRONE_RADIUS = 5.0

# --- LISTA MASTER DEGLI OSTACOLI ---
# Questa è l'unica lista che dovrai modificare.
# x, z: coordinate al suolo
# dim_x: larghezza sull'asse orizzontale
# dim_z: profondità sull'asse Z
MASTER_OBSTACLES = [
    {"desc": "Dritto 1",       "x": -46.0,  "z": -75.0,  "dim_x": 20.0,  "dim_z": 200.0},
    {"desc": "Rovesciato 2",   "x": -126.0, "z": -173.0, "dim_x": 200.0, "dim_z": 20.0},
    {"desc": "Dritto 3",       "x": 45.0,   "z": -75.0,  "dim_x": 20.0,  "dim_z": 200.0},
    {"desc": "Rovesciato 4",   "x": -68.0,  "z": -233.0, "dim_x": 200.0, "dim_z": 20.0},
    {"desc": "Dritto 5",       "x": -243.0, "z": -269.0, "dim_x": 20.0,  "dim_z": 200.0},
    {"desc": "Rovesciato 6",   "x": -151.0, "z": -390.0, "dim_x": 200.0, "dim_z": 20.0},
    {"desc": "Rovesciato 7",   "x": -73.0,  "z": -315.0, "dim_x": 200.0, "dim_z": 20.0},
    {"desc": "Dritto 8",       "x": -161.0, "z": -270.0, "dim_x": 20.0,  "dim_z": 60.0},
    {"desc": "Rovesciato 9",   "x": 84.0,   "z": -160.0, "dim_x": 60.0,  "dim_z": 20.0},
    {"desc": "Dritto 10",      "x": 104.0,  "z": -202.0, "dim_x": 20.0,  "dim_z": 60.0},
    {"desc": "Rovesciato 11",  "x": 63.0,   "z": -239.0, "dim_x": 60.0,  "dim_z": 20.0},
    {"desc": "Dritto 12",      "x": -48.0,  "z": -427.0, "dim_x": 20.0,  "dim_z": 60.0},
    {"desc": "Rovesciato 13",  "x": -9.0,   "z": -458.0, "dim_x": 60.0,  "dim_z": 20.0},
    {"desc": "Rovesciato 14",  "x": 60.0,   "z": -458.0, "dim_x": 60.0,  "dim_z": 20.0},
    {"desc": "Dritto 15",      "x": 104.0,  "z": -427.0, "dim_x": 20.0,  "dim_z": 60.0},
    {"desc": "Dritto 16",      "x": 104.0,  "z": -356.0, "dim_x": 20.0,  "dim_z": 60.0},
    {"desc": "Rovesciato 17",  "x": 67.0,   "z": -315.0, "dim_x": 60.0,  "dim_z": 20.0},
]


def get_obstacles_path_manager():
    """Genera ostacoli per Dijkstra (Altezza = asse Y, Profondità = asse Z)"""
    obstacles = []
    for obs in MASTER_OBSTACLES:
        obstacles.append(ObstacleAABB(
            x=obs["x"],
            y=PATH_Y,               # Quota fissa (es. 20.0)
            z=obs["z"],
            width=obs["dim_x"],     # Dimensione su asse X
            depth=HEIGHT_OBSTACLE,  # Dijkstra usa 'depth' per l'altezza Y (60.0)
            height=obs["dim_z"],    # Dijkstra usa 'height' per la profondità Z
            drone_radius=DRONE_RADIUS
        ))
    return obstacles


def get_obstacles_lidar_manager():
    """Genera ostacoli per il Lidar (Altezza = asse Z, Profondità = asse Y)"""
    obstacles = []
    for obs in MASTER_OBSTACLES:
        obstacles.append(ObstacleAABB(
            x=obs["x"],
            y=obs["z"],             # Lidar inverte Y e Z: Y diventa la profondità
            z=LIDAR_Z,              # Quota fissa del Lidar (es. 35.0)
            width=obs["dim_x"],     # Dimensione su asse X
            depth=obs["dim_z"],     # Lidar usa 'depth' per la profondità reale al suolo
            height=HEIGHT_OBSTACLE, # Lidar usa 'height' per l'altezza Z (60.0)
            drone_radius=DRONE_RADIUS
        ))
    return obstacles