import math
import heapq
from obstacle_config import ObstacleConfig


class ObstacleAABB:
    def __init__(self, x, y, z, width, depth, height, drone_radius=None):
        if drone_radius is None:
            drone_radius = ObstacleConfig.get_default_aabb_drone_radius()
        self.min_x = x - (width / 2) - drone_radius
        self.max_x = x + (width / 2) + drone_radius
        self.min_y = y - (depth / 2) - drone_radius
        self.max_y = y + (depth / 2) + drone_radius
        self.min_z = z - (height / 2) - drone_radius
        self.max_z = z + (height / 2) + drone_radius

    def is_inside(self, px, py, pz):
        return (self.min_x <= px <= self.max_x and
                self.min_y <= py <= self.max_y and
                self.min_z <= pz <= self.max_z)

    def overlaps_with_bounds(self, b_min_x, b_max_x, b_min_y, b_max_y, b_min_z, b_max_z):
        """Restituisce True se i limiti forniti si sovrappongono a questo ostacolo"""
        return not (self.max_x < b_min_x or self.min_x > b_max_x or
                    self.max_y < b_min_y or self.max_y > b_max_y or
                    self.max_z < b_min_z or self.min_z > b_max_z)


class PathPlanner:
    def __init__(self, bounds, resolution, obstacles):
        self.resolution = resolution
        self.obstacles = obstacles
        self.bounds = bounds

    def _round_to_grid(self, val):
        return round(round(val / self.resolution) * self.resolution, 3)

    def is_valid(self, x, y, z):
        if not (self.bounds['x'][0] <= x <= self.bounds['x'][1] and
                self.bounds['y'][0] <= y <= self.bounds['y'][1] and
                self.bounds['z'][0] <= z <= self.bounds['z'][1]):
            return False
        for obs in self.obstacles:
            if obs.is_inside(x, y, z):
                return False
        return True

    def is_segment_valid(self, x1, y1, z1, x2, y2, z2):
        """Controlla punti intermedi lungo il segmento adattando i controlli alla lunghezza"""
        seg_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        if seg_len == 0:
            return True

        num_steps = max(
            ObstacleConfig.get_segment_check_min_steps(),
            int(seg_len / ObstacleConfig.get_segment_check_step_meters())
        )  # 1 controllo ogni metro

        for step in range(1, num_steps):
            t = step / float(num_steps)
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            iz = z1 + t * (z2 - z1)
            if not self.is_valid(ix, iy, iz):
                return False
        return True

    def decouple_path(self, initial_yaw, raw_positions):
        decoupled_path = []
        current_yaw = initial_yaw

        for i in range(len(raw_positions) - 1):
            curr_pos = raw_positions[i]
            next_pos = raw_positions[i + 1]

            # Calcoliamo le differenze sui 3 assi
            dx = next_pos[0] - curr_pos[0]
            dy = next_pos[1] - curr_pos[1]
            dz = -(next_pos[2] - curr_pos[2])  # Inversione Z per il calcolo dell'angolo

            # ---------------------------------------------------------
            # 1. DISACCOPPIAMENTO ROTAZIONE (Fermo sul posto e ruota)
            # ---------------------------------------------------------
            if abs(dx) > 0.01 or abs(dz) > 0.01:
                desired_yaw = math.atan2(dx, dz)
                diff_yaw = abs((desired_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi)

                if diff_yaw > math.radians(1.0):
                    # Inserisce un waypoint con coordinate vecchie, ma angolo nuovo
                    decoupled_path.append((*curr_pos, desired_yaw))
                    current_yaw = desired_yaw

            # ---------------------------------------------------------
            # 2. DISACCOPPIAMENTO VERTICALE / ORIZZONTALE
            # ---------------------------------------------------------
            has_horiz = (abs(dx) > 0.01 or abs(dz) > 0.01)
            has_vert = (abs(dy) > 0.01)

            if has_horiz and has_vert:
                # Se deve fare ENTRAMBI, li spezziamo in due movimenti.
                # STEP A: Cambia solo la quota (Y), mantenendo X e Z attuali (Ascensore)
                intermediate_pos = (curr_pos[0], next_pos[1], curr_pos[2])
                decoupled_path.append((*intermediate_pos, current_yaw))

                # STEP B: Adesso che è all'altezza giusta, si sposta in orizzontale (X, Z)
                decoupled_path.append((*next_pos, current_yaw))
            else:
                # Se è un movimento puro (o solo su/giù, o solo orizzontale), basta un passo
                decoupled_path.append((*next_pos, current_yaw))

        return decoupled_path

    def get_neighbors(self, x, y, z):
        neighbors = []

        # Sostituiamo i 3 cicli 'for' annidati definendo a priori le sole direzioni valide.
        # Questo elimina automaticamente i controlli "tutto zero" e i movimenti sporchi.
        valid_directions = [
            (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1),  # Orizzontali dritti
            (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),  # Diagonali orizzontali
            (0, 1, 0), (0, -1, 0)  # Verticali puri
        ]

        for dx, dy, dz in valid_directions:
            nx = self._round_to_grid(x + dx * self.resolution)
            ny = self._round_to_grid(y + dy * self.resolution)
            nz = self._round_to_grid(z + dz * self.resolution)

            # 1. Controlla che la cella finale sia valida
            if not self.is_valid(nx, ny, nz):
                continue

            # 2. Controlla che il segmento lineare non attraversi muri
            if not self.is_segment_valid(x, y, z, nx, ny, nz):
                continue

            # 3. Nelle diagonali orizzontali, impedisci di tagliare gli spigoli
            is_diagonal = (dx != 0 and dz != 0)
            if is_diagonal and not (self.is_valid(nx, y, z) and self.is_valid(x, y, nz)):
                continue
            #Se un cubo vicino supera i controlli anti-collisione, la funzione
            # ne calcola la distanza lineare usando il Teorema di Pitagora
            # esteso a tre dimensioni
            cost = math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2)
            neighbors.append(((nx, ny, nz), cost))

        return neighbors

    def smooth_path(self, raw_positions):
        """String Pulling: unisce i piccoli passi in lunghe linee rette dove non ci sono ostacoli"""
        if len(raw_positions) <= 2:
            return raw_positions

        smoothed = [raw_positions[0]]
        current_idx = 0

        while current_idx < len(raw_positions) - 1:
            furthest_valid = current_idx + 1

            # Prova a guardare sempre più avanti lungo la rotta
            for test_idx in range(current_idx + 2, len(raw_positions)):
                p_curr = raw_positions[current_idx]
                p_test = raw_positions[test_idx]

                # Spara un raggio dal punto attuale al punto futuro lontano. È libero?
                if self.is_segment_valid(p_curr[0], p_curr[1], p_curr[2],
                                         p_test[0], p_test[1], p_test[2]):
                    furthest_valid = test_idx
                else:
                    # Appena il raggio colpisce un ostacolo, non possiamo più "tagliare" dritto.
                    # Ci fermiamo e il waypoint precedente diventa la nuova tappa obbligatoria.
                    break

            smoothed.append(raw_positions[furthest_valid])
            current_idx = furthest_valid

        return smoothed

    def dijkstra(self, start, goal, initial_yaw=0.0):
        real_start = (float(start[0]), float(start[1]), float(start[2]))

        # Usiamo una "Generator Expression" per applicare l'arrotondamento in una sola riga
        grid_start = tuple(self._round_to_grid(coord) for coord in start)
        grid_goal = tuple(self._round_to_grid(coord) for coord in goal)

        if not self.is_valid(*grid_start) or not self.is_valid(*grid_goal):
            print(f"ERRORE: Start {grid_start} o Goal {grid_goal} non validi (fuori mappa o dentro ostacolo).")
            return []

        queue = [(0.0, grid_start)]
        distances = {grid_start: 0.0}
        came_from = {grid_start: None}
        raw_positions = []

        while queue:
            current_dist, current_node = heapq.heappop(queue)

            if current_node == grid_goal:
                # Rimuoviamo la variabile superflua 'curr' eseguendo la risalita direttamente
                while current_node is not None:
                    raw_positions.append(current_node)
                    current_node = came_from[current_node]
                raw_positions.reverse()
                break

            for next_node, weight in self.get_neighbors(*current_node):
                new_dist = current_dist + weight
                if next_node not in distances or new_dist < distances[next_node]:
                    distances[next_node] = new_dist
                    came_from[next_node] = current_node
                    heapq.heappush(queue, (new_dist, next_node))

        if not raw_positions:
            return []

        raw_positions[0] = real_start

        # =====================================================================
        # 1. OTTIMIZZAZIONE (String Pulling): Elimina i waypoint intermedi inutili
        # =====================================================================
        raw_positions = self.smooth_path(raw_positions)

        # =====================================================================
        # 2. DISACCOPPIAMENTO: SEPARAZIONE ROTAZIONE E TRASLAZIONE
        # =====================================================================
        return self.decouple_path(initial_yaw, raw_positions)


