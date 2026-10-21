import math
import heapq


class ObstacleAABB:
    def __init__(self, x, y, z, width, depth, height, drone_radius=0.2):
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
        """Controlla punti intermedi lungo il segmento per impedire di saltare attraverso i muri"""
        for step in range(1, 6):
            t = step / 6.0
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            iz = z1 + t * (z2 - z1)
            if not self.is_valid(ix, iy, iz):
                return False
        return True

    def get_neighbors(self, x, y, z):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:  # Riabilita il cambio di quota Y (salita/discesa)
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    # Se cambia quota (dy != 0), sale o scende in verticale pura sul posto:
                    # evita che il drone tagli in diagonale contro la parete frontale dell'ostacolo
                    if dy != 0 and (dx != 0 or dz != 0):
                        continue

                    nx = self._round_to_grid(x + dx * self.resolution)
                    ny = self._round_to_grid(y + dy * self.resolution)
                    nz = self._round_to_grid(z + dz * self.resolution)

                    if not self.is_valid(nx, ny, nz):
                        continue

                    # 1. Controlla che il segmento non attraversi un ostacolo
                    if not self.is_segment_valid(x, y, z, nx, ny, nz):
                        continue

                    # 2. Sul piano XZ, impedisci di tagliare gli spigoli in diagonale
                    if dx != 0 and dz != 0:
                        if not (self.is_valid(nx, y, z) and self.is_valid(x, y, nz)):
                            continue

                    cost = math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2)
                    neighbors.append(((nx, ny, nz), cost))
        return neighbors

    def dijkstra(self, start, goal, initial_yaw=0.0):
        real_start = (float(start[0]), float(start[1]), float(start[2]))
        grid_start = (
            self._round_to_grid(start[0]),
            self._round_to_grid(start[1]),
            self._round_to_grid(start[2]),
        )
        goal = (
            self._round_to_grid(goal[0]),
            self._round_to_grid(goal[1]),
            self._round_to_grid(goal[2]),
        )

        if not self.is_valid(*grid_start) or not self.is_valid(*goal):
            print(
                f"ERRORE: Start {grid_start} o Goal {goal} non validi (fuori mappa o dentro un ostacolo)."
            )
            return []

        queue = [(0.0, grid_start)]
        distances = {grid_start: 0.0}
        came_from = {grid_start: None}

        raw_positions = []
        while queue:
            current_dist, current_node = heapq.heappop(queue)

            if current_node == goal:
                curr = current_node
                while curr is not None:
                    raw_positions.append(curr)
                    curr = came_from[curr]
                raw_positions.reverse()
                break

            for next_node, weight in self.get_neighbors(*current_node):
                new_dist = current_dist + weight
                if (
                    next_node not in distances
                    or new_dist < distances[next_node]
                ):
                    distances[next_node] = new_dist
                    came_from[next_node] = current_node
                    heapq.heappush(queue, (new_dist, next_node))

        if not raw_positions:
            return []

        # Sostituiamo il primo nodo della griglia con la posizione esatta attuale del drone
        # così non torna indietro al centro della cella precedente
        raw_positions[0] = real_start

        # =====================================================================
        # DISACCOPPIAMENTO: SEPARAZIONE ROTAZIONE E TRASLAZIONE
        # =====================================================================
        decoupled_path = []
        current_yaw = initial_yaw

        for i in range(len(raw_positions) - 1):
            curr_pos = raw_positions[i]
            next_pos = raw_positions[i + 1]

            dx = next_pos[0] - curr_pos[0]
            dz = -(next_pos[2] - curr_pos[2])

            if abs(dx) > 0.01 or abs(dz) > 0.01:
                desired_yaw = math.atan2(dx, dz)
                diff_yaw = abs(
                    (desired_yaw - current_yaw + math.pi) % (2 * math.pi)
                    - math.pi
                )

                if diff_yaw > math.radians(1.0):
                    decoupled_path.append(
                        (curr_pos[0], curr_pos[1], curr_pos[2], desired_yaw)
                    )
                    current_yaw = desired_yaw

            decoupled_path.append(
                (next_pos[0], next_pos[1], next_pos[2], current_yaw)
            )

        return decoupled_path