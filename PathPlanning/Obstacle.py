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

class PathPlanner:
    def __init__(self, bounds, resolution, obstacles):
        self.resolution = resolution
        self.obstacles = obstacles
        self.bounds = bounds  # Ora accetta correttamente il dizionario

    def _round_to_grid(self, val):
        return round(round(val / self.resolution) * self.resolution, 3)

    def is_valid(self, x, y, z):
        # Controlla i limiti della mappa usando il dizionario
        if not (self.bounds['x'][0] <= x <= self.bounds['x'][1] and
                self.bounds['y'][0] <= y <= self.bounds['y'][1] and
                self.bounds['z'][0] <= z <= self.bounds['z'][1]):
            return False
        # Controlla le collisioni
        for obs in self.obstacles:
            if obs.is_inside(x, y, z):
                return False
        return True

    def get_neighbors(self, x, y, z):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    nx = self._round_to_grid(x + dx * self.resolution)
                    ny = self._round_to_grid(y + dy * self.resolution)
                    nz = self._round_to_grid(z + dz * self.resolution)

                    if self.is_valid(nx, ny, nz):
                        cost = math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2)
                        neighbors.append(((nx, ny, nz), cost))
        return neighbors

    def dijkstra(self, start, goal):
        start = (self._round_to_grid(start[0]), self._round_to_grid(start[1]), self._round_to_grid(start[2]))
        goal = (self._round_to_grid(goal[0]), self._round_to_grid(goal[1]), self._round_to_grid(goal[2]))

        if not self.is_valid(*start) or not self.is_valid(*goal):
            print(f"ERRORE: Start {start} o Goal {goal} non validi (fuori mappa o dentro un ostacolo).")
            return []

        queue = [(0.0, start)]
        distances = {start: 0.0}
        came_from = {start: None}

        while queue:
            current_dist, current_node = heapq.heappop(queue)

            if current_node == goal:
                path = []
                while current_node is not None:
                    path.append(current_node)
                    current_node = came_from[current_node]
                return path[::-1]

            for next_node, weight in self.get_neighbors(*current_node):
                new_dist = current_dist + weight
                if next_node not in distances or new_dist < distances[next_node]:
                    distances[next_node] = new_dist
                    came_from[next_node] = current_node
                    heapq.heappush(queue, (new_dist, next_node))

        return []