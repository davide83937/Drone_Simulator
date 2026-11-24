import math




class PantoPlanningPartial:
    def __init__(self, path_manager):
        self.path_manager = path_manager
        self.list_vertexes = path_manager.vertexes
        self.start_vertex = None
        self.end_vertex = None
        self.list_vertex_path = []
        self.list_edge_path = []
        self.list_commands = []
        self.rejoin_idx = None

    @staticmethod
    def _find_nearest_grid_vertex(vertexes, target):
        tx = target[0] if isinstance(target, (list, tuple)) else target.x
        tz = target[2] if isinstance(target, (list, tuple)) else target.z
        free_verts = [v for v in vertexes if v.free]
        pool = free_verts if free_verts else vertexes
        return min(pool, key=lambda v: math.hypot(v.x - tx, v.z - tz))

    def _prepare_grid(self, original_vertex_path, hit_segments):
        # Libera solo i vertici del vecchio tratto sostituito, lasciando bloccato il cilindro
        start_seg = hit_segments[0]
        for v in original_vertex_path[start_seg:self.rejoin_idx]:
            for gv in self.list_vertexes:
                if gv.x == v.x and gv.z == v.z:
                    gv.free = True

        # Resetta direzioni e contatori di esplorazione
        for v in self.list_vertexes:
            v.index = 0
            v.directions = {'Ahead': True, 'Left': True, 'Back': True, 'Right': True}

    def action(self, next_v, current_vertex, direction):
        if next_v is None or not next_v.free:
            current_vertex.delete_direction(direction)
            current_vertex.index += 1
            return True
        current_vertex.delete_direction(direction)
        current_vertex.index += 1
        next_v.free = False
        self.list_vertex_path.append(next_v)
        return False

    def build_path(self):
        self.list_vertex_path = [self.start_vertex]
        while (self.list_vertex_path[-1].x != self.end_vertex.x or
               self.list_vertex_path[-1].z != self.end_vertex.z):
            current_vertex = self.list_vertex_path[-1]
            if current_vertex.index < 4:
                for direction in list(current_vertex.directions.keys())[current_vertex.index:]:
                    if direction == "Ahead" and current_vertex.directions[direction]:
                        next_v = self.path_manager.increment_step_positive_x(current_vertex)
                        if not self.action(next_v, current_vertex, direction):
                            break
                    elif direction == "Back" and current_vertex.directions[direction]:
                        next_v = self.path_manager.increment_step_negative_x(current_vertex)
                        if not self.action(next_v, current_vertex, direction):
                            break
                    elif direction == "Left" and current_vertex.directions[direction]:
                        next_v = self.path_manager.increment_step_negative_z(current_vertex)
                        if not self.action(next_v, current_vertex, direction):
                            break
                    elif direction == "Right" and current_vertex.directions[direction]:
                        next_v = self.path_manager.increment_step_positive_z(current_vertex)
                        if not self.action(next_v, current_vertex, direction):
                            break
            else:
                self.list_vertex_path.pop()

    def check_two_points(self, one, two, list_blocked_points):
        min_x, max_x = min(one.x, two.x), max(one.x, two.x)
        min_z, max_z = min(one.z, two.z), max(one.z, two.z)
        for point in list_blocked_points:
            if min_x <= point.x <= max_x and min_z <= point.z <= max_z:
                return True
        return False

    def delete_useless_points(self, list_blocked_points):
        lista_temp = [self.start_vertex, self.end_vertex]
        prev_len = len(self.list_vertex_path)
        j = 0

        for i in range(len(self.list_vertex_path)):
            if self.check_two_points(self.list_vertex_path[j], self.list_vertex_path[i], list_blocked_points):
                lista_temp.insert(-1, self.list_vertex_path[i - 1])
                j = i - 1

        self.list_edge_path = lista_temp
        if len(self.list_edge_path) < prev_len - 1:
            self.list_vertex_path = self.list_edge_path
            self.list_edge_path = []
            self.delete_useless_points(list_blocked_points)

    def join_points(self):
        from PathPlanning.PantoPlanning import vertex
        new_path = []
        for i in range(len(self.list_vertex_path) - 1):
            a = self.list_vertex_path[i]
            b = self.list_vertex_path[i + 1]
            new_path.append(a)

            curr_x, curr_z = a.x, a.z
            step_x = 5 if b.x > curr_x else -5
            while curr_x != b.x:
                curr_x += step_x
                if curr_x != b.x or curr_z != b.z:
                    new_path.append(vertex(curr_x, 0, curr_z))

            step_z = 5 if b.z > curr_z else -5
            while curr_z != b.z:
                curr_z += step_z
                if curr_z != b.z:
                    new_path.append(vertex(curr_x, 0, curr_z))

        if self.list_vertex_path:
            new_path.append(self.list_vertex_path[-1])
        self.list_vertex_path = new_path

    def connect_farthest_visible(self):
        if len(self.list_vertex_path) <= 2:
            return
        list_blocked_points = [v for v in self.list_vertexes if not v.free]
        new_path = [self.list_vertex_path[0]]
        current_idx = 0

        while current_idx < len(self.list_vertex_path) - 1:
            farthest_idx = current_idx + 1
            for check_idx in range(len(self.list_vertex_path) - 1, current_idx + 1, -1):
                if not self.check_two_points(self.list_vertex_path[current_idx],
                                             self.list_vertex_path[check_idx],
                                             list_blocked_points):
                    farthest_idx = check_idx
                    break
            new_path.append(self.list_vertex_path[farthest_idx])
            current_idx = farthest_idx

        self.list_vertex_path = new_path

    def smooth_path(self):
        list_blocked_points = [v for v in self.list_vertexes if not v.free]
        self.list_edge_path = [self.start_vertex, self.end_vertex]
        self.delete_useless_points(list_blocked_points)
        self.join_points()
        self.connect_farthest_visible()

    def build_commands(self):
        from PathPlanning.PantoPlanning import angle
        commands = []
        for i in range(len(self.list_vertex_path) - 1):
            current = self.list_vertex_path[i]
            next_vertex = self.list_vertex_path[i + 1]
            dx = next_vertex.x - current.x
            dz = next_vertex.z - current.z
            target_angle = math.degrees(math.atan2(dx, -dz))
            target_angle = (target_angle + 180) % 360 - 180
            commands.append(angle(target_angle))
            commands.append(next_vertex)
        self.list_commands = commands

    def compute(self, original_vertex_path, stop_pos, hit_segments):
        from PathPlanning.PantoPlanning import vertex
        if not hit_segments:
            print("[PARTIAL] hit_segments vuoto, nessun tratto parziale da ricalcolare.")
            return [], [], None
        last_seg = hit_segments[-1]
        self.rejoin_idx = min(last_seg + 2, len(original_vertex_path) - 1)
        self.start_vertex = self._find_nearest_grid_vertex(self.list_vertexes, stop_pos)
        self.end_vertex = self._find_nearest_grid_vertex(self.list_vertexes, original_vertex_path[self.rejoin_idx])

        self._prepare_grid(original_vertex_path, hit_segments)
        self.build_path()

        for v in self.list_vertex_path:
            v.free = True

        self.smooth_path()

        # Imposta la posizione esatta di stop come primo punto del tratto parziale
        stop_v = vertex(stop_pos[0], 0, stop_pos[2])
        if self.list_vertex_path:
            self.list_vertex_path[0] = stop_v

        self.build_commands()
        return self.list_vertex_path, self.list_commands, self.rejoin_idx