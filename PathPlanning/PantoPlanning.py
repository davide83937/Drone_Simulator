import math

from PathPlanning.getObstacles import get_obstacles_path_manager


class vertex:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.free = True
        #self.directions = {'Ahead': True, 'Right': True, 'Back': True, 'Left': True}
        self.directions = {'Ahead': True, 'Left': True, 'Back': True, 'Right': True}
        self.index = 0
        self.fake_mutex=True
        self.change_path=[]
        self.change_path.append(False)

    def delete_ahead(self):
        self.directions['Ahead'] = False

    def delete_back(self):
        self.directions['Back'] = False

    def delete_left(self):
        self.directions['Left'] = False

    def delete_right(self):
        self.directions['Right'] = False

    def delete_direction(self, direction):
        self.directions[direction] = False

class angle:
    def __init__(self, angle):
        self.angle = angle

class altitude:
    def __init__(self, altitude):
        self.altitude = altitude

class PantoPlanning:
    def __init__(self, path_manager ,start_vertex, end_vertex):
        self.list_vertex_path=[]
        self.list_edge_path=[]
        self.list_vertexes = path_manager.vertexes
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.path_manager = path_manager
        self.list_commands = []
        self.fake_mutex = True
        self.change_path=[]
        self.change_path.append(False)


    def add_vertex_in_path(self, v):
        self.list_vertex_path.append(v)

    def delete_last_vertex_in_path(self):
        self.list_vertex_path.pop()

    def action(self, next, current_vertex, direction):
        if next is None or not next.free:
            #print(f"Dir Eliminata {current_vertex.x}, {current_vertex.z}, {current_vertex.index}, {direction}")
            current_vertex.delete_direction(direction)
            current_vertex.index = current_vertex.index + 1
            return True
        elif next.free:
            #print(f"Nuovo punto: {next.x}, {next.z}, {next.free}")
            current_vertex.delete_direction(direction)
            current_vertex.index = current_vertex.index + 1
            next.free = False
            self.list_vertex_path.append(next)
            return False

    def build_path(self):
        current_vertex = self.start_vertex
        self.list_vertex_path.append(current_vertex)
        while self.list_vertex_path and (
                self.list_vertex_path[-1].x != self.end_vertex.x or
                self.list_vertex_path[-1].z != self.end_vertex.z
        ):
            current_vertex = self.list_vertex_path[-1]
            if(current_vertex.index < 4):
                #print(f"Current {current_vertex.x}, {current_vertex.z}, {current_vertex.directions}")
                for direction in list(current_vertex.directions.keys())[current_vertex.index:]:
                    if direction == "Ahead" and current_vertex.directions[direction]:
                        next = self.path_manager.increment_step_positive_x(current_vertex)
                        breaker = self.action(next, current_vertex, direction)
                        if not breaker:
                            break
                    elif direction == "Back" and current_vertex.directions[direction]:
                        next = self.path_manager.increment_step_negative_x(current_vertex)
                        breaker = self.action(next, current_vertex, direction)
                        if not breaker:
                            break
                    elif direction == "Left" and current_vertex.directions[direction]:
                        next = self.path_manager.increment_step_negative_z(current_vertex)
                        breaker = self.action(next, current_vertex, direction)
                        if not breaker:
                            break
                    elif direction == "Right" and current_vertex.directions[direction]:
                        next = self.path_manager.increment_step_positive_z(current_vertex)
                        breaker = self.action(next, current_vertex, direction)
                        if not breaker:
                            break
            else:
                #print(f"Punto eliminato { self.list_vertex_path[-1]}")
                self.delete_last_vertex_in_path()

    def check_vertexes(self):
        obstacles = get_obstacles_path_manager()
        for vertex in self.list_vertexes:
            vertex.free = True
            for obstacle in obstacles:
                if (vertex.x >= obstacle.min_x - 10 and vertex.x <= obstacle.max_x + 10 and
                        vertex.z >= obstacle.min_z - 10 and vertex.z <= obstacle.max_z + 10):
                    vertex.free = False
                    break


    def check_two_points(self, one, two, list_blocked_points):
        min_x = min(one.x, two.x)
        min_z = min(one.z, two.z)
        max_x = max(one.x, two.x)
        max_z = max(one.z, two.z)
        for point in list_blocked_points:
            if point.x >= min_x and point.x <= max_x and point.z >= min_z and point.z <= max_z:
                return True
        return False

    def delete_useless_points(self, list_blocked_points):
        lista_temp=[]
        lista_temp.append(self.start_vertex)
        lista_temp.append(self.end_vertex)
        prev_len = len(self.list_vertex_path)
        j = 0

        for i in range(0, len(self.list_vertex_path)):
            result = self.check_two_points(self.list_vertex_path[j], self.list_vertex_path[i], list_blocked_points)
            if result :
                #print(f"Punto Ostacolato {i}: {self.list_vertex_path[i]}")
                lista_temp.insert(-1, self.list_vertex_path[i - 1])
                j = i - 1

        self.list_edge_path = lista_temp

        #print(f"Prev_len: {prev_len-1}, actual_len: {len(self.list_edge_path)}")
        if len(self.list_edge_path) < prev_len-1:
            self.list_vertex_path = self.list_edge_path
            self.list_edge_path = []
            self.delete_useless_points(list_blocked_points)


    def free_path(self):
        for vertex in self.list_vertexes:
            if vertex in self.list_vertex_path:
                vertex.free = True
        self.check_vertexes()




    def smooth_path(self):
        print("INIZIO SMOOTH")
        list_blocked_points = []
        for vertex in self.list_vertexes:
            if not vertex.free:
                list_blocked_points.append(vertex)
        self.list_edge_path.append(self.start_vertex)
        self.list_edge_path.append(self.end_vertex)
        self.delete_useless_points(list_blocked_points)
        self.join_points()
        self.connect_farthest_visible()


    def join_points(self):
        new_path = []

        for i in range(len(self.list_vertex_path) - 1):
            a = self.list_vertex_path[i]
            b = self.list_vertex_path[i + 1]
            new_path.append(a)

            curr_x = a.x
            curr_z = a.z

            # Spostamento lungo X (sia positivo che negativo)
            step_x = 5 if b.x > curr_x else -5
            while curr_x != b.x:
                curr_x += step_x
                # Evitiamo di duplicare b se z è già uguale
                if curr_x != b.x or curr_z != b.z:
                    new_path.append(vertex(curr_x, 0, curr_z))

            # Spostamento lungo Z (sia positivo che negativo)
            step_z = 5 if b.z > curr_z else -5
            while curr_z != b.z:
                curr_z += step_z
                if curr_z != b.z:
                    new_path.append(vertex(curr_x, 0, curr_z))

        # Aggiungi l'ultimo punto del percorso
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

            # Cerca partendo dalla fine verso current_idx + 1
            for check_idx in range(len(self.list_vertex_path) - 1, current_idx + 1, -1):
                if not self.check_two_points(self.list_vertex_path[current_idx], self.list_vertex_path[check_idx],
                                             list_blocked_points):
                    farthest_idx = check_idx
                    break

            new_path.append(self.list_vertex_path[farthest_idx])
            current_idx = farthest_idx

        self.list_vertex_path = new_path

    def add_rotation_angles_relative(self, initial_orientation):
        new_path = [altitude(15)]

        previous_angle = initial_orientation

        for i in range(len(self.list_vertex_path) - 1):
            current = self.list_vertex_path[i]
            next_vertex = self.list_vertex_path[i + 1]

            dx = next_vertex.x - current.x
            dz = next_vertex.z - current.z

            current_angle = math.degrees(math.atan2(dz, dx))

            rotation = current_angle - previous_angle

            # Normalizzazione tra -180 e +180 gradi
            rotation = (rotation + 180) % 360 - 180

            new_path.append(angle(rotation))
            new_path.append(next_vertex)

            previous_angle = current_angle

        self.list_commands = new_path

    def add_rotation_angles(self, initial_orientation=0.0):
        new_path = [altitude(15)]

        for i in range(len(self.list_vertex_path) - 1):
            current = self.list_vertex_path[i]
            next_vertex = self.list_vertex_path[i + 1]

            dx = next_vertex.x - current.x
            dz = next_vertex.z - current.z

            target_angle = math.degrees(math.atan2(dx, -dz))
            target_angle = (target_angle + 180) % 360 - 180

            new_path.append(angle(target_angle))
            new_path.append(next_vertex)

        self.list_commands = new_path



    def run(self):
        self.build_path()
        self.free_path()
        self.smooth_path()
        self.add_rotation_angles(0.0)
        for i in self.list_commands:
            if isinstance(i, vertex):
                print(f"Punto: {i.x}, {i.y}, {i.z}")

            elif isinstance(i, angle):
                print(f"Rotazione: {i.angle}°")

            elif isinstance(i, altitude):
                print(f"Altitudine: {i.altitude} m")

    @staticmethod
    def _segment_distance_and_hit(ax: float, az: float, bx: float, bz: float,
                                  cx: float, cz: float, radius: float):
        """Restituisce (interseca: bool, dist: float, t: float, closest_x: float, closest_z: float)."""
        ab_x = bx - ax
        ab_z = bz - az
        ab_len_sq = ab_x * ab_x + ab_z * ab_z

        if ab_len_sq == 0.0:
            dist = math.hypot(cx - ax, cz - az)
            return dist <= radius, dist, 0.0, float(ax), float(az)

        t = ((cx - ax) * ab_x + (cz - az) * ab_z) / ab_len_sq
        t = max(0.0, min(1.0, t))

        closest_x = ax + t * ab_x
        closest_z = az + t * ab_z
        dist = math.hypot(cx - closest_x, cz - closest_z)
        return dist <= radius, dist, t, closest_x, closest_z

    def cylinder_found2(self, cylinder: dict, current_index: int, margin: float = 20.0) -> bool:
        """Controlla se il cilindro interseca uno dei segmenti rimanenti da current_index."""
        cx, cz = cylinder["center_xz"]
        r = cylinder["radius"]
        effective_radius = r + margin

        print(
            f"[DEBUG CYL] center=({cx:.2f}, {cz:.2f}), R={r:.2f}, margin={margin:.2f}, "
            f"eff_R={effective_radius:.2f}, cmd_idx={current_index}, "
            f"path_len={len(self.list_vertex_path)}, cmds_len={len(self.list_commands)}"
        )

        if not self.list_vertex_path or len(self.list_vertex_path) < 2:
            print("[DEBUG CYL] list_vertex_path troppo corta o vuota")
            return False

        start_seg = max(0, (current_index - 1) // 2 - 1)
        start_seg = min(start_seg, len(self.list_vertex_path) - 2)
        print(f"[DEBUG CYL] controllo segmenti da {start_seg} a {len(self.list_vertex_path) - 2}")
        self.hit_points = []
        for i in range(start_seg, len(self.list_vertex_path) - 1):
            a = self.list_vertex_path[i]
            b = self.list_vertex_path[i + 1]
            hit, dist, t, px, pz = self._segment_distance_and_hit(
                a.x, a.z, b.x, b.z, cx, cz, effective_radius
            )
            print(
                f"  [SEG {i}] ({a.x}, {a.z}) -> ({b.x}, {b.z}) | "
                f"dist={dist:.2f} vs eff_R={effective_radius:.2f} | t={t:.2f} | hit={hit}"
            )
            if hit:
                self.hit_points.append(i)
                print(
                    f"[COLLISIONE PERCORSO] Cilindro in ({cx:.2f}, {cz:.2f}) R={r:.2f} "
                    f"interseca il segmento {i} ({a.x}, {a.z}) -> ({b.x}, {b.z}) "
                    f"(cmd_index={current_index})"
                )
                return True

        print("NON TOCCA")
        return False

    def cylinder_found(self, cylinder: dict, current_index: int, margin: float = 5.0) -> tuple[bool, list]:
        """Controlla se il cilindro interseca segmenti e punti del percorso."""

        cx, cz = cylinder["center_xz"]
        r = cylinder["radius"]
        effective_radius = r + margin

        print(
            f"[DEBUG CYL] center=({cx:.2f}, {cz:.2f}), R={r:.2f}, margin={margin:.2f}, "
            f"eff_R={effective_radius:.2f}, cmd_idx={current_index}, "
            f"path_len={len(self.list_vertex_path)}, cmds_len={len(self.list_commands)}"
        )

        if not self.list_vertex_path or len(self.list_vertex_path) < 2:
            print("[DEBUG CYL] list_vertex_path troppo corta o vuota")
            return False, []

        start_seg = max(0, (current_index - 1) // 2 - 1)
        start_seg = min(start_seg, len(self.list_vertex_path) - 2)

        print(f"[DEBUG CYL] controllo segmenti da {start_seg} a {len(self.list_vertex_path) - 2}")

        # 1. Controllo intersezione segmenti
        segment_hit = False

        for i in range(start_seg, len(self.list_vertex_path) - 1):
            a = self.list_vertex_path[i]
            b = self.list_vertex_path[i + 1]

            hit, dist, t, px, pz = self._segment_distance_and_hit(
                a.x, a.z, b.x, b.z, cx, cz, effective_radius
            )

            print(
                f"  [SEG {i}] ({a.x}, {a.z}) -> ({b.x}, {b.z}) | "
                f"dist={dist:.2f} vs eff_R={effective_radius:.2f} | t={t:.2f} | hit={hit}"
            )

            if hit:
                segment_hit = True
                print(
                    f"[COLLISIONE PERCORSO] Cilindro in ({cx:.2f}, {cz:.2f}) R={r:.2f} "
                    f"interseca il segmento {i} ({a.x}, {a.z}) -> ({b.x}, {b.z}) "
                    f"(cmd_index={current_index})"
                )

        if not segment_hit:
            print("NON TOCCA NESSUN SEGMENTO")

        # 2. Controllo intersezione punti
        self.overlapping_points = []

        for i in range(start_seg, len(self.list_vertex_path)):
            point = self.list_vertex_path[i]

            dx = point.x - cx
            dz = point.z - cz
            dist = math.sqrt(dx ** 2 + dz ** 2)

            hit = dist <= effective_radius

            print(
                f"  [POINT {i}] ({point.x}, {point.z}) | "
                f"dist={dist:.2f} vs eff_R={effective_radius:.2f} | hit={hit}"
            )

            if hit:
                self.overlapping_points.append(i)
                print(f"[PUNTO INTERSECATO] Indice {i}: ({point.x}, {point.z})")

        print(
            f"[DEBUG CYL] segment_hit={segment_hit}, "
            f"overlapping_points={self.overlapping_points}"
        )

        return segment_hit


    def add_point(self, index, pos):

        print(f"[DEBUG] add_point x:{pos[0]:.2f}, y:{0.0:.2f}, z:{pos[2]:.2f}")
        v = vertex(pos[0], 0.0, pos[2])
        self.list_vertex_path.insert(index+1, v)

    def process_recalculating(self, index, pos):
        if self.fake_mutex:
            self.fake_mutex = False
            self.list_vertex_path = [
                vertex for i, vertex in enumerate(self.list_vertex_path)
                if i not in self.overlapping_points
            ]
            self.add_point(index-1, pos)
            print(f"NUOVA LUNGHEZZA {len(self.list_vertex_path)}")
            for i in self.list_vertex_path:
                print(f"Punto x={i.x:.2f}, y={i.y:.2f}, z={i.z:.2f}")

            panto2 = PantoPlanning(self.path_manager, self.list_vertex_path[index-1], self.list_vertex_path[index])
            panto2.build_path()
            panto2.free_path()
            self.list_vertex_path[index:index] = panto2.list_vertex_path[1:-1]
            self.change_path[0]=True
            self.fake_mutex = True







