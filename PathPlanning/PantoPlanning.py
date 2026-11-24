import math
import heapq
from PathPlanning.getObstacles import get_obstacles_path_manager


class vertex:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.free = True
        self.directions = {'Ahead': True, 'Left': True, 'Back': True, 'Right': True}
        self.index = 0


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
    def __init__(self, path_manager, start_vertex, end_vertex):
        self.list_vertex_path = []
        self.list_edge_path = []
        self.list_vertexes = path_manager.vertexes
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.path_manager = path_manager
        self.list_commands = []
        self.fake_mutex = True
        self.change_path = [False]
        self.overlapping_points = []
        self.hit_points = []
        self.last_cylinder = None
        self.cylinder = []

    def _nearest_vertex_3d(self, x: float, y: float, z: float, only_free: bool = False):
        vertexes = list(self.list_vertexes)
        if not vertexes:
            return vertex(x, y, z)
        pool = [v for v in vertexes if v.free] if only_free else vertexes
        if not pool:
            pool = vertexes
        return min(
            pool,
            key=lambda v: (v.x - x) ** 2 + (v.y - y) ** 2 + (v.z - z) ** 2
        )

    def _is_blocked_3d(self, v: vertex, cylinder: dict = None, margin: float = 5.0) -> bool:
        if not v.free:
            return True
        if cylinder is not None:
            cx, cz = cylinder["center_xz"]
            eff_r = cylinder["radius"] + margin
            if math.hypot(v.x - cx, v.z - cz) <= eff_r:
                return True
        return False

    # margin: margine di sicurezza (in metri/unità di mappa) aggiunto al raggio del cilindro ostacolo per allargare la zona vietata.
    # neighbor_radius: distanza massima entro cui due vertici 3D vengono collegati come vicini nel grafo di Dijkstra; se è troppo piccolo rispetto al passo della griglia il grafo risulta sconnesso, se è troppo grande crea archi lunghi che possono tagliare ostacoli o aumentare il costo computazionale.
    # pad: margine extra (in metri/unità di mappa) aggiunto attorno al bounding box tra start_node e goal_node per ritagliare il sotto-grafo 3D locale su cui gira Dijkstra.
    def dijkstra_partial_3d(self, start_pos, goal_vertex: vertex, cylinder: dict = None,
                            margin: float = 0.0, neighbor_radius: float = 20.0, pad: float = 40.0):
        """Calcola un percorso parziale in 3D (x, y, z) con Dijkstra tra start_pos e goal_vertex."""
        sx = float(start_pos[0])
        sy = float(start_pos[1]) if len(start_pos) > 1 else 0.0
        sz = float(start_pos[2]) if len(start_pos) > 2 else 0.0

        start_node = self._nearest_vertex_3d(sx, sy, sz, only_free=True)
        goal_node = self._nearest_vertex_3d(goal_vertex.x, goal_vertex.y, goal_vertex.z, only_free=True)

        min_x = min(start_node.x, goal_node.x) - pad
        max_x = max(start_node.x, goal_node.x) + pad
        min_y = min(start_node.y, goal_node.y) - pad
        max_y = max(start_node.y, goal_node.y) + pad
        min_z = min(start_node.z, goal_node.z) - pad
        max_z = max(start_node.z, goal_node.z) + pad

        all_verts = list(self.list_vertexes)
        candidates = [
            v for v in all_verts
            if min_x <= v.x <= max_x and min_y <= v.y <= max_y and min_z <= v.z <= max_z
        ]
        for special in (start_node, goal_node):
            if special not in candidates:
                candidates.append(special)

        blocked_ids = {
            id(v) for v in candidates
            if v is not start_node and v is not goal_node and self._is_blocked_3d(v, cylinder, margin)
        }

        adj = {id(v): [] for v in candidates}
        r_sq = neighbor_radius * neighbor_radius
        for i, u in enumerate(candidates):
            id_u = id(u)
            if id_u in blocked_ids:
                continue
            for j in range(i + 1, len(candidates)):
                w = candidates[j]
                id_w = id(w)
                if id_w in blocked_ids:
                    continue
                dx = u.x - w.x
                dy = u.y - w.y
                dz = u.z - w.z
                d_sq = dx * dx + dy * dy + dz * dz
                if 0.0 < d_sq <= r_sq:
                    dist = math.sqrt(d_sq)
                    adj[id_u].append((dist, id_w, w))
                    adj[id_w].append((dist, id_u, u))

        # Garantisce che start_node e goal_node abbiano almeno qualche vicino libero se isolati
        for anchor in (start_node, goal_node):
            anchor_id = id(anchor)
            if anchor_id not in blocked_ids and not adj[anchor_id]:
                others = [
                    (math.sqrt((anchor.x - v.x) ** 2 + (anchor.y - v.y) ** 2 + (anchor.z - v.z) ** 2), v)
                    for v in candidates
                    if id(v) != anchor_id and id(v) not in blocked_ids
                ]
                others.sort(key=lambda item: item[0])
                for dist, v in others[:4]:
                    adj[anchor_id].append((dist, id(v), v))
                    adj[id(v)].append((dist, anchor_id, anchor))

        start_id = id(start_node)
        goal_id = id(goal_node)

        pq = [(0.0, start_id)]
        dist_map = {start_id: 0.0}
        prev_map = {start_id: None}
        node_map = {id(v): v for v in candidates}

        while pq:
            curr_cost, curr_id = heapq.heappop(pq)
            if curr_cost > dist_map.get(curr_id, float("inf")):
                continue
            if curr_id == goal_id:
                break

            for step_cost, nxt_id, _ in adj.get(curr_id, []):
                new_cost = curr_cost + step_cost
                if new_cost < dist_map.get(nxt_id, float("inf")):
                    dist_map[nxt_id] = new_cost
                    prev_map[nxt_id] = curr_id
                    heapq.heappush(pq, (new_cost, nxt_id))

        if goal_id not in prev_map:
            return [vertex(sx, sy, sz), vertex(goal_vertex.x, goal_vertex.y, goal_vertex.z)]

        chain = []
        curr = goal_id
        while curr is not None:
            chain.append(node_map[curr])
            curr = prev_map[curr]
        chain.reverse()

        result = [vertex(sx, sy, sz)]
        for v in chain[1:]:
            result.append(vertex(v.x, v.y, v.z))
        return result

    def add_vertex_in_path(self, v):
        self.list_vertex_path.append(v)

    def delete_last_vertex_in_path(self):
        self.list_vertex_path.pop()

    def action(self, next, current_vertex, direction):
        if next is None or not next.free:
            current_vertex.delete_direction(direction)
            current_vertex.index = current_vertex.index + 1
            return True
        elif next.free:
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
            if current_vertex.index < 4:
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
                self.delete_last_vertex_in_path()

    def check_vertexes(self):
        obstacles = get_obstacles_path_manager()
        for v in self.list_vertexes:
            v.free = True
            for obstacle in obstacles:
                if (v.x >= obstacle.min_x - 10 and v.x <= obstacle.max_x + 10 and
                        v.z >= obstacle.min_z - 10 and v.z <= obstacle.max_z + 10):
                    v.free = False
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
        lista_temp = [self.start_vertex, self.end_vertex]
        prev_len = len(self.list_vertex_path)
        j = 0

        for i in range(0, len(self.list_vertex_path)):
            result = self.check_two_points(self.list_vertex_path[j], self.list_vertex_path[i], list_blocked_points)
            if result:
                lista_temp.insert(-1, self.list_vertex_path[i - 1])
                j = i - 1

        self.list_edge_path = lista_temp

        if len(self.list_edge_path) < prev_len - 1:
            self.list_vertex_path = self.list_edge_path
            self.list_edge_path = []
            self.delete_useless_points(list_blocked_points)

    def free_path(self):
        path_set = set(self.list_vertex_path)
        for v in self.list_vertexes:
            if v in path_set:
                v.free = True
        self.check_vertexes()

    def smooth_path(self):
        print("INIZIO SMOOTH")
        list_blocked_points = [v for v in self.list_vertexes if not v.free]
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

        path = list(self.list_vertex_path)
        cmds = list(self.list_commands)

        print(
            f"[DEBUG CYL] center=({cx:.2f}, {cz:.2f}), R={r:.2f}, margin={margin:.2f}, "
            f"eff_R={effective_radius:.2f}, cmd_idx={current_index}, "
            f"path_len={len(path)}, cmds_len={len(cmds)}"
        )

        if not path or len(path) < 2:
            print("[DEBUG CYL] list_vertex_path troppo corta o vuota")
            return False

        start_seg = max(0, (current_index - 1) // 2 - 1)
        start_seg = min(start_seg, len(path) - 2)
        print(f"[DEBUG CYL] controllo segmenti da {start_seg} a {len(path) - 2}")
        hit_points = []
        for i in range(start_seg, len(path) - 1):
            a = path[i]
            b = path[i + 1]
            hit, dist, t, px, pz = self._segment_distance_and_hit(
                a.x, a.z, b.x, b.z, cx, cz, effective_radius
            )
            print(
                f"  [SEG {i}] ({a.x}, {a.z}) -> ({b.x}, {b.z}) | "
                f"dist={dist:.2f} vs eff_R={effective_radius:.2f} | t={t:.2f} | hit={hit}"
            )
            if hit:
                hit_points.append(i)
                self.hit_points = hit_points
                self.last_cylinder = cylinder
                print(
                    f"[COLLISIONE PERCORSO] Cilindro in ({cx:.2f}, {cz:.2f}) R={r:.2f} "
                    f"interseca il segmento {i} ({a.x}, {a.z}) -> ({b.x}, {b.z}) "
                    f"(cmd_index={current_index})"
                )
                return True

        self.hit_points = hit_points
        print("NON TOCCA")
        return False

    def _is_inside_existing_cylinder(self, cylinder: dict, extra_margin: float = 0.0) -> bool:
        """Restituisce True se il centro del cilindro cade dentro uno già confermato in self.cylinder."""
        cx, cz = cylinder["center_xz"]
        for existing in self.cylinder:
            ex, ez = existing["center_xz"]
            limit = float(existing["radius"]) + float(extra_margin)
            if math.hypot(cx - ex, cz - ez) <= limit:
                return True
        return False

    def cylinder_found(self, cylinder: dict, current_index: int, margin: float = 5.0,
                       duplicate_margin: float = 0.0) -> bool:
        """Controlla se il cilindro interseca segmenti e punti del percorso."""
        cx, cz = cylinder["center_xz"]
        r = cylinder["radius"]
        effective_radius = r + margin

        path = list(self.list_vertex_path)
        cmds = list(self.list_commands)

        print(
            f"[DEBUG CYL] center=({cx:.2f}, {cz:.2f}), R={r:.2f}, margin={margin:.2f}, "
            f"eff_R={effective_radius:.2f}, cmd_idx={current_index}, "
            f"path_len={len(path)}, cmds_len={len(cmds)}"
        )

        if self._is_inside_existing_cylinder(cylinder, extra_margin=duplicate_margin):
            print(f"[CYL IGNORATO] Centro ({cx:.2f}, {cz:.2f}) già dentro un cilindro confermato.")
            self.overlapping_points = []
            return False

        if not path or len(path) < 2:
            print("[DEBUG CYL] list_vertex_path troppo corta o vuota")
            self.overlapping_points = []
            return False

        start_seg = max(0, (current_index - 1) // 2 - 1)
        start_seg = min(start_seg, len(path) - 2)

        print(f"[DEBUG CYL] controllo segmenti da {start_seg} a {len(path) - 2}")

        segment_hit = False
        for i in range(start_seg, len(path) - 1):
            a = path[i]
            b = path[i + 1]

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

        overlapping = []
        for i in range(start_seg, len(path)):
            point = path[i]
            dx = point.x - cx
            dz = point.z - cz
            dist = math.sqrt(dx ** 2 + dz ** 2)
            hit = dist <= effective_radius

            print(
                f"  [POINT {i}] ({point.x}, {point.z}) | "
                f"dist={dist:.2f} vs eff_R={effective_radius:.2f} | hit={hit}"
            )

            if hit:
                overlapping.append(i)
                print(f"[PUNTO INTERSECATO] Indice {i}: ({point.x}, {point.z})")

        self.overlapping_points = overlapping
        if segment_hit or overlapping:
            self.last_cylinder = cylinder
            self.cylinder.append(cylinder)

        print(
            f"[DEBUG CYL] segment_hit={segment_hit}, "
            f"overlapping_points={self.overlapping_points}, "
            f"confirmed_cyls={len(self.cylinder)}"
        )

        return segment_hit

    def add_point(self, index, pos):
        y_val = float(pos[1]) if len(pos) > 1 else 0.0
        z_val = float(pos[2]) if len(pos) > 2 else 0.0
        x_val = float(pos[0])
        print(f"[DEBUG] add_point x:{x_val:.2f}, y:{y_val:.2f}, z:{z_val:.2f}")
        v = vertex(x_val, y_val, z_val)
        insert_idx = max(0, min(index + 1, len(self.list_vertex_path)))
        self.list_vertex_path.insert(insert_idx, v)
        return insert_idx

    def process_recalculating(self, index, pos, cylinder=None, margin=10.0):
        if self.fake_mutex:
            self.fake_mutex = False
            try:
                active_cylinder = cylinder if cylinder is not None else self.last_cylinder
                overlapping_set = set(getattr(self, "overlapping_points", []))
                print(f"\n=== [RECALC START] index={index}, pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), overlapping={overlapping_set}, cyl={active_cylinder} ===")

                self.list_vertex_path = [
                    v for i, v in enumerate(self.list_vertex_path)
                    if i not in overlapping_set
                ]

                if not self.list_vertex_path:
                    self.list_vertex_path = [self.end_vertex]

                safe_index = max(0, min(index, len(self.list_vertex_path)))
                stop_idx = self.add_point(safe_index - 1, pos)

                rejoin_idx = min(stop_idx + 1, len(self.list_vertex_path) - 1)
                goal_v = self.list_vertex_path[rejoin_idx]
                print(f"[RECALC] safe_index={safe_index}, stop_idx={stop_idx}, rejoin_idx={rejoin_idx}, goal_v=({goal_v.x:.2f}, {goal_v.y:.2f}, {goal_v.z:.2f})")

                bypass_3d = self.dijkstra_partial_3d(pos, goal_v, cylinder=active_cylinder, margin=margin)
                print(f"[RECALC] bypass_3d len={len(bypass_3d)}:")
                for bi, bv in enumerate(bypass_3d):
                    print(f"  BYPASS[{bi}] -> ({bv.x:.2f}, {bv.y:.2f}, {bv.z:.2f})")

                self.list_vertex_path = (
                    self.list_vertex_path[:stop_idx]
                    + bypass_3d
                    + self.list_vertex_path[rejoin_idx + 1:]
                )

                print(f"[RECALC] nuovo list_vertex_path len={len(self.list_vertex_path)}:")
                for pi, pv in enumerate(self.list_vertex_path):
                    print(f"  PATH[{pi}] -> ({pv.x:.2f}, {pv.y:.2f}, {pv.z:.2f})")

                self.add_rotation_angles(0.0)
                self.change_path[0] = True
                print(f"=== [RECALC END] cmds_len={len(self.list_commands)} ===\n")
            finally:
                self.fake_mutex = True

    def process_recalculating2(self, index, pos):
        if self.fake_mutex:
            self.fake_mutex = False
            try:
                overlapping_set = set(getattr(self, "overlapping_points", []))
                self.list_vertex_path = [
                    v for i, v in enumerate(self.list_vertex_path)
                    if i not in overlapping_set
                ]
                if not self.list_vertex_path:
                    self.list_vertex_path = [self.end_vertex]

                safe_index = max(1, min(index, len(self.list_vertex_path)))
                self.add_point(safe_index - 1, pos)
                print(f"NUOVA LUNGHEZZA {len(self.list_vertex_path)}")
                for v in self.list_vertex_path:
                    print(f"Punto x={v.x:.2f}, y={v.y:.2f}, z={v.z:.2f}")

                ref_v = self.list_vertex_path[safe_index - 1]
                nearest_point = self.path_manager.find_nearest_vertex(ref_v.x, ref_v.z)
                goal_v = self.list_vertex_path[min(safe_index, len(self.list_vertex_path) - 1)]

                panto2 = PantoPlanning(self.path_manager, nearest_point, goal_v)
                panto2.build_path()
                panto2.free_path()
                panto2.smooth_path()
                panto2.add_rotation_angles(0.0)

                self.list_vertex_path[safe_index:safe_index] = panto2.list_vertex_path[1:-1]
                self.add_rotation_angles(0.0)
                self.change_path[0] = True
            finally:
                self.fake_mutex = True