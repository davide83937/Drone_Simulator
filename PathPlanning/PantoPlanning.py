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

class segment:
    def __init__(self, direction):
        self.point_list = []
        self.direction = direction

class PantoPlanning:
    def __init__(self, path_manager ,start_vertex, end_vertex):
        self.list_vertex_path=[]
        self.list_edge_path=[]
        self.list_vertexes = path_manager.vertexes
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.path_manager = path_manager

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
        while (self.list_vertex_path[-1].x != self.end_vertex.x or self.list_vertex_path[-1].z != self.end_vertex.z):
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



    def run(self):
        self.build_path()
        self.free_path()
        self.smooth_path()







