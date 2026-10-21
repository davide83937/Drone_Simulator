import os
import math
import json
from std_msgs.msg import String
from sharedState import current_state

# --- SETTINGS AMBIENTE WINDOWS ROS 2 ---
extra_dll_dirs = [
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\Library\bin",
]

for directory in extra_dll_dirs:
    if os.path.isdir(directory):
        os.add_dll_directory(directory)
        os.environ["PATH"] = directory + os.pathsep + os.environ.get("PATH", "")

prefix_path = (
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces;"
    r"C:\Users\david\OneDrive\Desktop\ros2-windows"
)
os.environ.setdefault("AMENT_PREFIX_PATH", prefix_path)
# ----------------------------------------

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import MoveAxis
from geometry_msgs.msg import Vector3

# IMPORTA LA LOGICA DAL TUO PRIMO FILE
# (Sostituisci "path_planner" col nome effettivo del file 1)
from Obstacle import PathPlanner, ObstacleAABB


class AutonomousDroneClient(Node):

    def __init__(
        self, planner, final_goal, path, start_pos=(0.0, 15.0, 0.0), start_yaw=0.0
    ):
        super().__init__('autonomous_drone_client')
        self._action_client = ActionClient(self, MoveAxis, 'move_robot')

        # Sottoscrizione al topic di visione
        self.create_subscription(
            String, 'new_static_obstacle', self.obstacle_callback, 10
        )

        # Aggiornamento in tempo reale della posizione e rotazione del drone
        self.create_subscription(
            Vector3, 'drone_position', self.live_pos_callback, 10
        )
        self.create_subscription(
            Vector3, 'drone_rotation', self.live_rot_callback, 10
        )

        # Publisher per avvisare il LiDAR Viewer che l'ostacolo intralcia il cammino
        self.blocking_obs_pub = self.create_publisher(
            String, 'blocking_obstacle', 10
        )
        self.planner = planner
        self.final_goal = final_goal
        self.path = path
        self.current_wp_index = 0

        # Convenzione unica in tutto il file: (X, Altezza=Z_ros, Profondità=Y_ros)
        self.drone_pos = start_pos
        self.drone_yaw = start_yaw

        self.current_goal_handle = None
        self.is_replanning = False

    def live_pos_callback(self, msg):
        """Mantiene self.drone_pos sempre aggiornata nel formato (X, Altezza, Profondità)"""
        self.drone_pos = (msg.x, msg.z, msg.y)

    def live_rot_callback(self, msg):
        """Mantiene self.drone_yaw sempre aggiornato in radianti"""
        self.drone_yaw = math.radians(msg.z)

    def obstacle_callback(self, msg):
        """Riceve un nuovo ostacolo fisso dal LiDAR e valuta se è sulla rotta"""
        if (
            self.is_replanning
            or not self.path
            or self.current_wp_index >= len(self.path)
        ):
            return

        data = json.loads(msg.data)

        # Allineamento agli assi di Dijkstra: (X, Altezza, Profondità)
        new_obs = ObstacleAABB(
            x=data['x'],
            y=data['z'],  # Altezza dal suolo
            z=data['y'],  # Profondità
            width=data['w'],
            depth=data['h'],  # Dimensione altezza sull'asse Y
            height=data['d'],  # Dimensione profondità sull'asse Z
            drone_radius=7.0,
        )
        self.planner.obstacles.append(new_obs)

        self.get_logger().info(f'\n--- VALUTAZIONE NUOVO OSTACOLO ---')
        self.get_logger().info(
            f'Posizione attuale drone: X:{self.drone_pos[0]:.1f}, Y(Alt):{self.drone_pos[1]:.1f}, Z(Prof):{self.drone_pos[2]:.1f}'
        )
        self.get_logger().info(
            f'Limiti X (Orizzontale): [{new_obs.min_x:.1f}  ->  {new_obs.max_x:.1f}]'
        )
        self.get_logger().info(
            f'Limiti Y (Altezza)    : [{new_obs.min_y:.1f}  ->  {new_obs.max_y:.1f}]'
        )
        self.get_logger().info(
            f'Limiti Z (Profondità) : [{new_obs.min_z:.1f}  ->  {new_obs.max_z:.1f}]'
        )

        # Catena completa della rotta futura:
        # Parte DALLA POSIZIONE ATTUALE DEL DRONE -> fino a self.path[current_wp_index] -> fino a fine percorso
        route_points = [self.drone_pos] + [
            (wp[0], wp[1], wp[2]) for wp in self.path[self.current_wp_index :]
        ]

        collision = False
        wp_intersecato = None

        for i in range(len(route_points)):
            p_curr = route_points[i]

            # 1. Controlla se il vertice cade dentro l'ostacolo
            if new_obs.is_inside(p_curr[0], p_curr[1], p_curr[2]):
                collision = True
                wp_intersecato = p_curr
                break

            # 2. Controlla il segmento tra p_curr e il punto successivo (passo di 1.0 metro)
            if i < len(route_points) - 1:
                p_next = route_points[i + 1]
                seg_len = math.sqrt(
                    (p_next[0] - p_curr[0]) ** 2
                    + (p_next[1] - p_curr[1]) ** 2
                    + (p_next[2] - p_curr[2]) ** 2
                )
                num_steps = max(5, int(seg_len / 1.0))

                for step in range(1, num_steps):
                    t = step / float(num_steps)
                    mx = p_curr[0] + t * (p_next[0] - p_curr[0])
                    my = p_curr[1] + t * (p_next[1] - p_curr[1])
                    mz = p_curr[2] + t * (p_next[2] - p_curr[2])
                    if new_obs.is_inside(mx, my, mz):
                        collision = True
                        wp_intersecato = (mx, my, mz)
                        break
                if collision:
                    break

        if collision:
            self.blocking_obs_pub.publish(msg)
            self.get_logger().warning(
                f'💥 COLLISIONE PREVISTA sul punto (X:{wp_intersecato[0]:.1f}, Y:{wp_intersecato[1]:.1f}, Z:{wp_intersecato[2]:.1f})!'
            )
            self.get_logger().warning("⚠️ Frenata d'emergenza in corso...")
            self.is_replanning = True

            if self.current_goal_handle:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)
            else:
                self.execute_replan()
        else:
            self.get_logger().info(
                "✅ L'ostacolo è stato aggiunto alla mappa, ma NON tocca la rotta futura. Il drone continua la missione.\n"
            )
    def cancel_done_callback(self, future):
        self.get_logger().info('Drone fermato con successo. Procedo al ricalcolo...')
        self.execute_replan()

    def execute_replan(self):
        """Ricalcola Dijkstra partendo dall'esatto punto in cui il drone ha frenato"""
        self.get_logger().info(
            f'Avvio ricalcolo da {self.drone_pos} verso {self.final_goal}...'
        )

        new_path = self.planner.dijkstra(
            self.drone_pos, self.final_goal, initial_yaw=self.drone_yaw
        )

        if not new_path:
            self.get_logger().error(
                'CRITICO: Nessuna deviazione possibile! Mi arrendo.'
            )
            rclpy.shutdown()
            return

        self.path = new_path
        self.current_wp_index = 0
        self.is_replanning = False
        self.get_logger().info(
            f'Nuova variante trovata ({len(self.path)} WP). Riprendo missione!'
        )
        self.send_next_waypoint()

    def start_mission(self):
        self.get_logger().info('In attesa del server Action "move_robot"...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server connesso! Inizio missione.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.is_replanning:
            return  # Blocca l'invio se stiamo ricalcolando

        if self.current_wp_index >= len(self.path):
            self.get_logger().info('MISSIONE COMPLETATA!')
            rclpy.shutdown()
            return

        target = self.path[self.current_wp_index]
        target_theta = target[3]

        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        goal_msg.target_rotation = float(target_theta)

        self.get_logger().info(
            f'-> WP [{self.current_wp_index + 1}/{len(self.path)}]: '
            f'Pos({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) | Yaw({math.degrees(target_theta):.1f}°)'
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().error('Il server ha rifiutato il target!')
            return

        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.is_replanning:
            return  # Se stiamo ricalcolando, ignoriamo la notifica di azione abortita

        result = future.result().result
        status = future.result().status

        if status == 4 and result.success:
            self.get_logger().info('OK! Waypoint raggiunto.')
            self.drone_pos = (
                result.final_position.x,
                result.final_position.z,
                result.final_position.y,
            )
            self.drone_yaw = self.path[self.current_wp_index][3]
            self.current_wp_index += 1
            self.send_next_waypoint()
        else:
            self.get_logger().error(f'Errore nel raggiungimento del waypoint. Status: {status}')
            rclpy.shutdown()


def get_real_start_pos():
    """Ascolta la rete ROS 2 per 1 secondo e cattura la posizione esatta del drone"""
    temp_node = rclpy.create_node('temp_pos_reader')
    drone_pos = None

    def pos_callback(msg):
        nonlocal drone_pos
        drone_pos = (msg.x, msg.z, msg.y)

    # Si iscrive al topic 'drone_position' pubblicato dal file Server
    temp_node.create_subscription(Vector3, 'drone_position', pos_callback, 10)

    print("In attesa della posizione reale dal Server ROS 2...")
    while drone_pos is None and rclpy.ok():
        rclpy.spin_once(temp_node)

    temp_node.destroy_node()
    return drone_pos


def main(args=None):
    rclpy.init(args=args)
    width = 20
    # 1. SETUP DEGLI OSTACOLI STATICI
    obstacles = [
        #Dritto   1
        ObstacleAABB(x=-46, y=20.0, z=-75.0, width=width, depth=60, height=200, drone_radius=5.0),
        #Rovesciato    2
        ObstacleAABB(x=-126.0, y=20.0, z=-173.0, width=200, depth=60.0, height=width, drone_radius=5.0),
        #Dritto    3
        ObstacleAABB(x=45.0, y=20.0, z=-75.0, width=width, depth=60.0, height=200, drone_radius=5.0),
        #Rovesciato    4
        ObstacleAABB(x=-68.0, y=20.0, z=-233.0, width=200, depth=60.0, height=width, drone_radius=5.0),
        #Dritto    5
        ObstacleAABB(x=-243.0, y=20.0, z=-269.0, width=width, depth=60.0, height=200.0, drone_radius=5.0),
        #Rovesciato    6
        ObstacleAABB(x=-151.0, y=20.0, z=-390.0, width=200, depth=60.0, height=width, drone_radius=5.0),
        #Rovesciato 7
        ObstacleAABB(x=-73.0, y=20.0, z=-315.0, width=200, depth=60.0, height=width, drone_radius=5.0),
        #Dritto  8
        ObstacleAABB(x=-161.0, y=20.0, z=-270.0, width=width, depth=60.0, height=60.0, drone_radius=5.0),
        #Rovesciato   9
        ObstacleAABB(x=84.0, y=20.0, z=-160.0, width=60.0, depth=60.0, height=width, drone_radius=5.0),
        #Dritto    10
        ObstacleAABB(x=104.0, y=20.0, z=-202.0, width=width, depth=60.0, height=60.0, drone_radius=5.0),
        #Rovesciato    11
        ObstacleAABB(x=63.0, y=20.0, z=-239.0, width=60.0, depth=60.0, height=width, drone_radius=5.0),
        # Dritto   12
        ObstacleAABB(x=-48.0, y=20.0, z=-427.0, width=width, depth=60.0, height=60.0, drone_radius=5.0),
        # Rovesciato    13
        ObstacleAABB(x=-9.0, y=20.0, z=-458.0, width=60.0, depth=60.0, height=width, drone_radius=5.0),
        # Rovesciato    14
        ObstacleAABB(x=60, y=20.0, z=-458.0, width=60.0, depth=60.0, height=width, drone_radius=5.0),
        # Dritto    15
        ObstacleAABB(x=104.0, y=20.0, z=-427.0, width=width, depth=60.0, height=60.0, drone_radius=5.0),
        # Dritto     16
        ObstacleAABB(x=104.0, y=20.0, z=-356.0, width=width, depth=60.0, height=60.0, drone_radius=5.0),
        # Rovesciato    17
        ObstacleAABB(x=67.0, y=20.0, z=-315.0, width=60.0, depth=60.0, height=width, drone_radius=5.0),


    ]

    bounds = {
        'x': (-10000.0, 10000.0),
        'y': (0.0, 45.0),
        'z': (-10000.0, 10000.0)
    }

    planner = PathPlanner(bounds=bounds, resolution=15, obstacles=obstacles)

    # 2. CATTURA LA POSIZIONE REALE VIA ROS 2
    start_pos = get_real_start_pos()
    print(f"Posizione reale del drone acquisita: {start_pos}")

    goal_pos = (26, 15.0, -404)
    #goal_pos = (0.0, 0.0, 0.0)

    # 3. CALCOLO DIJKSTRA
    print(f"Calcolo percorso con Dijkstra da {start_pos} a {goal_pos}...")
    path = planner.dijkstra(start_pos, goal_pos, initial_yaw=0.0)

    if not path:
        print("ERRORE: Nessun percorso valido trovato.")
        return

    print(f"Percorso trovato! Composto da {len(path)} waypoint.")

    # 4. AVVIO DEL CLIENT
    # 4. AVVIO DEL CLIENT (passiamo anche planner e goal_pos)
    client = AutonomousDroneClient(planner, goal_pos, path, start_pos=start_pos, start_yaw=0.0)
    client.start_mission()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()