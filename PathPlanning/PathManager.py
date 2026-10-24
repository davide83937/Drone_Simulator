import os
import math
import json
from std_msgs.msg import String
from PathPlanning.getObstacles import get_obstacles_path_manager
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from PathPlanning.obstacle_config import ObstacleConfig
from custom_interfaces.action import MoveAxis
from geometry_msgs.msg import Vector3
from Obstacle import PathPlanner, ObstacleAABB


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

    def _register_new_obstacle(self, json_str):
        """1. Converte il JSON, crea l'ostacolo AABB, lo aggiunge al planner e stampa i log."""
        data = json.loads(json_str)

        new_obs = ObstacleAABB(
            x=data['x'],
            y=data['z'],
            z=data['y'],
            width=data['w'],
            depth=data['h'],
            height=data['d'],
            drone_radius=ObstacleConfig.get_drone_safety_radius(),
        )
        self.planner.obstacles.append(new_obs)

        self.get_logger().info(
            f'\n--- VALUTAZIONE NUOVO OSTACOLO ---\n'
            f'Posizione attuale drone: X:{self.drone_pos[0]:.1f}, Y(Alt):{self.drone_pos[1]:.1f}, Z(Prof):{self.drone_pos[2]:.1f}\n'
            f'Limiti X (Orizzontale): [{new_obs.min_x:.1f} -> {new_obs.max_x:.1f}]\n'
            f'Limiti Y (Altezza)    : [{new_obs.min_y:.1f} -> {new_obs.max_y:.1f}]\n'
            f'Limiti Z (Profondità) : [{new_obs.min_z:.1f} -> {new_obs.max_z:.1f}]'
        )
        return new_obs

    def _find_route_collision(self):
        """2. Controlla la rotta futura. Restituisce il punto di impatto o None se la via è libera."""
        route_points = [self.drone_pos] + [
            (wp[0], wp[1], wp[2]) for wp in self.path[self.current_wp_index:]
        ]

        for i, p_curr in enumerate(route_points):
            # Controlla il vertice esatto
            if not self.planner.is_valid(*p_curr):
                return p_curr

            # Controlla il segmento verso il punto successivo
            if i < len(route_points) - 1:
                p_next = route_points[i + 1]
                if not self.planner.is_segment_valid(*p_curr, *p_next):
                    return p_curr

        return None

    def _trigger_emergency_stop(self, msg, wp_intersecato):
        """3. Gestisce l'allarme grafico e blocca immediatamente il movimento in corso."""
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
            self.execute_backup()

    def obstacle_callback(self, msg):
        """Regista principale: coordina registrazione, verifica e reazione."""
        if self.is_replanning or not self.path or self.current_wp_index >= len(self.path):
            return

        # 1. Registra l'ostacolo nella mappa
        self._register_new_obstacle(msg.data)

        # 2. Controlla se intralcia la rotta rimanente
        wp_intersecato = self._find_route_collision()

        # 3. Agisce di conseguenza
        if wp_intersecato is not None:
            self._trigger_emergency_stop(msg, wp_intersecato)
        else:
            self.get_logger().info(
                "✅ L'ostacolo è stato aggiunto alla mappa, ma NON tocca la rotta futura. Il drone continua la missione.\n"
            )

    def cancel_done_callback(self, future):
        """Si attiva appena il server conferma che i motori sono fermi."""
        self.get_logger().info('🛑 Drone fermato con successo. Eseguo retromarcia di sicurezza...')
        self.execute_backup()

    def execute_backup(self):
        """Manda il drone all'indietro rispetto al suo muso PRIMA di calcolare Dijkstra."""
        distanza_ritirata = ObstacleConfig.get_emergency_backup_distance()  # Metri di retromarcia

        back_x = self.drone_pos[0] #- math.sin(self.drone_yaw) * distanza_ritirata
        back_z = self.drone_pos[2] #+ math.cos(self.drone_yaw) * distanza_ritirata
        back_y = self.drone_pos[1]  # Mantiene la quota

        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=float(back_x), y=float(back_y), z=float(back_z))
        goal_msg.target_rotation = float(self.drone_yaw)

        self.get_logger().info(f'🔄 Retromarcia di {distanza_ritirata}m verso X:{back_x:.1f}, Z:{back_z:.1f}...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.backup_response_callback)

    def backup_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Retromarcia rifiutata dal server! Ricalcolo dalla posizione attuale.')
            self.execute_replan()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.backup_result_callback)

    def backup_result_callback(self, future):
        """Si attiva al termine della retromarcia per far partire Dijkstra in sicurezza."""
        self.get_logger().info("✅ Retromarcia completata. Procedo con il ricalcolo del percorso...")
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
    # 1. SETUP DEGLI OSTACOLI STATICI
    obstacles = get_obstacles_path_manager()

    bounds = {
        'x': (-800.0, 800.0),
        'y': (0.0, 45.0),
        'z': (-800.0, 800.0)
    }

    planner = PathPlanner(bounds=bounds, resolution=15, obstacles=obstacles)

    # 2. CATTURA LA POSIZIONE REALE VIA ROS 2
    # 2. CATTURA LA POSIZIONE REALE VIA ROS 2
    start_pos = get_real_start_pos()
    print(f"Posizione reale del drone acquisita: {start_pos}")

    # --- SETUP QUOTA DI VOLO ---
    quota_volo = 15.0
    # Creiamo un punto fittizio per far credere a Dijkstra che siamo già in aria
    pos_in_aria = (start_pos[0], quota_volo, start_pos[2])

    goal_pos = (26.0, quota_volo, -404.0)

    # 3. CALCOLO DIJKSTRA (Calcola il percorso partendo già dalla quota di 15.0)
    print(f"Calcolo percorso con Dijkstra da {pos_in_aria} a {goal_pos}...")
    path = planner.dijkstra(pos_in_aria, goal_pos, initial_yaw=0.0)

    if not path:
        print("ERRORE: Nessun percorso valido trovato.")
        return

    # --- INIEZIONE DECOLLO MANUALE ---
    # Inseriamo come primissimo comando (indice 0) la salita verticale in stile ascensore.
    # Il drone manterrà le sue X e Z attuali, ma salirà a Y = 15.0 con angolo 0.0
    wp_decollo = (start_pos[0], quota_volo, start_pos[2], 0.0)
    path.insert(0, wp_decollo)

    print(f"Percorso trovato! Composto da {len(path)} waypoint (incluso decollo verticale).")

    # 4. AVVIO DEL CLIENT
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