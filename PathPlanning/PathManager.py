import os
import math
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
    def __init__(self, path, start_pos=(0.0, 15.0, 0.0), start_yaw=0.0):
        super().__init__('autonomous_drone_client')
        self._action_client = ActionClient(self, MoveAxis, 'move_robot')
        self.path = path
        self.current_wp_index = 0
        self.drone_pos = start_pos
        self.drone_yaw = start_yaw
        self.mission_completed = False

    def start_mission(self):
        self.get_logger().info('In attesa del server Action "move_robot"...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server connesso! Inizio missione.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        # Filtro per saltare waypoint già soddisfatti
        while self.current_wp_index < len(self.path) - 1:
            wp = self.path[self.current_wp_index]
            distanza_wp = math.sqrt((wp[0] - self.drone_pos[0]) ** 2 + (wp[2] - self.drone_pos[2]) ** 2)

            # Differenza angolare normalizzata [-pi, pi]
            delta_yaw = abs((wp[3] - self.drone_yaw + math.pi) % (2 * math.pi) - math.pi)

            # Salta solo se sia la posizione che l'angolo sono già stati raggiunti
            if distanza_wp < 2.0 and delta_yaw < math.radians(10.0):
                self.current_wp_index += 1
            else:
                break

        if self.current_wp_index >= len(self.path):
            self.get_logger().info('MISSIONE COMPLETATA!')
            self.mission_completed = True
            rclpy.shutdown()
            return

        target = self.path[self.current_wp_index]
        target_theta = target[3]  # L'angolo è già disaccoppiato e memorizzato nel percorso

        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        goal_msg.target_rotation = float(target_theta)

        # Identifichiamo nei log se è un'azione di pura rotazione o traslazione
        prev_pos = self.path[self.current_wp_index - 1][:3] if self.current_wp_index > 0 else self.drone_pos
        is_pure_rotation = math.isclose(target[0], prev_pos[0], abs_tol=0.01) and \
                           math.isclose(target[2], prev_pos[2], abs_tol=0.01)

        tipo_azione = "RUOTA SUL POSTO" if is_pure_rotation else "TRASLA LINEARE"
        self.get_logger().info(
            f'-> [{tipo_azione}] WP: Pos({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) | Yaw({math.degrees(target_theta):.1f}°)')

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Il server ha rifiutato il target!')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        #self.get_logger().info(
        #    f'POSIZIONE REALE DRONE -> X: {fb.current_position.x:.2f} | '
        #    f'Y: {fb.current_position.y:.2f} | '
        #    f'Z: {fb.current_position.z:.2f} || '
        #    f'Yaw: {fb.current_rotation:.2f} rad'
        #)
        pass

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        # Status 4 = SUCCEEDED in ROS 2
        if status == 4 and result.success:
            self.get_logger().info(f'OK! Waypoint raggiunto.')
            # Aggiorna la posizione e l'orientamento correnti con quelli raggiunti dal robot
            self.drone_pos = (result.final_position.x, result.final_position.y, result.final_position.z)
            self.drone_yaw = self.path[self.current_wp_index][3]

            self.current_wp_index += 1
            self.send_next_waypoint()
        else:
            self.get_logger().error(f'Errore nel raggiungimento del waypoint. Status: {status}, Msg: {result.message}')
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
    obstacles = [
        ObstacleAABB(x=0.0, y=0.0, z=-40.0, width=8.0, depth=200.0, height=8.0, drone_radius=7.0),
        ObstacleAABB(x=-10.0, y=0.0, z=-50.0, width=8.0, depth=200.0, height=8.0, drone_radius=7.0),
        ObstacleAABB(x=10.0, y=0.0, z=-50.0, width=8.0, depth=200.0, height=8.0, drone_radius=7.0),
        ObstacleAABB(x=25.0, y=0.0, z=-50.0, width=12.0, depth=200.0, height=12.0, drone_radius=7.0),
        ObstacleAABB(x=-25.0, y=0.0, z=-50.0, width=12.0, depth=200.0, height=12.0, drone_radius=7.0)
    ]

    bounds = {
        'x': (-100.0, 100.0),
        'y': (-100.0, 100.0),
        'z': (-100.0, 100.0)
    }

    planner = PathPlanner(bounds=bounds, resolution=5, obstacles=obstacles)

    # 2. CATTURA LA POSIZIONE REALE VIA ROS 2
    start_pos = get_real_start_pos()
    print(f"Posizione reale del drone acquisita: {start_pos}")

    goal_pos = (-10.0, 15.0, -80.0)
    #goal_pos = (0.0, 0.0, 0.0)

    # 3. CALCOLO DIJKSTRA
    print(f"Calcolo percorso con Dijkstra da {start_pos} a {goal_pos}...")
    path = planner.dijkstra(start_pos, goal_pos, initial_yaw=0.0)

    if not path:
        print("ERRORE: Nessun percorso valido trovato.")
        return

    print(f"Percorso trovato! Composto da {len(path)} waypoint.")

    # 4. AVVIO DEL CLIENT
    client = AutonomousDroneClient(path, start_pos=start_pos, start_yaw=0.0)
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