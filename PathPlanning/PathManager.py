import os
import math

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
    def __init__(self, path):
        super().__init__('autonomous_drone_client')
        self._action_client = ActionClient(self, MoveAxis, 'move_robot')
        self.path = path
        self.current_wp_index = 0

    def start_mission(self):
        self.get_logger().info('In attesa del server Action "move_robot"...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server connesso! Inizio missione.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_wp_index >= len(self.path):
            self.get_logger().info('MISSIONE COMPLETATA! Tutti i waypoint sono stati raggiunti.')
            rclpy.shutdown()
            return

        target = self.path[self.current_wp_index]

        # Calcola la rotazione verso il prossimo waypoint (Yaw)
        target_theta = 0.0
        if self.current_wp_index < len(self.path) - 1:
            next_target = self.path[self.current_wp_index + 1]
            target_theta = math.atan2(next_target[0] - target[0], -(next_target[2] - target[2]))

        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        goal_msg.target_rotation = float(target_theta)

        self.get_logger().info(f'-> Invio WP {self.current_wp_index + 1}/{len(self.path)}: '
                               f'Target({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) '
                               f'Yaw: {target_theta:.2f} rad')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
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
            self.get_logger().info(f'OK! Waypoint raggiunto. (Posizione finale server: '
                                   f'{result.final_position.x:.2f}, '
                                   f'{result.final_position.y:.2f}, '
                                   f'{result.final_position.z:.2f})')

            # Passa al prossimo punto
            self.current_wp_index += 1
            self.send_next_waypoint()
        else:
            self.get_logger().error(f'Errore nel raggiungimento del waypoint. Status: {status}, Msg: {result.message}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # 1. SETUP DEGLI OSTACOLI E DEL GRAFO
    obstacles = [
        ObstacleAABB(x=0.0, y=0.0, z=-40.0, width=8.0, depth=200.0, height=8.0, drone_radius=4.0)
    ]

    bounds = {
        'x': (-100.0, 100.0),
        'y': (-100.0, 100.0),
        'z': (-100.0, 100.0)
    }

    planner = PathPlanner(bounds=bounds, resolution=10, obstacles=obstacles)

    start_pos = (0.0, 15.0, 0.0)
    goal_pos = (0.0, 15.0, -80.0)

    print(f"Calcolo percorso con Dijkstra da {start_pos} a {goal_pos}...")
    path = planner.dijkstra(start_pos, goal_pos)

    if not path:
        print("ERRORE: Nessun percorso valido trovato.")
        return

    print(f"Percorso trovato! Composto da {len(path)} waypoint.")

    # 2. AVVIO DEL CLIENT AUTONOMO ROS 2
    client = AutonomousDroneClient(path)
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