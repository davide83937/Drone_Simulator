import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node

from control_scheme.control_scheme import droneControlScheme
from custom_interfaces.action import MoveAxis
from geometry_msgs.msg import Vector3
from control_scheme.state import State


class MoveRobotServer(Node):
    def __init__(self, state: State, drone_control_scheme: droneControlScheme):
        self.state = state
        self.drone_control_scheme = drone_control_scheme
        super().__init__('move_robot')
        self._action_server = ActionServer(
            self,
            MoveAxis,
            'move_robot',
            execute_callback=self.execute_callback
        )
        self.get_logger().info('Action Server "move_axis" avviato e in ascolto...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Nuovo Goal ricevuto dal client!')
        target_pos = goal_handle.request.target_position
        target_rot = goal_handle.request.target_rotation

        angle_target = target_rot
        z_end = target_pos.z
        z_end = -z_end

        # Avvio del controller con il tuo incrocio di assi
        self.drone_control_scheme.start(
            y_start=self.state.pos_z, y_end=target_pos.y,
            z_start=-self.state.pos_y, x_start=self.state.pos_x,
            z_end=z_end, x_end=target_pos.x,
            ang_start=self.state.yaw_magnetometer, ang_end=angle_target
        )

        feedback_msg = MoveAxis.Feedback()
        loop_counter = 0  # Contatore per non intasare il terminale con i log

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning('Azione cancellata dal client!')
                result = MoveAxis.Result()
                result.success = False
                return result

            # Calcolo distanza lineare
            dx = target_pos.x - self.state.pos_x
            dy = target_pos.y - self.state.pos_z
            dz = (target_pos.z) - self.state.pos_y

            distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            #distance = math.sqrt(dx ** 2 + dz ** 2)

            # Calcolo distanza rotazionale (Normalizzata tra -Pi e Pi per evitare blocchi)
            diff_angolo = angle_target - self.state.yaw_magnetometer
            distance_rotation = abs((diff_angolo + math.pi) % (2 * math.pi) - math.pi)

            # Condizione di uscita
            if distance <= 5 and distance_rotation < 5:
                break

            # DEBUG: Stampa a schermo ogni 10 cicli (1 secondo) i valori matematici
                # DEBUG: Stampa a schermo ogni 10 cicli (circa 1 secondo) i valori matematici
            if loop_counter % 10 == 0:
                self.get_logger().info(
                    f'Distanza totale: {distance:.2f}m (Soglia <= 1.0) | Errore rot: {distance_rotation:.2f}rad\n'
                    f'       -> dx: {dx:.2f} | dy: {dy:.2f} | dz: {dz:.2f}'
                )

            # Aggiornamento feedback
            feedback_msg.current_position = Vector3(
                x=float(self.state.pos_x),
                y=float(self.state.pos_y),
                z=float(self.state.pos_z)
            )
            feedback_msg.current_rotation = float(self.state.yaw_magnetometer)

            # Corretto l'errore di sintassi inserendo il segno meno "-"
            if distance > 0:
                feedback_msg.progress_percentage = float(max(0.0, 100.0 - (distance * 2)))

            goal_handle.publish_feedback(feedback_msg)

            loop_counter += 1
            time.sleep(0.1)

        goal_handle.succeed()

        result = MoveAxis.Result()
        result.success = True
        result.final_position = Vector3(x=self.state.pos_x, y=self.state.pos_y, z=self.state.pos_z)
        result.final_rotation = self.state.yaw_magnetometer
        result.message = "Target raggiunto con successo."
        self.get_logger().info('Waypoint raggiunto, richiedo il prossimo!')

        return result