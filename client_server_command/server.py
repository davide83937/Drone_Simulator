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

    def is_only_rotation(self, target_pos):
        # Calcolo distanza iniziale per capire se il waypoint è di ROTAZIONE o TRASLAZIONE
        dx_init = target_pos.x - self.state.pos_x
        dy_init = target_pos.y - self.state.pos_z
        dz_init = target_pos.z - self.state.pos_y
        init_distance = math.sqrt(dx_init ** 2 + dy_init ** 2 + dz_init ** 2)

        # Se le coordinate target sono quelle attuali del drone (< 1.0m), è SOLO ROTAZIONE
        is_rotation_only = (init_distance < 1.0)
        return is_rotation_only

    def launch_command(self, is_rotation_only, angle_target, target_pos, z_end):
        if is_rotation_only:
            # SOLO ROTAZIONE: blocca X, Y, Z sulla posizione attuale, ruota verso angle_target
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=self.state.pos_z,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=-self.state.pos_y, x_end=self.state.pos_x,
                ang_start=self.state.yaw_magnetometer, ang_end=angle_target
            )
        else:
            # SOLO TRASLAZIONE: muove X, Y, Z verso il target, tiene fisso angle_target
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=target_pos.y,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=z_end, x_end=target_pos.x,
                ang_start=angle_target, ang_end=angle_target
            )
    def difference_distance_compute(self, target_pos, angle_target, loop_counter):
        dx = target_pos.x - self.state.pos_x
        dy = target_pos.y - self.state.pos_z
        dz = target_pos.z - self.state.pos_y

        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        target_deg = math.degrees(angle_target)
        # Calcolo distanza rotazionale in GRADI puliti (da 0° a 180°)
        diff_angolo = target_deg - self.state.yaw_magnetometer
        distance_rotation = abs((diff_angolo + 180.0) % 360.0 - 180.0)
        if loop_counter % 10 == 0:
            self.get_logger().info(
                f'Distanza totale: {distance:.2f}m (Soglia <= 20.0) | '
                f'Errore rot: {math.degrees(distance_rotation):.1f}° (Soglia <= 5.0)\n'
                f'       -> dx: {dx:.2f} | dy: {dy:.2f} | dz: {dz:.2f}'
            )
        return distance, distance_rotation

    def execute_callback(self, goal_handle):
        self.get_logger().info('Nuovo Goal ricevuto dal client!')
        target_pos = goal_handle.request.target_position
        target_rot = goal_handle.request.target_rotation

        angle_target = float(target_rot)
        z_end = -target_pos.z


        is_rotation_only = self.is_only_rotation(target_pos)
        self.launch_command(is_rotation_only, angle_target, target_pos, z_end)

        feedback_msg = MoveAxis.Feedback()
        loop_counter = 0

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning('Azione cancellata dal client!')
                result = MoveAxis.Result()
                result.success = False
                return result

            # Calcolo distanza lineare (identico all'originale)
            distance, distance_rotation = self.difference_distance_compute(target_pos, angle_target, loop_counter)
            if is_rotation_only:
                if distance_rotation <= 5.0:  # 5 gradi reali di tolleranza
                    break
            else:
                if distance <= 10.0 and distance_rotation <= 5.0:
                    break

            feedback_msg.current_position = Vector3(
                x=float(self.state.pos_x),
                y=float(self.state.pos_y),
                z=float(self.state.pos_z)
            )
            feedback_msg.current_rotation = float(self.state.yaw_magnetometer)

            if distance > 0:
                #Viene inviato all'Action Client ROS 2, ovvero alla classe AutonomousDroneClient in PathManager.py.
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