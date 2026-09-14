import time
from rclpy.action import ActionServer
from rclpy.node import Node

from control_scheme.control_scheme import droneControlScheme
from custom_interfaces.action import MoveAxis
from geometry_msgs.msg import Vector3
from control_scheme.state import State
import math


class MoveRobotServer(Node):
    def __init__(self, state:State, drone_control_scheme:droneControlScheme):
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
        self.get_logger().info('Goal reached')
        target_pos = goal_handle.request.target_position
        target_rot = goal_handle.request.target_rotation

        angle_target = math.radians(target_rot)

        z_end = target_pos.z
        z_end = -z_end


        #self.drone_control_scheme.reset_pid()
        self.drone_control_scheme.start(
            y_start=self.state.pos_y, y_end=target_pos.y,
            z_start=self.state.pos_z, x_start=self.state.pos_x,
            z_end=z_end, x_end=target_pos.x,
            ang_start=self.state.yaw_magnetometer, ang_end=angle_target
        )



        feedback_msg = MoveAxis.Feedback()
        feedback_msg.current_position = Vector3(x=self.state.pos_x, y=self.state.pos_y, z=self.state.pos_z)
        feedback_msg.current_rotation = self.state.yaw_magnetometer
        feedback_msg.progress_percentage = 0.0
        distance = abs(math.sqrt(feedback_msg.current_position.x ** 2 + feedback_msg.current_position.y ** 2 + feedback_msg.current_position.z ** 2) - math.sqrt(target_pos.x ** 2 + target_pos.y ** 2 + target_pos.z ** 2))
        distance_rotation = abs(feedback_msg.current_rotation - target_rot)

        while distance > 5 and distance_rotation > 2:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning('Azione cancellata dal client!')
                result = MoveAxis.Result()
                result.success = False
                result.final_position = Vector3(x=self.state.pos_x, y=self.state.pos_y, z=self.state.pos_z)
                result.final_rotation = self.state.yaw_magnetometer
                result.message = "Operazione interrotta prima del completamento."
                return result

            # Distanza euclidea dal target
            dx = target_pos.x - self.state.pos_x
            dy = target_pos.y - self.state.pos_y
            dz = target_pos.z - self.state.pos_z
            distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            distance_rotation = abs(target_rot - self.state.yaw_magnetometer)

            # Condizione di uscita (interrompi il loop se ENTRAMBI i target sono stati raggiunti)
            if distance <= 5.0 and distance_rotation <= 2.0:
                break

            # Aggiornamento feedback
            feedback_msg.current_position = Vector3(x=float(self.state.pos_x), y=float(self.state.pos_y),
                                                    z=float(self.state.pos_z))
            feedback_msg.current_rotation = float(self.state.yaw_magnetometer)

            # Previene divisioni per 0 se la distanza iniziale è nulla
            if distance > 0:
                feedback_msg.progress_percentage = float(max(0.0, 100.0 - (distance * 2)))

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = MoveAxis.Result()
        result.success = True
        result.final_position = Vector3(x=self.state.pos_x, y=self.state.pos_y, z=self.state.pos_z)
        result.final_rotation = self.state.yaw_magnetometer
        result.message = "Target raggiunto con successo."
        self.get_logger().info('Azione completata con successo!')
        return result

