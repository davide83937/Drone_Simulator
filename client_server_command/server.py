import time
import math
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from control_scheme.control_scheme import droneControlScheme
from custom_interfaces.action import MoveAxis
from custom_interfaces.action import TraslateCommand
from custom_interfaces.action import StopCommand
from custom_interfaces.action import UnidimensionalCommand
from geometry_msgs.msg import Vector3
from control_scheme.state import State


class MoveRobotServer(Node):
    def __init__(self, state: State, drone_control_scheme: droneControlScheme):
        self.state = state
        self.drone_control_scheme = drone_control_scheme
        super().__init__('move_robot')
        self.cb_group = ReentrantCallbackGroup()
        #self._action_server = ActionServer(self, MoveAxis, 'move_robot', execute_callback=self.execute_callback)
        self._action_traslate_server = ActionServer(self, TraslateCommand, 'traslate', execute_callback=self.traslate_callback, callback_group=self.cb_group)
        self._action_stop_server = ActionServer(self, StopCommand, 'stop',execute_callback=self.stop_callback, callback_group=self.cb_group)
        self._action_unidimensional_server = ActionServer(self, UnidimensionalCommand, 'uni',execute_callback=self.unidimensional_callback, callback_group=self.cb_group)

        self.get_logger().info('Action Server "move_axis" avviato e in ascolto...')


    #def execute_callback(self):
    #    pass


    def traslate_callback(self):
        pass

    def stop_callback(self):
        pass

    def unidimensional_callback(self):
        pass

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

