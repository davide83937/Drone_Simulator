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


    def traslate_callback(self, goal_handle):
        target_position = goal_handle.request.target_position
        print(f'target_position: {target_position}')
        self.launch_command_traslate(target_position.x, target_position.z)
        result = TraslateCommand.Result()
        while True:
            dist = math.hypot(target_position.x - self.state.pos_x, target_position.z - self.state.pos_y)
            if dist <= 5:
                goal_handle.succeed()
                result.success = True
                return result
            time.sleep(0.05)


    def stop_callback(self, goal_handle):
        target = goal_handle.request.emergency_stop
        result = StopCommand.Result()
        result.success = False
        if target:
            self.launch_command_stop()
            goal_handle.succeed()
            result = StopCommand.Result()
            result.success = True
        return result

    def unidimensional_callback(self, goal_handle):
        target = goal_handle.request.target_value

        type = goal_handle.request.axis_type
        result = UnidimensionalCommand.Result()
        if type == 1: #ALTITUDE
            self.launch_command_altitude(target)
            while True:

                if abs(target - self.state.pos_z) <= 0.50:

                    goal_handle.succeed()
                    result.success = True
                    return result
                time.sleep(0.05)
        elif type == 2: #ROTATION
            self.launch_command_rotation(target)
            while True:
                err = abs((target - self.state.yaw_magnetometer + 180.0) % 360.0 - 180.0)
                if err <= 2.0:
                    goal_handle.succeed()
                    result.success = True
                    return result
                time.sleep(0.05)


    def launch_command_traslate(self, target_x, target_y):
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=self.state.pos_z,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=-target_y, x_end=target_x,
                ang_start=math.radians(self.state.yaw_magnetometer), ang_end=math.radians(self.state.yaw_magnetometer)
            )

    def launch_command_rotation(self, target_rotation):
            target_rotation= math.radians(target_rotation)
            print(f"target_rotation: {target_rotation}")
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=self.state.pos_z,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=-self.state.pos_y, x_end=self.state.pos_x,
                ang_start=math.radians(self.state.yaw_magnetometer), ang_end=target_rotation,
            )

    def launch_command_altitude(self, target_altitude):
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=target_altitude,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=-self.state.pos_y, x_end=self.state.pos_x,
                ang_start=math.radians(self.state.yaw_magnetometer), ang_end=math.radians(self.state.yaw_magnetometer),
            )

    def launch_command_stop(self):
            self.drone_control_scheme.start(
                y_start=self.state.pos_z, y_end=self.state.pos_z,
                z_start=-self.state.pos_y, x_start=self.state.pos_x,
                z_end=-self.state.pos_y, x_end=self.state.pos_x,
                ang_start=math.radians(self.state.yaw_magnetometer), ang_end=math.radians(self.state.yaw_magnetometer),
            )



