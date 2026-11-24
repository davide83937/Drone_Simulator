import os

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from PathPlanning.PantoPlanning import PantoPlanning, vertex, angle, altitude
from custom_interfaces.action import TraslateCommand
from custom_interfaces.action import StopCommand
from custom_interfaces.action import UnidimensionalCommand




# --- CONFIGURAZIONE AMBIENTE ROS 2 WINDOWS ---
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
os.environ["AMENT_PREFIX_PATH"] = prefix_path



class WaypointSender(Node):
    def __init__(self, panto_planning:PantoPlanning):
        super().__init__('waypoint_sender')
        self.cb_group = ReentrantCallbackGroup()
        self._action_traslate_client = ActionClient(self, TraslateCommand, 'traslate', callback_group=self.cb_group)
        self._action_stop_client = ActionClient(self, StopCommand, 'stop', callback_group=self.cb_group)
        self._action_unidimensional_client = ActionClient(self, UnidimensionalCommand, 'uni', callback_group=self.cb_group)
        self.create_subscription(Vector3, 'drone_position', self.pos_callback, 10)
        self.create_subscription(Vector3, 'drone_rotation', self.rot_callback, 10)
        self.panto_planning = panto_planning
        self.index = 0
        self.index_traslate = 0
        self.drone_pos = np.array([0, 0, 0])
        self.drone_yaw = 0


    def pos_callback(self, msg):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])

    def rot_callback(self, msg):
        self.drone_yaw = np.radians(msg.z)

    def get_pos(self):
        return self.drone_pos
    def get_yaw(self):
        return self.drone_yaw

    def send_traslate(self, x, z):
        self._action_traslate_client.wait_for_server()
        goal_msg = TraslateCommand.Goal()
        goal_msg.target_position = Vector3(x=float(x), y=0.0, z=float(z))
        print(f"Sending traslate command: x={x}, z={z}")
        send_goal_future = self._action_traslate_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_accepted)

    def send_stop(self):
        self._action_stop_client.wait_for_server()
        goal_msg = StopCommand.Goal()
        goal_msg.emergency_stop = True
        goal_msg.reason = ""
        send_goal_future = self._action_stop_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_stop_accepted)


    def send_unidimensional(self, type, num):
        self._action_unidimensional_client.wait_for_server()
        goal_msg = UnidimensionalCommand.Goal()
        if type == "rotate":
            goal_msg.axis_type=UnidimensionalCommand.Goal.AXIS_ROTATION
            print(f"Sending unidimensional command: axis_type={type}, num={num}")
        elif type == "altitude":
            print(f"Sending unidimensional command: axis_type={type}, num={num}")
            goal_msg.axis_type = UnidimensionalCommand.Goal.AXIS_ALTITUDE
        goal_msg.target_value = int(round(num))
        send_goal_future = self._action_unidimensional_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rifiutato dal server!")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_waypoint_traslate_completed)

    def on_waypoint_traslate_completed(self, future):
        self.send_next_waypoint()

    def on_waypoint_unidimensional_completed(self):
        pass

    def _on_stop_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rifiutato dal server!")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_handle_stop)

    def on_handle_stop(self, future):
        pos = [self.drone_pos[0], self.drone_pos[2], self.drone_pos[1]]
        print(f"posX={pos[0]}, posY={pos[1]}, posZ={pos[2]}")
        #print("PUNTO AGGIUNTO")
        self.panto_planning.process_recalculating(self.index_traslate, pos)
        #self.send_next_waypoint()

    def send_next_waypoint(self):
        if isinstance(self.panto_planning.list_commands[self.index], vertex):
            self.send_traslate(self.panto_planning.list_commands[self.index].x, self.panto_planning.list_commands[self.index].z)
            self.increment_index_traslate()
        elif isinstance(self.panto_planning.list_commands[self.index], angle):
            self.send_unidimensional("rotate", self.panto_planning.list_commands[self.index].angle)
        elif isinstance(self.panto_planning.list_commands[self.index], altitude):
            self.send_unidimensional("altitude", self.panto_planning.list_commands[self.index].altitude)
        self.increment_index()

    def cylinder_found(self, cylinder):
        detected = self.panto_planning.cylinder_found(cylinder, self.index_traslate-2)
        if detected:
            self.send_stop()

    def increment_index(self):
        self.index += 1

    def decrement_index(self):
        self.index -= 1

    def increment_index_traslate(self):
        self.index_traslate += 1

