import os

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool  # Aggiunto Bool per il comando di aggancio
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from PathPlanning.PantoPlanning import PantoPlanning
from custom_interfaces.action import MoveAxis
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
        self.panto_planning = panto_planning

    def send_traslate(self):
        pass

    def send_stop(self):
        pass

    def send_unidimensional(self):
        pass

    def on_waypoint_completed(self):
        pass

    def on_handle_stop(self):
        pass