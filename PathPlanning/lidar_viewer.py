import os

import numpy as np




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
from rclpy.node import Node
from geometry_msgs.msg import Vector3


# --- NODO ROS 2 PER POSIZIONE, ROTAZIONE E ALLARMI ---
class DroneStateSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_viewer_state_sub')
        self.create_subscription(Vector3, 'drone_position', self.pos_callback, 10)
        self.create_subscription(Vector3, 'drone_rotation', self.rot_callback, 10)
        #self.create_subscription(String, 'blocking_obstacle', self.blocking_obs_callback, 10)

        #self.static_obs_pub = self.create_publisher(String, 'new_static_obstacle', 10)
        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_yaw = 0.0
        self.blocking_obstacles_queue = []

    def pos_callback(self, msg):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])

    def rot_callback(self, msg):
        self.drone_yaw = np.radians(msg.z)




# --- CONFIGURAZIONE LIDAR UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 9999


def main():
    rclpy.init()
    state_node = DroneStateSubscriber()



if __name__ == "__main__":
    main()