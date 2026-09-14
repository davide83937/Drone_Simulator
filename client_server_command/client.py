import os

# Registra i percorsi binari contenenti le DLL di ROS 2 e delle interfacce custom
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

# Assicura la variabile AMENT_PREFIX_PATH per la localizzazione delle risorse
prefix_path = (
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces;"
    r"C:\Users\david\OneDrive\Desktop\ros2-windows"
)
os.environ.setdefault("AMENT_PREFIX_PATH", prefix_path)

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import MoveAxis
from geometry_msgs.msg import Vector3

class MoveAxisClient(Node):
    def __init__(self):
        super().__init__('move_axis_client')
        self._action_client = ActionClient(self, MoveAxis, 'move_robot')

    def send_goal(self, target_x, target_y, target_z,  target_theta):
        self._action_client.wait_for_server()
        self._action_client.wait_for_server()

        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=target_x, y=target_y, z=target_z)
        goal_msg.target_rotation = target_theta

        self.get_logger().info(f'Invio goal: Target({target_x}, {target_y}, {target_z}) a rotazione {target_theta}')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Target non valido')
            return

        self.get_logger().info("Il server ha accettato il goal, esecuzione in corso...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Avanzamento: {fb.progress_percentage:.1f}% -> '
            f'Pos: ({fb.current_position.x:.2f}, {fb.current_position.y:.2f}, {fb.current_position.z:.2f}'
            f'Rot: ({fb.current_rotation})'
        )

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Stato finale: {status}')
        self.get_logger().info(
            f'Risultato: Success={result.success}, Msg="{result.message}", '
            f'Posizione Finale: ({result.final_position.x:.2f}, '
            f'{result.final_position.y:.2f}, {result.final_position.z:.2f})'
            f'{result.final_rotation:.2f}'
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = MoveAxisClient()
    x = float(input("Inserisci x: "))
    y = float(input("Inserisci y: "))
    z = float(input("Inserisci z: "))
    rotation = float(input("Inserisci rot: "))
    client.send_goal(x, y, z, rotation)
    rclpy.spin(client)

if __name__ == '__main__':
    while True:
        main()


