from rclpy.node import Node
from std_msgs.msg import String
import json


class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher = self.create_publisher(String,"/ttacm",  10)

    def invia(self, time,  target, actual, cmd, mode):
        dati = {
            "time": time,
            "target": target,
            "actual": actual,
            "cmd": cmd,
            "mode": mode
        }
        msg = String()
        msg.data = json.dumps(dati)
        self.publisher.publish(msg)
        #self.get_logger().info(f"PublisherNode Invia: {msg.data}")