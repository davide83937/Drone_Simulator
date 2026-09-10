import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from plotting.plotManager import plotManager


class ReceiverSubscriber(Node):
    def __init__(self):
        super().__init__('_plot_receiver')
        self.log_sub = self.create_subscription(String, '/ttacm', self.ricevi, 10)
        self.last_msg = None
        self.plotManager = plotManager(time_end=25)


    def ricevi(self, msg):
        data = json.loads(msg.data)

        #return data
        time = data['time']
        target = data['target']
        actual = data['actual']
        cmd = data['cmd']
        mode = data['mode']
        self.plotManager.fillListsPlot(time, target, actual, cmd, mode)

        #return time, target, actual, cmd, mode


