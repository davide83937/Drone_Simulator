import rclpy
from plotting.plotReceiver import ReceiverSubscriber

if __name__ == '__main__':
    rclpy.init(args=None)
    node = ReceiverSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()