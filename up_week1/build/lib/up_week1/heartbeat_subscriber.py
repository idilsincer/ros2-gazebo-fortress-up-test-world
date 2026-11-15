import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatSubscriber(Node):
    def __init__(self):
        super().__init__('heartbeat_subscriber')
        self.create_subscription(String, '/up/heartbeat', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main():
    rclpy.init()
    rclpy.spin(HeartbeatSubscriber())
    rclpy.shutdown()