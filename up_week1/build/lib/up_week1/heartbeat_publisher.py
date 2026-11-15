import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        self.pub = self.create_publisher(String, '/up/heartbeat', 10)
        self.seq = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = f'Idil Sincer - heartbeat #{self.seq}'
        self.pub.publish(msg)
        self.seq += 1


def main():
    rclpy.init()
    rclpy.spin(HeartbeatPublisher())
    rclpy.shutdown()