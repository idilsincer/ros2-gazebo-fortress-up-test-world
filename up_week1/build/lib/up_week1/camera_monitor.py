import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        self.count = 0
        self.last = None
        self.create_subscription(Image, '/camera/image_raw', self.callback, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def callback(self, msg):
        self.count += 1
        self.last = msg

    def report(self):
        if self.last is None:
            self.get_logger().info('No image yet (total: 0)')
            return
        m = self.last
        self.get_logger().info(
            f'{m.width}x{m.height} encoding={m.encoding} total={self.count}')


def main():
    rclpy.init()
    rclpy.spin(CameraMonitor())
    rclpy.shutdown()