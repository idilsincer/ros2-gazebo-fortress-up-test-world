import math
import os
import shutil
import subprocess
import time

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from up_week2.scene_geometry import target_visibility

try:
    from ros_ign_interfaces.msg import Entity
    from ros_ign_interfaces.srv import SetEntityPose
except ImportError:  # newer ros_gz naming
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose


class TargetMover(Node):

    def __init__(self):
        super().__init__('target_mover')

        self.declare_parameter('use_mover', True)
        self.declare_parameter('path_file', '')
        self.declare_parameter('entity_name', 'red_capsule')
        self.declare_parameter('set_pose_service', '/world/up_test_world/set_pose')
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('enable_cli_fallback', True)
        self.declare_parameter('service_wait_timeout', 5.0)
        self.declare_parameter('cli_rate_hz', 5.0)
        self.declare_parameter('use_gazebo_pose', True)
        self.declare_parameter('pose_info_topic',
                               '/world/up_test_world/pose/info')
        self.declare_parameter('pose_timeout', 0.75)
        self.declare_parameter('camera_position', [0.9, -4.5, 1.5])
        self.declare_parameter('camera_rpy', [0.0, 0.265, 1.305])
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_hfov', 1.047)
        self.declare_parameter('obstacle_center', [1.858, -0.298, 0.5])
        self.declare_parameter('obstacle_size', [1.0, 1.0, 1.0])

        self.use_mover = bool(self.get_parameter('use_mover').value)
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.service_name = str(self.get_parameter('set_pose_service').value)
        self.enable_cli_fallback = bool(
            self.get_parameter('enable_cli_fallback').value)
        self.service_wait_timeout = float(
            self.get_parameter('service_wait_timeout').value)
        self.cli_rate_hz = max(0.5, float(self.get_parameter('cli_rate_hz').value))
        self.camera_position = [
            float(v) for v in self.get_parameter('camera_position').value]
        self.camera_rpy = [
            float(v) for v in self.get_parameter('camera_rpy').value]
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_hfov = float(self.get_parameter('camera_hfov').value)
        self.obstacle_center = [
            float(v) for v in self.get_parameter('obstacle_center').value]
        self.obstacle_size = [
            float(v) for v in self.get_parameter('obstacle_size').value]

        path_file = str(self.get_parameter('path_file').value)
        if not path_file:
            path_file = os.path.join(
                get_package_share_directory('up_week2'),
                'config', 'target_path.yaml')
        self.waypoints, self.loop, self.frame_id = self._load_path(path_file)

        self.pose_pub = self.create_publisher(PoseStamped, '/target/ground_truth', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.visibility_pub = self.create_publisher(String, '/target/visibility', qos)

        self.use_gazebo_pose = bool(
            self.get_parameter('use_gazebo_pose').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.actual_pose = None          # (x, y, z, stamp_msg)
        self.actual_pose_wall = None
        self.actual_pose_warned = False
        if self.use_gazebo_pose:
            pose_topic = str(self.get_parameter('pose_info_topic').value)
            self.create_subscription(TFMessage, pose_topic,
                                     self.pose_info_callback, 10)
            self.get_logger().info(
                f'Ground truth: actual Gazebo pose from {pose_topic} '
                f'(fallback: commanded pose)')

        self.cli_tool = None
        if self.enable_cli_fallback and self.use_mover:
            self.cli_tool = shutil.which('ign') or shutil.which('gz')

        self.client = None
        if self.use_mover:
            self.client = self.create_client(SetEntityPose, self.service_name)

        self.period = max(0.0, self.waypoints[-1][0])
        self.started = False
        self.start_time = None
        self.backend = 'service' if self.client is not None else 'cli'
        self.last_cli_call = 0.0
        self.wait_started = time.monotonic()
        self._service_warned = False

        rate = max(1.0, float(self.get_parameter('update_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {path_file} '
            f'(loop={self.loop}, period={self.period:.1f} s)')

    # ------------------------------------------------------- actual gz pose
    def pose_info_callback(self, msg):
        for transform in msg.transforms:
            name = transform.child_frame_id or transform.header.frame_id
            if self.entity_name in name:
                self.actual_pose = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.header.stamp,
                )
                self.actual_pose_wall = time.monotonic()
                return

    def _actual_pose_is_fresh(self):
        return (self.actual_pose is not None
                and self.actual_pose_wall is not None
                and time.monotonic() - self.actual_pose_wall
                <= self.pose_timeout)

    # ------------------------------------------------------------------ path
    def _load_path(self, path_file):
        with open(path_file, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)

        if not isinstance(data, dict) or 'waypoints' not in data:
            raise ValueError(f'{path_file}: expected a "waypoints" list')

        waypoints = []
        for index, item in enumerate(data['waypoints']):
            try:
                waypoints.append((
                    float(item['t']),
                    float(item['x']),
                    float(item['y']),
                    float(item.get('z', 0.5)),
                ))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'{path_file}: invalid waypoint #{index}: {exc}') from exc

        if len(waypoints) < 2:
            raise ValueError(f'{path_file}: at least two waypoints are required')

        waypoints.sort(key=lambda item: item[0])
        if waypoints[0][0] != 0.0:
            raise ValueError(f'{path_file}: first waypoint must have t=0')
        for previous, current in zip(waypoints, waypoints[1:]):
            if current[0] <= previous[0]:
                raise ValueError(
                    f'{path_file}: waypoint times must be strictly increasing')

        loop = bool(data.get('loop', True))
        frame_id = str(data.get('frame_id', 'world'))
        return waypoints, loop, frame_id

    def _pose_at(self, elapsed):
        if self.loop and self.period > 0.0:
            elapsed = math.fmod(elapsed, self.period)
        else:
            elapsed = min(elapsed, self.period)

        previous = self.waypoints[0]
        for current in self.waypoints[1:]:
            if elapsed <= current[0]:
                span = current[0] - previous[0]
                ratio = 0.0 if span <= 0.0 else (elapsed - previous[0]) / span
                return (
                    previous[1] + ratio * (current[1] - previous[1]),
                    previous[2] + ratio * (current[2] - previous[2]),
                    previous[3] + ratio * (current[3] - previous[3]),
                )
            previous = current
        return previous[1], previous[2], previous[3]

    # --------------------------------------------------------------- driving
    def _send_pose_service(self, x, y, z):
        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.w = 1.0
        self.client.call_async(request)

    def _send_pose_cli(self, x, y, z):
        if self.cli_tool is None:
            return
        now = time.monotonic()
        if now - self.last_cli_call < 1.0 / self.cli_rate_hz:
            return
        self.last_cli_call = now

        request = (
            f'name: "{self.entity_name}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            'orientation: {w: 1.0}'
        )
        command = [
            self.cli_tool, 'service',
            '-s', self.service_name,
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '200',
            '--req', request,
        ]
        try:
            subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            self.get_logger().error(f'CLI fallback failed: {exc}')

    def _publish_ground_truth(self, x, y, z):
        if self.use_gazebo_pose and self._actual_pose_is_fresh():
            x, y, z, _ = self.actual_pose
        elif (self.use_gazebo_pose and self.started
                and not self.actual_pose_warned):
            self.actual_pose_warned = True
            self.get_logger().warning(
                'No Gazebo pose received yet; publishing commanded pose '
                'as ground truth.')
        stamp = self.get_clock().now().to_msg()

        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = float(z)
        message.pose.orientation.w = 1.0
        self.pose_pub.publish(message)

        visibility = target_visibility(
            (x, y, z),
            self.camera_position,
            self.camera_rpy,
            self.camera_width,
            self.camera_height,
            self.camera_hfov,
            self.obstacle_center,
            self.obstacle_size,
        )
        label = String()
        label.data = visibility
        self.visibility_pub.publish(label)

    # ------------------------------------------------------------------ timer
    def on_timer(self):
        if not self.use_mover:
            x, y, z = self.waypoints[0][1], self.waypoints[0][2], self.waypoints[0][3]
            self._publish_ground_truth(x, y, z)
            return

        if not self.started:
            if self.client is not None and self.client.service_is_ready():
                self.backend = 'service'
                self.started = True
            elif (self.enable_cli_fallback and self.cli_tool is not None
                  and time.monotonic() - self.wait_started
                  > self.service_wait_timeout):
                self.backend = 'cli'
                self.started = True
                self.get_logger().warning(
                    f'{self.service_name} not available after '
                    f'{self.service_wait_timeout:.1f} s; using '
                    f'"{self.cli_tool} service" fallback at '
                    f'{self.cli_rate_hz:.1f} Hz.')
            else:
                x, y, z = (self.waypoints[0][1], self.waypoints[0][2],
                           self.waypoints[0][3])
                self._publish_ground_truth(x, y, z)
                if not self._service_warned:
                    self._service_warned = True
                    self.get_logger().info(
                        f'Waiting for Gazebo service {self.service_name} ...')
                return

            self.start_time = self.get_clock().now()
            self.get_logger().info(f'Target movement started (backend='
                                   f'{self.backend}).')

        if (self.backend == 'cli' and self.client is not None
                and self.client.service_is_ready()):
            self.backend = 'service'
            self.get_logger().info('ROS set_pose service is ready now; '
                                   'switching from CLI fallback.')

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x, y, z = self._pose_at(elapsed)

        if self.backend == 'service':
            self._send_pose_service(x, y, z)
        else:
            self._send_pose_cli(x, y, z)

        self._publish_ground_truth(x, y, z)


def main(args=None):
    rclpy.init(args=args)
    node = TargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
