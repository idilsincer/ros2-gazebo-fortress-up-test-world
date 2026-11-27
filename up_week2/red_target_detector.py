import time

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


class RedTargetDetector(Node):

    INT_PARAMETERS = (
        'hsv_lower_h', 'hsv_upper_h', 'hsv_lower_h2', 'hsv_upper_h2',
        'min_saturation', 'min_value', 'confirm_frames', 'lost_frames',
    )
    FLOAT_PARAMETERS = (
        'min_area', 'max_area_ratio', 'stream_timeout', 'stats_period',
    )

    def __init__(self):
        super().__init__('red_target_detector')

        descriptor = ParameterDescriptor(description='red target detector')
        self.declare_parameter('hsv_lower_h', 0, descriptor)
        self.declare_parameter('hsv_upper_h', 8, descriptor)
        self.declare_parameter('hsv_lower_h2', 177, descriptor)
        self.declare_parameter('hsv_upper_h2', 180, descriptor)
        self.declare_parameter('min_saturation', 90, descriptor)
        self.declare_parameter('min_value', 50, descriptor)
        self.declare_parameter('min_area', 150.0, descriptor)
        self.declare_parameter('max_area_ratio', 0.5, descriptor)
        self.declare_parameter('confirm_frames', 2, descriptor)
        self.declare_parameter('lost_frames', 3, descriptor)
        self.declare_parameter('debug_image', True, descriptor)
        self.declare_parameter('stream_timeout', 1.0, descriptor)
        self.declare_parameter('stats_period', 1.0, descriptor)

        values = self._current_values()
        ok, reason = self._validate(values)
        if not ok:
            raise ValueError(f'Invalid detector configuration: {reason}')

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.confirmed = False
        self.hit_count = 0
        self.miss_count = 0
        # Grace period: do not report a timeout before the first image had a
        # chance to arrive.  Wall time is used so that a paused simulation
        # clock cannot hide a real camera outage.
        self.last_image_time = time.monotonic()
        self.stream_timed_out = False
        self.frames = 0
        self.frame_counter = 0
        self.processing_times = []
        self.last_report = self.get_clock().now()
        self.last_log_time = 0.0

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray, '/vision/detections', 10)
        self.detected_pub = self.create_publisher(
            Bool, '/vision/target_detected', 10)
        self.offset_pub = self.create_publisher(
            PointStamped, '/vision/target_offset', 10)
        self.debug_pub = self.create_publisher(
            Image, '/vision/debug_image', self.debug_qos)

        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, image_qos)
        self.create_timer(0.25, self.check_stream)

        self.get_logger().info(
            'red_target_detector ready: HSV [%d-%d] U [%d-%d], S>=%d, V>=%d, '
            'min_area=%.1f px^2, confirm=%d, lost=%d'
            % (values['hsv_lower_h'], values['hsv_upper_h'],
               values['hsv_lower_h2'], values['hsv_upper_h2'],
               values['min_saturation'], values['min_value'],
               values['min_area'], values['confirm_frames'],
               values['lost_frames']))

    # ------------------------------------------------------------- parameters
    def _current_values(self):
        values = {}
        for name in self.INT_PARAMETERS:
            values[name] = int(self.get_parameter(name).value)
        for name in self.FLOAT_PARAMETERS:
            values[name] = float(self.get_parameter(name).value)
        values['debug_image'] = bool(self.get_parameter('debug_image').value)
        return values

    @staticmethod
    def _validate(values):
        for name in ('hsv_lower_h', 'hsv_upper_h',
                     'hsv_lower_h2', 'hsv_upper_h2'):
            if not 0 <= values[name] <= 180:
                return False, f'{name} must be in [0, 180]'
        if values['hsv_lower_h'] > values['hsv_upper_h']:
            return False, 'hsv_lower_h must be <= hsv_upper_h'
        if values['hsv_lower_h2'] > values['hsv_upper_h2']:
            return False, 'hsv_lower_h2 must be <= hsv_upper_h2'
        for name in ('min_saturation', 'min_value'):
            if not 0 <= values[name] <= 255:
                return False, f'{name} must be in [0, 255]'
        if values['min_area'] < 0.0:
            return False, 'min_area must be >= 0'
        if not 0.0 < values['max_area_ratio'] <= 1.0:
            return False, 'max_area_ratio must be in (0, 1]'
        if values['confirm_frames'] < 1:
            return False, 'confirm_frames must be >= 1'
        if values['lost_frames'] < 1:
            return False, 'lost_frames must be >= 1'
        if values['stream_timeout'] <= 0.0:
            return False, 'stream_timeout must be > 0'
        if values['stats_period'] <= 0.0:
            return False, 'stats_period must be > 0'
        return True, ''

    def _on_set_parameters(self, params):
        values = self._current_values()
        for parameter in params:
            if parameter.name in values:
                values[parameter.name] = parameter.value

        ok, reason = self._validate(values)
        if not ok:
            self.get_logger().warning(f'Rejected parameter update: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        if any(parameter.name == 'debug_image' for parameter in params):
            self.get_logger().info(
                'debug_image = %s' % values['debug_image'])
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ stream guard
    def check_stream(self):
        age = time.monotonic() - self.last_image_time
        timed_out = age > float(self.get_parameter('stream_timeout').value)

        if timed_out and not self.stream_timed_out:
            self.stream_timed_out = True
            self.confirmed = False
            self.hit_count = 0
            self.miss_count = 0
            self._publish_detected(False)
            self._publish_detections(self.get_clock().now().to_msg(), '', [])
            self.get_logger().warning(
                'Camera stream timeout (> %.1f s); reporting no detection.'
                % float(self.get_parameter('stream_timeout').value))
        elif not timed_out and self.stream_timed_out:
            self.stream_timed_out = False
            self.get_logger().info('Camera stream is back; resuming detection.')

    # ---------------------------------------------------------------- callbacks
    def image_callback(self, msg):
        started = time.perf_counter()
        self.last_image_time = time.monotonic()
        if self.stream_timed_out:
            self.stream_timed_out = False
            self.get_logger().info('Camera stream is back; resuming detection.')

        try:
            frame = self._to_bgr(msg)
        except Exception as exc:  # corrupted/unsupported image must not kill us
            self.get_logger().warning(
                f'Dropping malformed image: {exc}', throttle_duration_sec=2.0)
            return

        if frame is None or frame.size == 0:
            return

        try:
            candidates = self._detect(frame, msg)
        except Exception as exc:  # defensive: never crash on a bad frame
            self.get_logger().error(
                f'Detection failed on this frame: {exc}',
                throttle_duration_sec=2.0)
            return

        # Confirmation / loss hysteresis.
        if candidates:
            self.hit_count += 1
            self.miss_count = 0
            if not self.confirmed and self.hit_count >= int(
                    self.get_parameter('confirm_frames').value):
                self.confirmed = True
                self.get_logger().info('Target confirmed.')
        else:
            self.hit_count = 0
            self.miss_count += 1
            if self.confirmed and self.miss_count >= int(
                    self.get_parameter('lost_frames').value):
                self.confirmed = False
                self.get_logger().info('Target lost.')

        self._publish_detected(self.confirmed)
        self._publish_detections(msg.header.stamp, msg.header.frame_id,
                                 candidates)

        if candidates and self.confirmed:
            largest = candidates[0]
            self._publish_offset(msg.header.stamp, msg.header.frame_id,
                                 largest, msg.width, msg.height)

        if bool(self.get_parameter('debug_image').value):
            self._publish_debug(frame, msg, candidates)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.processing_times.append(elapsed_ms)
        self.frames += 1
        self.frame_counter += 1
        self._report_stats(msg, candidates)

    # -------------------------------------------------------------- conversion
    @staticmethod
    def _to_bgr(msg):
        encoding = (msg.encoding or '').lower()
        if encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif encoding in ('rgba8', 'bgra8'):
            channels = 4
        else:
            raise ValueError(f'unsupported encoding "{msg.encoding}"')

        if msg.width <= 0 or msg.height <= 0:
            raise ValueError('image has zero width or height')

        row_bytes = int(msg.step) if msg.step else msg.width * channels
        if row_bytes < msg.width * channels:
            raise ValueError(
                f'step too small ({row_bytes} < {msg.width * channels})')
        minimum = row_bytes * msg.height
        if len(msg.data) < minimum:
            raise ValueError(
                f'image buffer too small ({len(msg.data)} < {minimum})')

        buffer = np.frombuffer(msg.data, dtype=np.uint8, count=minimum)
        rows = buffer.reshape(msg.height, row_bytes)
        image = rows[:, :msg.width * channels].reshape(
            msg.height, msg.width, channels)

        if channels == 4:
            image = image[:, :, :3]
        if encoding.startswith('rgb'):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Own a writable, contiguous array (np.frombuffer is read-only).
        return np.ascontiguousarray(image).copy()

    # ---------------------------------------------------------------- detection
    def _detect(self, bgr, msg):
        lower_h = int(self.get_parameter('hsv_lower_h').value)
        upper_h = int(self.get_parameter('hsv_upper_h').value)
        lower_h2 = int(self.get_parameter('hsv_lower_h2').value)
        upper_h2 = int(self.get_parameter('hsv_upper_h2').value)
        min_saturation = int(self.get_parameter('min_saturation').value)
        min_value = int(self.get_parameter('min_value').value)
        min_area = float(self.get_parameter('min_area').value)
        max_area_ratio = float(self.get_parameter('max_area_ratio').value)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([lower_h, min_saturation, min_value], dtype=np.uint8),
            np.array([upper_h, 255, 255], dtype=np.uint8))
        mask |= cv2.inRange(
            hsv,
            np.array([lower_h2, min_saturation, min_value], dtype=np.uint8),
            np.array([upper_h2, 255, 255], dtype=np.uint8))

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = max_area_ratio * float(msg.width * msg.height)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            candidates.append({
                'area': area,
                'area_ratio': area / float(msg.width * msg.height),
                'x': int(x),
                'y': int(y),
                'width': int(width),
                'height': int(height),
                'cx': float(x) + float(width) * 0.5,
                'cy': float(y) + float(height) * 0.5,
            })

        candidates.sort(key=lambda item: item['area'], reverse=True)
        return candidates

    # --------------------------------------------------------------- publishing
    def _publish_detected(self, detected):
        message = Bool()
        message.data = bool(detected)
        self.detected_pub.publish(message)

    def _publish_detections(self, stamp, frame_id, candidates):
        message = Detection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = frame_id

        for index, candidate in enumerate(candidates):
            detection = Detection2D()
            detection.header.stamp = stamp
            detection.header.frame_id = frame_id
            detection.id = str(index)
            detection.bbox.center.position.x = candidate['cx']
            detection.bbox.center.position.y = candidate['cy']
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(candidate['width'])
            detection.bbox.size_y = float(candidate['height'])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'red_target'
            hypothesis.hypothesis.score = candidate['area_ratio']
            detection.results.append(hypothesis)
            message.detections.append(detection)

        self.detections_pub.publish(message)

    def _publish_offset(self, stamp, frame_id, candidate, width, height):
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.point.x = (candidate['cx'] - width * 0.5) / (width * 0.5)
        message.point.y = (candidate['cy'] - height * 0.5) / (height * 0.5)
        message.point.z = candidate['area'] / float(width * height)
        self.offset_pub.publish(message)

    def _publish_debug(self, frame, msg, candidates):
        debug = frame.copy()
        for index, candidate in enumerate(candidates):
            color = (0, 255, 0) if index else (255, 0, 255)
            thickness = 2 if index else 3
            x, y = candidate['x'], candidate['y']
            x2 = x + candidate['width']
            y2 = y + candidate['height']
            cv2.rectangle(debug, (x, y), (x2, y2), color, thickness)
            center = (int(round(candidate['cx'])),
                      int(round(candidate['cy'])))
            cv2.drawMarker(debug, center, (0, 255, 255),
                           cv2.MARKER_CROSS, 14, 2)

        if candidates:
            largest = candidates[0]
            text = (f'area={largest["area"]:.0f} '
                    f'center=({largest["cx"]:.0f},{largest["cy"]:.0f})')
        else:
            text = 'no detection'
        cv2.putText(debug, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)

        output = Image()
        output.header = msg.header
        output.height = msg.height
        output.width = msg.width
        output.encoding = 'bgr8'
        output.is_bigendian = 0
        output.step = msg.width * 3
        output.data = debug.tobytes()
        self.debug_pub.publish(output)

    # ------------------------------------------------------------------- stats
    def _report_stats(self, msg, candidates):
        now = self.get_clock().now()
        period = float(self.get_parameter('stats_period').value)
        elapsed = (now - self.last_report).nanoseconds / 1e9
        if elapsed < period:
            return

        average_ms = 0.0
        if self.processing_times:
            average_ms = sum(self.processing_times) / len(self.processing_times)
        self.processing_times.clear()

        if candidates:
            largest = candidates[0]
            geometry = (f'center=({largest["cx"]:.1f},{largest["cy"]:.1f}) '
                        f'area={largest["area"]:.1f}')
        else:
            geometry = 'center=none area=0'

        self.get_logger().info(
            f'frames={self.frame_counter} detected={self.confirmed} '
            f'{geometry} avg_proc={average_ms:.2f} ms')
        self.frame_counter = 0
        self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = RedTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
