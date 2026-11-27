import csv
import math
import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from up_week2.scene_geometry import project_point, target_visibility

TP_THRESHOLD = 0.90
FP_THRESHOLD = 0.05
CENTER_ERROR_THRESHOLD = 10.0


class EvaluationNode(Node):

    def __init__(self):
        super().__init__('evaluation_node')

        self.declare_parameter('duration', 45.0)
        self.declare_parameter('min_frames', 100)
        self.declare_parameter('path_period', 38.0)
        self.declare_parameter('lighting_label', 'normal')
        self.declare_parameter('output_csv', 'evaluation_results.csv')

        self.declare_parameter('camera_position', [0.9, -4.5, 1.5])
        self.declare_parameter('camera_rpy', [0.0, 0.265, 1.305])
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_hfov', 1.047)
        self.declare_parameter('target_size', [0.4, 0.4, 1.0])
        self.declare_parameter('obstacle_center', [1.858, -0.298, 0.5])
        self.declare_parameter('obstacle_size', [1.0, 1.0, 1.0])
        self.declare_parameter('match_radius_px', 80.0)
        self.declare_parameter('gt_timeout', 0.5)

        self.duration = float(self.get_parameter('duration').value)
        self.min_frames = int(self.get_parameter('min_frames').value)
        self.path_period = float(self.get_parameter('path_period').value)
        self.lighting_label = str(self.get_parameter('lighting_label').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.camera_position = [
            float(v) for v in self.get_parameter('camera_position').value]
        self.camera_rpy = [
            float(v) for v in self.get_parameter('camera_rpy').value]
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_hfov = float(self.get_parameter('camera_hfov').value)
        self.target_size = [
            float(v) for v in self.get_parameter('target_size').value]
        self.obstacle_center = [
            float(v) for v in self.get_parameter('obstacle_center').value]
        self.obstacle_size = [
            float(v) for v in self.get_parameter('obstacle_size').value]
        self.match_radius_px = float(
            self.get_parameter('match_radius_px').value)
        self.gt_timeout = float(self.get_parameter('gt_timeout').value)

        self.gt_history = deque(maxlen=300)
        self.first_stamp = None
        self.last_stamp = None
        self.finished = False
        self.rows = []

        self.counts = {
            'full': 0,
            'partial': 0,
            'hidden': 0,
            'out_of_view': 0,
            'unknown': 0,
        }
        self.detected = {key: 0 for key in self.counts}
        self.center_errors = []
        self.full_multi_candidate = 0

        self.create_subscription(
            Detection2DArray, '/vision/detections', self.detection_callback, 10)
        self.create_subscription(
            PoseStamped, '/target/ground_truth', self.gt_callback, 10)

        self.get_logger().info(
            f'Evaluation started for "{self.lighting_label}" lighting: '
            f'duration={self.duration:.1f} s, camera='
            f'{self.camera_width}x{self.camera_height}')

    def gt_callback(self, msg):
        stamp = msg.header.stamp
        self.gt_history.append(
            (stamp.sec + stamp.nanosec * 1e-9, msg))

    def _nearest_pose(self, stamp_seconds):
        if not self.gt_history:
            return None
        difference, pose = min(
            ((abs(item[0] - stamp_seconds), item[1])
             for item in self.gt_history),
            key=lambda item: item[0])
        if difference > self.gt_timeout:
            return None
        return pose

    def _visibility_of(self, pose):
        return target_visibility(
            (pose.pose.position.x,
             pose.pose.position.y,
             pose.pose.position.z),
            self.camera_position,
            self.camera_rpy,
            self.camera_width,
            self.camera_height,
            self.camera_hfov,
            self.obstacle_center,
            self.obstacle_size,
            target_radius=0.5 * self.target_size[0],
            target_height=self.target_size[2],
        )

    def _project_target_center(self, pose):
        half = [value * 0.5 for value in self.target_size]
        us, vs = [], []
        for dx in (-half[0], half[0]):
            for dy in (-half[1], half[1]):
                for dz in (-half[2], half[2]):
                    point = (pose.pose.position.x + dx,
                             pose.pose.position.y + dy,
                             pose.pose.position.z + dz)
                    projected = project_point(
                        point, self.camera_position, self.camera_rpy,
                        self.camera_width, self.camera_height,
                        self.camera_hfov)
                    if projected is None:
                        return None
                    us.append(projected[0])
                    vs.append(projected[1])
        return (0.5 * (min(us) + max(us)), 0.5 * (min(vs) + max(vs)))

    def detection_callback(self, msg):
        if self.finished:
            return

        stamp = msg.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = stamp_seconds
        self.last_stamp = stamp_seconds

        candidates = list(msg.detections)

        gt_uv = None
        visibility = 'unknown'
        gt_pose = self._nearest_pose(stamp_seconds)
        if gt_pose is not None:
            gt_uv = self._project_target_center(gt_pose)
            visibility = self._visibility_of(gt_pose)
            if visibility not in self.counts:
                visibility = 'unknown'
        matched = None
        if gt_uv is not None:
            for detection in candidates:
                distance = math.hypot(
                    detection.bbox.center.position.x - gt_uv[0],
                    detection.bbox.center.position.y - gt_uv[1])
                if distance <= self.match_radius_px:
                    if matched is None or distance < matched[0]:
                        matched = (distance, detection)

        detected = matched is not None
        largest = max(
            candidates,
            key=lambda det: det.bbox.size_x * det.bbox.size_y) if candidates else None

        center_error = None if matched is None else matched[0]

        if visibility in ('full', 'partial'):
            hit = detected
        else:
            hit = bool(candidates)

        self.counts[visibility] += 1
        if hit:
            self.detected[visibility] += 1
        if visibility == 'full' and len(candidates) > 1:
            self.full_multi_candidate += 1
        if visibility == 'full' and center_error is not None:
            self.center_errors.append(center_error)

        self.rows.append({
            'stamp': f'{stamp_seconds:.3f}',
            'stamp_rel': f'{stamp_seconds - self.first_stamp:.3f}',
            'visibility': visibility,
            'hit': int(hit),
            'gt_u': '' if gt_uv is None else f'{gt_uv[0]:.2f}',
            'gt_v': '' if gt_uv is None else f'{gt_uv[1]:.2f}',
            'detections': len(candidates),
            'det_u': ('' if largest is None else
                      f'{largest.bbox.center.position.x:.2f}'),
            'det_v': ('' if largest is None else
                      f'{largest.bbox.center.position.y:.2f}'),
            'det_area': ('' if largest is None else
                         f'{largest.bbox.size_x * largest.bbox.size_y:.1f}'),
            'center_error': '' if center_error is None else f'{center_error:.2f}',
        })

        elapsed = stamp_seconds - self.first_stamp
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True

        full = self.counts['full']
        invisible = self.counts['hidden'] + self.counts['out_of_view']
        partial = self.counts['partial']

        detection_rate = (self.detected['full'] / full) if full else 0.0
        false_positive_rate = (
            (self.detected['hidden'] + self.detected['out_of_view']) / invisible
            if invisible else 0.0)
        average_error = (
            sum(self.center_errors) / len(self.center_errors)
            if self.center_errors else float('nan'))

        covered = (self.last_stamp - self.first_stamp
                   if self.first_stamp is not None else 0.0)

        lines = []
        lines.append('=' * 68)
        lines.append(f'Detection evaluation - lighting: {self.lighting_label}')
        lines.append('=' * 68)
        lines.append(f'Duration              : {covered:.1f} s '
                     f'({len(self.rows)} processed frames)')
        lines.append(f'Path coverage         : '
                     f'{"yes" if covered >= self.path_period else "NO"} '
                     f'(need >= {self.path_period:.1f} s)')
        lines.append(f'Full-visible frames   : {full}')
        lines.append(f'Partial frames        : {partial}')
        lines.append(f'Invisible frames      : {invisible} '
                     f'(hidden={self.counts["hidden"]}, '
                     f'out_of_view={self.counts["out_of_view"]})')
        lines.append(f'Unknown frames        : {self.counts["unknown"]}')
        lines.append('-')
        lines.append(f'Detection rate (full) : {100.0 * detection_rate:.2f} % '
                     f'[>= {100.0 * TP_THRESHOLD:.0f} %]')
        lines.append(f'False positive rate   : '
                     f'{100.0 * false_positive_rate:.2f} % '
                     f'[<= {100.0 * FP_THRESHOLD:.0f} %]')
        lines.append(f'Mean centre error     : {average_error:.2f} px '
                     f'[<= {CENTER_ERROR_THRESHOLD:.0f} px]')
        lines.append(f'Full frames with >1 candidate: {self.full_multi_candidate}')
        lines.append(f'Partial detection rate: '
                     f'{(100.0 * self.detected["partial"] / partial) if partial else 0.0:.2f} %')
        lines.append('-')
        passed = (full > 0 and invisible > 0
                  and detection_rate >= TP_THRESHOLD
                  and false_positive_rate <= FP_THRESHOLD
                  and average_error <= CENTER_ERROR_THRESHOLD
                  and len(self.rows) >= self.min_frames
                  and covered >= self.path_period)
        lines.append('RESULT: ' + ('PASS' if passed else 'FAIL / needs tuning'))
        if len(self.rows) < self.min_frames:
            lines.append(f'NOTE: fewer than {self.min_frames} frames recorded.')
        lines.append('=' * 68)
        report = '\n'.join(lines)

        self.get_logger().info('\n' + report)
        print(report, flush=True)

        self._write_csv()
        # Stop from a timer, not from inside the subscription callback.
        self.create_timer(0.05, self._request_shutdown)

    def _request_shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def _write_csv(self):
        directory = os.path.dirname(os.path.abspath(self.output_csv))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                'stamp', 'stamp_rel', 'visibility', 'hit', 'gt_u', 'gt_v',
                'detections', 'det_u', 'det_v', 'det_area', 'center_error'])
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f'Wrote {len(self.rows)} rows to '
                               f'{self.output_csv}')


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
