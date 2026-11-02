"""Per-class stereo observations -> selected object coordinates in map."""
from dataclasses import dataclass, field
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from hornet_interfaces.msg import ObjectPosition, SourceSelection, StereoObservation

from hornet_front_camera.models import CameraModel
from .geometry import StereoGeometry, select_source, transform_estimate


@dataclass
class Track:
    candidates: dict = field(default_factory=dict)
    desired: str = 'object_detection'
    last_published_ns: int = -1
    touched_ns: int = 0
    last_feedback: tuple = ()


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation_node')
        defaults = {
            'left_yaml': '', 'right_yaml': '', 'map_frame': 'map',
            'object_types': ['gate', 'yellow_flare', 'orange_flare', 'blue_flare', 'red_flare', 'blue_drum', 'red_drum'],
            'allow_example_calibration': False, 'enter_color_m': 2.0, 'exit_color_m': 2.5,
            'max_position_stddev_m': 0.5, 'min_confidence': 0.25,
            'max_observation_age_sec': 0.5, 'max_source_skew_sec': 0.15,
            'future_tolerance_sec': 0.05, 'track_ttl_sec': 2.0, 'max_tracks': 256,
            'min_range_m': 0.2, 'max_range_m': 15.0,
            'min_disparity_px': 0.5, 'max_reprojection_px': 2.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        p = self.config
        finite_keys = [k for k, v in p.items() if isinstance(v, float)]
        if not np.isfinite([p[k] for k in finite_keys]).all():
            raise ValueError('All numeric parameters must be finite')
        if not 0 < p['enter_color_m'] < p['exit_color_m'] or p['max_position_stddev_m'] <= 0:
            raise ValueError('Invalid source selection thresholds')
        if not 0 <= p['min_confidence'] <= 1 or p['max_tracks'] <= 0:
            raise ValueError('Invalid confidence or track count')
        if min(p['max_observation_age_sec'], p['max_source_skew_sec'], p['track_ttl_sec']) <= 0 or p['future_tolerance_sec'] < 0:
            raise ValueError('Invalid age/expiry parameters')
        self.types = p['object_types']
        if not self.types or len(set(self.types)) != len(self.types) or any(not re.fullmatch('[a-z][a-z0-9_]*', t) for t in self.types):
            raise ValueError('object_types must be unique ROS-safe class names')
        if not p['map_frame'] or p['map_frame'].startswith('/'):
            raise ValueError('map_frame must be a nonempty TF frame without a leading slash')
        self.left = CameraModel.load(p['left_yaml'], p['allow_example_calibration'])
        self.right = CameraModel.load(p['right_yaml'], p['allow_example_calibration'])
        self.geometry = StereoGeometry(self.left, self.right, p['min_range_m'], p['max_range_m'], p['min_disparity_px'], p['max_reprojection_px'])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracks = {}
        self.subscriptions_ = []
        self.positions, self.selections = {}, {}
        # Sparse observations: reliable KEEP_LAST(10), unlike high-rate images.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for kind in self.types:
            self.positions[kind] = self.create_publisher(ObjectPosition, f'/objects/{kind}/position', qos)
            self.selections[kind] = self.create_publisher(SourceSelection, f'/perception/selection/{kind}', qos)
            for source, branch in [('object_detection', 'detections'), ('color_threshold', 'color')]:
                self.subscriptions_.append(self.create_subscription(
                    StereoObservation, f'/front/{branch}/{kind}',
                    lambda msg, k=kind, s=source: self.observe(msg, k, s), qos))
        # Retry exact-time TF lookups after TF callbacks have had a chance to run.
        self.timer = self.create_timer(0.05, self.flush)

    def observe(self, msg, kind, source):
        now = self.get_clock().now().nanoseconds
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        p = self.config
        if msg.object_type != kind or msg.source != source or not msg.instance_id or len(msg.instance_id) > 128:
            self.get_logger().warning('Rejected observation with inconsistent class/source/instance_id', throttle_duration_sec=3.0)
            return
        if msg.header.frame_id != self.left.frame_id or msg.calibration_id != self.left.calibration_id:
            self.get_logger().warning('Rejected observation with wrong rectified frame or calibration_id', throttle_duration_sec=3.0)
            return
        if stamp <= 0 or now - stamp > p['max_observation_age_sec'] * 1e9 or stamp - now > p['future_tolerance_sec'] * 1e9:
            self.get_logger().warning('Rejected stale, zero, or future observation timestamp', throttle_duration_sec=3.0)
            return
        if not np.isfinite(msg.confidence) or not p['min_confidence'] <= msg.confidence <= 1:
            return
        key = (kind, msg.instance_id)
        if key not in self.tracks:
            if len(self.tracks) >= p['max_tracks']:
                self.get_logger().warning('Track capacity reached; waiting for expiry', throttle_duration_sec=3.0)
                return
            self.tracks[key] = Track(touched_ns=now)
        track = self.tracks[key]
        old = track.candidates.get(source)
        if old is not None and stamp <= old[0]:
            return
        try:
            estimate = self.geometry.estimate((msg.u_left, msg.v_left), (msg.u_right, msg.v_right), msg.pixel_stddev_px)
        except ValueError as error:
            track.candidates.pop(source, None)
            self.get_logger().warning(f'Rejected {kind}/{msg.instance_id}: {error}', throttle_duration_sec=3.0)
        else:
            track.candidates[source] = (stamp, estimate, msg)
        track.touched_ns = now
        self.publish_track(key, track, now)

    def publish_track(self, key, track, now):
        kind, instance = key
        p = self.config
        candidates = {s: item for s, item in track.candidates.items()
                      if -p['future_tolerance_sec'] * 1e9 <= now-item[0] <= p['max_observation_age_sec'] * 1e9}
        if candidates:
            latest = max(item[0] for item in candidates.values())
            candidates = {s: item for s, item in candidates.items() if latest-item[0] <= p['max_source_skew_sec'] * 1e9}
        estimates = {s: item[1] for s, item in candidates.items()}
        chosen, track.desired, reason = select_source(estimates, track.desired, p['enter_color_m'], p['exit_color_m'], p['max_position_stddev_m'])
        feedback = SourceSelection()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.header.frame_id = self.left.frame_id
        feedback.object_type, feedback.instance_id = kind, instance
        feedback.selected_source, feedback.preferred_source = chosen, track.desired
        feedback.valid = bool(chosen)
        feedback.reason = reason
        feedback.camera_range_m = float('nan')
        feedback.position_stddev_m = float('nan')
        if chosen:
            stamp, estimate, observation = candidates[chosen]
            feedback.measurement_stamp = observation.header.stamp
            feedback.camera_range_m = estimate.range_m
            feedback.position_stddev_m = estimate.stddev_m
            try:
                transform = self.buffer.lookup_transform(p['map_frame'], self.left.frame_id, Time.from_msg(observation.header.stamp))
                t, q = transform.transform.translation, transform.transform.rotation
                xyz, covariance = transform_estimate(estimate, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                feedback.map_transform_available = True
                if stamp > track.last_published_ns:
                    output = ObjectPosition()
                    output.header.stamp = observation.header.stamp
                    output.header.frame_id = p['map_frame']
                    output.object_type, output.instance_id, output.source = kind, instance, chosen
                    output.calibration_id = self.left.calibration_id
                    output.position.x, output.position.y, output.position.z = map(float, xyz)
                    output.position_covariance = covariance.ravel().tolist()
                    output.includes_localization_uncertainty = False
                    output.camera_range_m = estimate.range_m
                    output.reprojection_rms_px = estimate.reprojection_rms_px
                    self.positions[kind].publish(output)
                    track.last_published_ns = stamp
            except (TransformException, ValueError) as error:
                feedback.reason = f'{reason}; map position withheld: {error}'
        # Changes and new measurements get feedback; unchanged timer ticks stay quiet.
        signature = (chosen, track.desired, feedback.valid, feedback.map_transform_available,
                     feedback.reason, feedback.measurement_stamp.sec, feedback.measurement_stamp.nanosec)
        if signature != track.last_feedback:
            self.selections[kind].publish(feedback)
            track.last_feedback = signature

    def flush(self):
        now = self.get_clock().now().nanoseconds
        for key, track in list(self.tracks.items()):
            self.publish_track(key, track, now)
            if now-track.touched_ns > self.config['track_ttl_sec'] * 1e9 or now < track.touched_ns:
                del self.tracks[key]


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
