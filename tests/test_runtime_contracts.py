"""Exercise the actual adapters with minimal ROS messages and a controllable TF.

These tests check orchestration, not ROS middleware compatibility. A Jazzy build
and live topic/TF checks remain necessary.
"""
import copy
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'calibration_node'), str(ROOT/'depth_estimation_node')]


def header():
    return NS(stamp=NS(sec=10, nanosec=0), frame_id='front_left_rect_optical_frame')


class Message:
    def __init__(self):
        self.header = header()
        self.measurement_stamp = NS(sec=0, nanosec=0)
        self.map_transform_available = False
        self.position = NS(x=0., y=0., z=0.)


class FakeTime:
    def __init__(self, ns=10_000_000_000):
        self.nanoseconds = ns

    @classmethod
    def from_msg(cls, msg):
        return cls(msg.sec*1_000_000_000+msg.nanosec)

    def to_msg(self):
        return NS(sec=self.nanoseconds//1_000_000_000, nanosec=self.nanoseconds%1_000_000_000)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.params = {}
        self.clock = FakeTime()
        self.log = NS(info=lambda *a, **k: None, warning=lambda *a, **k: None, error=lambda *a, **k: None)

    def declare_parameter(self, key, value):
        self.params[key] = self.overrides.get(key, value)

    def get_parameter(self, key):
        return NS(value=self.params[key])

    def get_clock(self):
        return NS(now=lambda: self.clock)

    def get_logger(self):
        return self.log

    def create_publisher(self, kind, topic, qos):
        messages = []
        return NS(messages=messages, topic=topic, publish=lambda msg: messages.append(copy.deepcopy(msg)))

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args


class MissingTransform(Exception):
    pass


class FakeBuffer:
    def __init__(self):
        self.available = False
        self.requests = []

    def lookup_transform(self, target, source, stamp):
        self.requests.append((target, source, stamp.nanoseconds))
        if not self.available:
            raise MissingTransform('No exact-time map transform')
        return NS(transform=NS(translation=NS(x=10., y=20., z=30.), rotation=NS(x=0., y=0., z=0., w=1.)))


def module(name, **attributes):
    result = ModuleType(name)
    result.__dict__.update(attributes)
    return result


def load_adapter(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        qos = lambda **kw: NS(**kw)
        stubs = {
            'rclpy': module('rclpy'), 'rclpy.node': module('rclpy.node', Node=FakeNode),
            'rclpy.qos': module('rclpy.qos', QoSProfile=qos, ReliabilityPolicy=NS(RELIABLE=1), qos_profile_sensor_data=object()),
            'rclpy.time': module('rclpy.time', Time=FakeTime),
            'tf2_ros': module('tf2_ros', Buffer=FakeBuffer, TransformException=MissingTransform, TransformListener=lambda *a: None),
            'hornet_interfaces': module('hornet_interfaces'),
            'hornet_interfaces.msg': module('hornet_interfaces.msg', ObjectPosition=Message, SourceSelection=Message, StereoObservation=Message),
            'sensor_msgs': module('sensor_msgs'), 'sensor_msgs.msg': module('sensor_msgs.msg', Image=Message),
            'cv_bridge': module('cv_bridge', CvBridge=lambda: None, CvBridgeError=ValueError),
        }
        cls.patcher = patch.dict(sys.modules, stubs)
        cls.patcher.start()
        cls.depth = load_adapter('hornet_depth.runtime_test_node', ROOT/'depth_estimation_node/hornet_depth/node.py')
        cls.calibration = load_adapter('hornet_calibration.runtime_test_node', ROOT/'calibration_node/hornet_calibration/node.py')

    @classmethod
    def tearDownClass(cls):
        cls.patcher.stop()

    def setUp(self):
        examples = ROOT/'calibration_node/config/examples'
        FakeNode.overrides = {'left_yaml': str(examples/'left.yaml'), 'right_yaml': str(examples/'right.yaml'), 'allow_example_calibration': True}
        self.node = self.depth.DepthEstimationNode()

    def observation(self, instance='gate_01', source='object_detection', stamp=10):
        msg = Message()
        msg.header.stamp.sec = stamp
        msg.object_type, msg.instance_id, msg.source = 'gate', instance, source
        msg.calibration_id = self.node.left.calibration_id
        # Independent projection of (0,0,1.2) through the example camera pair.
        msg.u_left, msg.v_left, msg.u_right, msg.v_right = 160., 120., 130., 120.
        msg.confidence, msg.pixel_stddev_px = .9, .5
        return msg

    def test_missing_tf_withholds_map_then_retries_exact_stamp(self):
        self.node.observe(self.observation(), 'gate', 'object_detection')
        self.assertEqual(len(self.node.positions['gate'].messages), 0)
        self.assertFalse(self.node.selections['gate'].messages[-1].map_transform_available)
        self.node.buffer.available = True
        self.node.flush()
        output = self.node.positions['gate'].messages[-1]
        self.assertEqual(output.header.frame_id, 'map')
        np.testing.assert_allclose([output.position.x, output.position.y, output.position.z], [10,20,31.2], atol=1e-8)
        self.assertFalse(output.includes_localization_uncertainty)
        self.assertEqual(self.node.buffer.requests[-1], ('map', 'front_left_rect_optical_frame', 10_000_000_000))
        self.node.flush()
        self.assertEqual(len(self.node.positions['gate'].messages), 1)

    def test_per_instance_tracks_do_not_mix_and_same_stamp_is_not_duplicated(self):
        self.node.buffer.available = True
        self.node.observe(self.observation('gate_01'), 'gate', 'object_detection')
        self.node.observe(self.observation('gate_02'), 'gate', 'object_detection')
        self.node.observe(self.observation('gate_01', 'color_threshold'), 'gate', 'color_threshold')
        outputs = self.node.positions['gate'].messages
        self.assertEqual([out.instance_id for out in outputs], ['gate_01','gate_02'])
        self.assertEqual(self.node.selections['gate'].messages[-1].selected_source, 'color_threshold')

    def test_stale_wrong_frame_and_wrong_calibration_are_rejected(self):
        for mutate in [lambda m: setattr(m.header, 'frame_id','raw_camera'),
                       lambda m: setattr(m, 'calibration_id','wrong'),
                       lambda m: setattr(m.header.stamp, 'sec',8),
                       lambda m: setattr(m.header.stamp, 'sec',11)]:
            msg = self.observation()
            mutate(msg)
            self.node.observe(msg, 'gate', 'object_detection')
        self.assertFalse(self.node.tracks)

    def test_lost_observations_emit_invalid_feedback_and_expire(self):
        self.node.observe(self.observation(), 'gate', 'object_detection')
        self.node.clock.nanoseconds += 600_000_000
        self.node.flush()
        self.assertFalse(self.node.selections['gate'].messages[-1].valid)
        self.node.clock.nanoseconds += 2_000_000_000
        self.node.flush()
        self.assertFalse(self.node.tracks)

    def test_source_candidates_too_far_apart_are_not_compared(self):
        self.node.observe(self.observation(), 'gate', 'object_detection')
        self.node.clock.nanoseconds += 300_000_000
        msg = self.observation(source='color_threshold')
        msg.header.stamp.nanosec = 300_000_000
        self.node.observe(msg, 'gate', 'color_threshold')
        self.assertEqual(self.node.selections['gate'].messages[-1].selected_source, 'color_threshold')

    def test_rectifier_preserves_image_pair_time_and_rejects_wrong_size(self):
        node = self.calibration.CalibrationNode()
        image = np.zeros((240,640,3), np.uint8)
        image[:, :320] = [0,10,20]
        image[:, 320:] = [30,40,50]
        node.bridge = NS(imgmsg_to_cv2=lambda *a, **k: image,
                         cv2_to_imgmsg=lambda array, **k: NS(pixels=array.copy(), header=None))
        msg = Message()
        msg.width, msg.height = 640,240
        original_header = copy.deepcopy(msg.header)
        node.process(msg)
        left, right = [pub.messages[-1] for pub in node.publishers_]
        self.assertEqual(left.header.stamp.sec, original_header.stamp.sec)
        self.assertEqual(left.header.frame_id, 'front_left_rect_optical_frame')
        self.assertEqual(right.header.frame_id, 'front_right_rect_optical_frame')
        np.testing.assert_array_equal(left.pixels[120,160], [0,10,20])
        np.testing.assert_array_equal(right.pixels[120,160], [30,40,50])
        self.assertEqual(msg.header.frame_id, original_header.frame_id)
        msg.width = 1280
        node.process(msg)
        self.assertEqual(len(node.publishers_[0].messages), 1)


if __name__ == '__main__':
    unittest.main()
