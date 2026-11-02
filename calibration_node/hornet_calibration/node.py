"""Apply calibration at runtime; publish images, never calibration scalar topics."""
from copy import deepcopy

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .models import CameraModel, validate_pair


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        defaults = {
            'mode': 'front_stereo', 'input_topic': '/front/image_raw',
            'left_yaml': '', 'right_yaml': '', 'bottom_yaml': '',
            'left_topic': '/front/left/image_rect', 'right_topic': '/front/right/image_rect',
            'bottom_topic': '/bottom/image_rect', 'swap_input_halves': False,
            'allow_example_calibration': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.mode, self.swap = p('mode'), p('swap_input_halves')
        allow = p('allow_example_calibration')
        if self.mode == 'front_stereo':
            self.models = [CameraModel.load(p('left_yaml'), allow), CameraModel.load(p('right_yaml'), allow)]
            baseline = validate_pair(*self.models)
            topics = [p('left_topic'), p('right_topic')]
            self.get_logger().info(f'Loaded stereo geometry once; baseline {baseline:.5f} m')
        elif self.mode == 'bottom_mono':
            self.models = [CameraModel.load(p('bottom_yaml'), allow)]
            topics = [p('bottom_topic')]
        else:
            raise ValueError('mode must be front_stereo or bottom_mono')
        self.maps = [model.maps() for model in self.models]
        self.bridge = CvBridge()
        self.publishers_ = [self.create_publisher(Image, topic, qos_profile_sensor_data) for topic in topics]
        self.sub = self.create_subscription(Image, p('input_topic'), self.process, qos_profile_sensor_data)
        self.get_logger().info(f'Rectifying {self.mode}; calibration_id={self.models[0].calibration_id}')

    def process(self, msg):
        model = self.models[0]
        expected_width = model.width * len(self.models)
        if (msg.width, msg.height) != (expected_width, model.height):
            self.get_logger().error(f'Expected {expected_width}x{model.height}, received {msg.width}x{msg.height}', throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            views = [image] if len(self.models) == 1 else [image[:, :model.width], image[:, model.width:]]
            if self.swap and len(views) == 2:
                views.reverse()
            for view, calibration, maps, publisher in zip(views, self.models, self.maps, self.publishers_):
                rectified = cv2.remap(view, *maps, interpolation=cv2.INTER_LINEAR)
                out = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
                out.header = deepcopy(msg.header)
                out.header.frame_id = calibration.frame_id
                publisher.publish(out)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError) as error:
            self.get_logger().error(f'Rectification failed: {error}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
