"""Apply calibration at runtime; publish images, never calibration scalar topics."""
from copy import deepcopy

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .models import CameraModel, validate_pair



class FrontCalibrationNode(Node):
    def __init__(self):
        super().__init__('front_calibration')
        defaults = {
            'input_topic': '/front/image_raw', 'left_yaml': '', 'right_yaml': '',
            'left_topic': '/front/left/image_rect', 'right_topic': '/front/right/image_rect',
            'swap_input_halves': False, 'allow_example_calibration': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.swap = p('swap_input_halves')
        allow = p('allow_example_calibration')
        self.models = [CameraModel.load(p('left_yaml'), allow), CameraModel.load(p('right_yaml'), allow)]
        baseline = validate_pair(*self.models)
        self.maps = [model.maps() for model in self.models]
        self.bridge = CvBridge()
        self.publishers_ = [self.create_publisher(Image, p(topic), qos_profile_sensor_data)
                            for topic in ('left_topic', 'right_topic')]
        self.sub = self.create_subscription(Image, p('input_topic'), self.process, qos_profile_sensor_data)
        self.get_logger().info(f'Loaded front stereo calibration once; baseline={baseline:.5f} m; calibration_id={self.models[0].calibration_id}')

    def process(self, msg):
        model = self.models[0]
        if (msg.width, msg.height) != (model.width * 2, model.height):
            self.get_logger().error(f'Expected {model.width*2}x{model.height} side-by-side input, received {msg.width}x{msg.height}', throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            views = [image[:, :model.width], image[:, model.width:]]
            if self.swap:
                views.reverse()
            for view, calibration, maps, publisher in zip(views, self.models, self.maps, self.publishers_):
                rectified = cv2.remap(view, *maps, interpolation=cv2.INTER_LINEAR)
                out = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
                out.header = deepcopy(msg.header)
                out.header.frame_id = calibration.frame_id
                publisher.publish(out)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError) as error:
            self.get_logger().error(f'Front rectification failed: {error}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FrontCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
