"""Apply calibration at runtime; publish images, never calibration scalar topics."""
from copy import deepcopy

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .models import CameraModel



class BottomCalibrationNode(Node):
    def __init__(self):
        super().__init__('bottom_calibration')
        defaults = {
            'input_topic': '/bottom/image_raw', 'bottom_yaml': '',
            'output_topic': '/bottom/image_rect', 'allow_example_calibration': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.model = CameraModel.load(p('bottom_yaml'), p('allow_example_calibration'))
        self.maps = self.model.maps()
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, p('output_topic'), qos_profile_sensor_data)
        self.sub = self.create_subscription(Image, p('input_topic'), self.process, qos_profile_sensor_data)
        self.get_logger().info(f'Loaded bottom color calibration once; calibration_id={self.model.calibration_id}')

    def process(self, msg):
        if (msg.width, msg.height) != self.model.size:
            self.get_logger().error(f'Expected {self.model.width}x{self.model.height} color input, received {msg.width}x{msg.height}', throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rectified = cv2.remap(image, *self.maps, interpolation=cv2.INTER_LINEAR)
            out = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
            out.header = deepcopy(msg.header)
            out.header.frame_id = self.model.frame_id
            self.publisher_.publish(out)
        except (CvBridgeError, cv2.error, ValueError, RuntimeError) as error:
            self.get_logger().error(f'Bottom color rectification failed: {error}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BottomCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
