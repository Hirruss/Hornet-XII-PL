from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('hornet_front_camera'))
    defaults = {
        'left_yaml': str(share / 'config/examples/left.yaml'),
        'right_yaml': str(share / 'config/examples/right.yaml'),
        'input_topic': '/front/image_raw', 'allow_example_calibration': 'false',
    }
    parameters = {key: LaunchConfiguration(key) for key in defaults}
    parameters['allow_example_calibration'] = ParameterValue(LaunchConfiguration('allow_example_calibration'), value_type=bool)
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='hornet_front_camera', executable='front_calibration', output='screen',
             parameters=[str(share / 'config/rectification.yaml'), parameters]),
    ])
