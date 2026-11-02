from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('hornet_depth'))
    calibration = Path(get_package_share_directory('hornet_calibration')) / 'config/examples'
    defaults = {'left_yaml': str(calibration / 'left.yaml'), 'right_yaml': str(calibration / 'right.yaml'),
                'params_file': str(share / 'config/depth.yaml'), 'allow_example_calibration': 'false'}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='hornet_depth', executable='depth_estimation_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'left_yaml': LaunchConfiguration('left_yaml'), 'right_yaml': LaunchConfiguration('right_yaml'),
                 'allow_example_calibration': ParameterValue(LaunchConfiguration('allow_example_calibration'), value_type=bool),
             }]),
    ])
