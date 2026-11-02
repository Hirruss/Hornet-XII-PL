from glob import glob
from setuptools import find_packages, setup

setup(
    name='hornet_calibration', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hornet_calibration']),
        ('share/hornet_calibration', ['package.xml']),
        ('share/hornet_calibration/launch', glob('launch/*.launch.py')),
        ('share/hornet_calibration/config', glob('config/*.yaml')),
        ('share/hornet_calibration/config/examples', glob('config/examples/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Hornet XII PL subteam', maintainer_email='maintainers@example.invalid',
    description='Offline calibration and runtime rectification', license='LicenseRef-Proprietary',
    entry_points={'console_scripts': [
        'calibration_node = hornet_calibration.node:main',
        'calibrate_cameras = hornet_calibration.calibrate:main',
    ]},
)
