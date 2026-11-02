from glob import glob
from setuptools import find_packages, setup

setup(
    name='hornet_bottom_camera', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hornet_bottom_camera']),
        ('share/hornet_bottom_camera', ['package.xml']),
        ('share/hornet_bottom_camera/launch', glob('launch/*.launch.py')),
        ('share/hornet_bottom_camera/config', glob('config/*.yaml')),
        ('share/hornet_bottom_camera/config/examples', glob('config/examples/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Hornet XII PL subteam', maintainer_email='maintainers@example.invalid',
    description='Offline calibration and runtime rectification', license='LicenseRef-Proprietary',
    entry_points={'console_scripts': [
        'bottom_calibration = hornet_bottom_camera.calibration:main',
        'calibrate_camera = hornet_bottom_camera.calibrate:main',
    ]},
)
