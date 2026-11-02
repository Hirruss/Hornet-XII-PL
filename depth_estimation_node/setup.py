from glob import glob
from setuptools import find_packages, setup

setup(
    name='hornet_depth', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hornet_depth']),
        ('share/hornet_depth', ['package.xml']),
        ('share/hornet_depth/launch', glob('launch/*.launch.py')),
        ('share/hornet_depth/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Hornet XII PL subteam', maintainer_email='maintainers@example.invalid',
    description='Object stereo depth and map coordinates', license='LicenseRef-Proprietary',
    entry_points={'console_scripts': ['depth_estimation_node = hornet_depth.node:main']},
)
