# Primary references

- [Hornet 9 camera repository, reviewed commit](https://github.com/hornetxauv/h9_camera_ws/tree/b2f5b65507e74cbada62e6a44cd9a10d4445c809): modular camera processing, calibration storage, detection/debug separation.
- [OpenCV calibration, stereoRectify, triangulation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html): pinhole projection and calibration operations.
- [Jazzy tf2 Python Buffer](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/tf2_ros/buffer.py): transform lookup at a requested time.
- [ROS coordinate conventions, REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst): units and optical/body axes.
- [ROS frame conventions, REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst): map, odom, and base frames.
- [Jazzy Image message](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Image.msg): image metadata, encoding, and acquisition header.

Implementation choices, topic names, switching thresholds, and team assignments in this repository are our design decisions, not claims about what Hornet 9 deployed.
