# Calibration and depth setup

Target: **Ubuntu 24.04, ROS 2 Jazzy, system Python 3.12**. Commands below assume Jazzy is already installed. Deactivate Conda before using ROS packages.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-numpy python3-yaml ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros ros-jazzy-usb-cam ros-jazzy-rqt-image-view

mkdir -p ~/hornet_ws/src
cd ~/hornet_ws/src
# Clone the private repository with your normal GitHub credentials.
git clone https://github.com/Hirrus177013/Hornet-XII-PL.git
cd ~/hornet_ws
# Run sudo rosdep init once if rosdep has never been initialized on this machine.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to hornet_depth
source install/setup.bash
```

`colcon` discovers the three supplied packages directly in this checkout. Do not create a nested workspace inside the repository. The interface package is generated before the Python consumers. YAMLs and launch files are included in the package installation.

### Calibrate first

The three YAMLs in `calibration_node/config/examples/` are **synthetic test examples**. Runtime nodes reject them by default. Do not replace the example flag with `false` to make them look measured. Use your own camera calibration or run the included offline tool:

```bash
ros2 run hornet_calibration calibrate_cameras \
  --mode stereo --left-dir ~/calibration_images/left --right-dir ~/calibration_images/right \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --output-dir ~/hornet_calibration/front_run_01

ros2 run hornet_calibration calibrate_cameras \
  --mode mono --images-dir ~/calibration_images/bottom \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --output-dir ~/hornet_calibration/bottom_run_01
```

The board settings are examples: measure your actual square size and count **inner corners**. Use matching stereo filenames, synchronized captures, varied board poses, fixed focus, and the deployment capture mode. At least 12 accepted views are required by default. Existing output directories are never overwritten. See the [calibration procedure](../calibration_node/README.md).

### Run calibration and depth nodes

Run each command in a separate terminal after sourcing Jazzy and your workspace. These examples require the camera owners' publishers; they do not start drivers.

```bash
ros2 launch hornet_calibration rectify.launch.py \
  left_yaml:="$HOME/hornet_calibration/front_run_01/left.yaml" \
  right_yaml:="$HOME/hornet_calibration/front_run_01/right.yaml"

ros2 launch hornet_calibration rectify.launch.py mode:=bottom_mono \
  node_name:=bottom_calibration input_topic:=/bottom/image_raw \
  bottom_yaml:="$HOME/hornet_calibration/bottom_run_01/bottom.yaml"

ros2 launch hornet_depth depth.launch.py \
  left_yaml:="$HOME/hornet_calibration/front_run_01/left.yaml" \
  right_yaml:="$HOME/hornet_calibration/front_run_01/right.yaml"
```

Depth estimation also requires detection observations and an exact-time TF chain to the **rectified left optical frame**. Until the localization/extrinsic work is supplied, it reports selection feedback and withholds map positions. It never substitutes an identity transform or the latest robot pose.
