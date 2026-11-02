# Bottom camera

This component owns the bottom **color camera**, its calibration, and color rectification. It may contain several ROS nodes. ROS package: `hornet_bottom_camera`.

## Files

- `hornet_bottom_camera/calibration.py`: dedicated `bottom_calibration` runtime node, preserving BGR8 color.
- `hornet_bottom_camera/calibrate.py`: dedicated `calibrate_camera` offline checkerboard command for this camera.
- `hornet_bottom_camera/models.py`: saved camera model and rectification-map loading.
- `config/examples/bottom.yaml`: synthetic example geometry, disabled by default.
- `config/rectification.yaml`, `launch/rectify.launch.py`: bottom camera configuration and launch.
- Add capture-driver configuration/launch and any `.py` acquisition adapter in this component. Record stable device identity, encoding, focus, housing, capture size and frame rate.

## Interfaces

The camera driver publishes `/bottom/image_raw` (`sensor_msgs/msg/Image`, sensor-data QoS). `bottom_calibration` reads `bottom.yaml` once, corrects geometry, and publishes `/bottom/image_rect` in **BGR8 color**, preserving acquisition time and setting `bottom_rect_optical_frame` from the YAML. Both optical flow and color thresholding consume this image.

There is no grayscale image publisher, stereo splitter, stereo matcher, left/right configuration or selectable camera mode in this package. Offline checkerboard corner detection internally uses intensity data; it does not change the runtime color pipeline. Calibration information is read from files, not streamed as focal-length/baseline topics.

## Offline calibration

Capture varied checkerboard poses with the actual housing/medium, fixed focus and chosen capture mode. Measure square size in metres and count inner corners. The settings below are examples.

```bash
ros2 run hornet_bottom_camera calibrate_camera \
  --images-dir ~/calibration_images/bottom \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --output-dir ~/hornet_calibration/bottom_run_01
```

The command writes a bottom-camera YAML with K/D/R/P, dimensions, units, frame ID, calibration ID and provenance metadata. It requires at least 12 accepted views by default and refuses existing output directories. Validate held-out image geometry; do not use the synthetic example for the vehicle.

## Runtime

```bash
ros2 launch hornet_bottom_camera rectify.launch.py \
  bottom_yaml:="$HOME/hornet_calibration/bottom_run_01/bottom.yaml"
```

Run the capture driver separately and match the saved image dimensions. Agree timestamps and measured camera extrinsics with localization. A single color camera does not provide metric depth by itself; bottom object localization and metric flow require a known plane or altitude/scale source. See [interfaces](../docs/interfaces.md) and [frames](../docs/frames.md).
