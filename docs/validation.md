# Validation and integration handoff

## Automated local checks

From the repository root, install `requirements-test.txt`, then run `python3 -m unittest discover -s tests -v`. Tests cover independently projected stereo points, nonidentical principal points, invalid observations, covariance behavior, source hysteresis/fallback, map transformations, example guards, runtime adapters with minimal ROS stubs, and offline calibration from synthetic checkerboard images. Synthetic tests verify implementation behavior, not real vehicle accuracy.

To redraw documentation images: install `requirements-diagrams.txt`, then run `python3 tools/draw_diagrams.py`. The editable Mermaid source also records the graph.

## Required Jazzy checks (not satisfied by non-ROS unit tests)

- [ ] Build `hornet_interfaces`, `hornet_calibration`, `hornet_depth` in a clean Ubuntu 24.04/Jazzy workspace.
- [ ] Inspect generated interfaces with `ros2 interface show hornet_interfaces/msg/StereoObservation` and the other two messages.
- [ ] Run front/bottom rectification with measured YAML; inspect images, timestamps, sizes, optical frames and QoS.
- [ ] Start depth with real observations; verify class/topic/source/instance/calibration IDs agree.
- [ ] Withhold the map TF: confirm feedback reports unavailable TF and there is no map position.
- [ ] Supply measured transforms at the image times; confirm positions appear and remain stable for a stationary target as the robot moves.
- [ ] Confirm wrong calibration IDs/frames, negative disparity, mismatch, no match, stale observations and inconsistent center correspondence are rejected.
- [ ] Compare ML/color sources at matched times, including several same-class instances; verify no cross-object switching, hysteresis, valid fallback and loss feedback.
- [ ] Replay a bag with consistent `use_sim_time`; verify stale data and backward clock jumps recover without retaining old pose/track state.
- [ ] Test late TF delivery within observation expiry and avoid using latest-pose transforms.

Useful inspection commands:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /front/left/image_rect --verbose
ros2 topic echo /perception/selection/gate
ros2 topic echo /objects/gate/position
ros2 run tf2_ros tf2_echo map front_left_rect_optical_frame
rqt_graph
```

## Required measured-camera / pool checks

- [ ] Verify hardware half order, physical baseline, synchronization, capture mode and fixed focus.
- [ ] Verify held-out rectified feature alignment across the image, including the housing/medium used underwater.
- [ ] Measure a textured/defined target at several independently known ranges and bearings; log bias, spread and false associations separately.
- [ ] Estimate pixel uncertainty from held-out labels; check whether predicted stereo uncertainty is calibrated.
- [ ] Measure localization drift and mounting/rectification errors; account for missing pose uncertainty downstream.
- [ ] Validate monocular bottom scale/plane assumptions and optical-flow rotation compensation before metric outputs are accepted.

No ROS installation, physical camera, IMU, pressure sensor, or vehicle localization is assumed by this scaffold. Fill in actual hardware results in the team issue before deployment.
