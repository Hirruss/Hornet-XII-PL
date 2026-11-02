# Interface contract, version 0.1

The `.msg` files in `hornet_interfaces/msg/` are the source of truth. Do not replace them with unlabelled numeric arrays. All time values are ROS acquisition timestamps in the same clock domain. Use `/clock` and `use_sim_time` consistently when replaying bags.

## Images

Image publishers/subscribers use ROS sensor-data QoS: best effort, volatile, depth 5. Rectification preserves the acquisition stamp, publishes `bgr8`, rejects dimensions inconsistent with YAML, and sets the correct rectified optical frame. Front input is **horizontal side-by-side** with equal lens views and no separator. A 320×240 per-lens calibration requires 640×240 combined capture. No automatic image resizing is performed.

| Image | Frame |
|---|---|
| `/front/left/image_rect` | `front_left_rect_optical_frame` |
| `/front/right/image_rect` | `front_right_rect_optical_frame` |
| `/bottom/image_rect` | `bottom_rect_optical_frame` |

The two front views come from one input message and share a stamp. This is a software pairing guarantee; confirm that the physical camera synchronizes its exposures. Black rectification borders are not valid object centers.

## StereoObservation

For each `<type>`, object detection publishes `/front/detections/<type>` with `source=object_detection`; color thresholding publishes `/front/color/<type>` with `source=color_threshold`. Use reliable, volatile, KEEP_LAST depth 10 on both sides. Publish once per object per image pair; publish nothing for an absent object, not a fake zero pixel or old center. Depth expires old candidates.

- `header.stamp`: original stereo image timestamp, never inference completion time. `header.frame_id`: left rectified optical frame.
- `calibration_id`: copy the ID from the same saved YAML used by rectification. Consumers load that YAML at startup too. Do not apply pixel undistortion a second time.
- `object_type`: same class as the topic. Class names use lowercase letters, digits, and underscores; start with a letter.
- `instance_id`: stable identity of one physical target, shared across left/right and across methods. Example `gate_01`. It is not a bounding-box index that changes every frame. Use explicit mission target association when only one target is expected; do not assume this for multiple drums/flares.
- `u_left,v_left,u_right,v_right`: corresponding center coordinates in each per-lens **rectified** image, measured in pixels, with origin top left. Do not use combined-image right-half offsets.
- `pixel_stddev_px`: positive, estimated one-sigma pixel noise used for covariance propagation. Estimate from labelled held-out data. It is not `1-confidence` or a proven bound.
- `confidence`: method-specific confidence from 0 to 1. Current default minimum is 0.25.

Class membership alone does not establish stereo correspondence. Both views must refer to the same object and geometrically corresponding point. Bounding-box centers or silhouette centroids can shift with perspective/occlusion; establish a consistent keypoint/center definition and validate its 3D bias. Reject unmatched/ambiguous centers rather than inventing a match. See the producer folder acceptance criteria.

## ObjectPosition

Front output: `/objects/<type>/position`. Planned bottom output: `/bottom/objects/<type>/position`. These remain separate until a mapping/fusion owner implements identity reconciliation. Subscribers must use `instance_id`, `source`, and acquisition time; the last message is not necessarily the only instance of that class.

`header.frame_id=map` by default; `position` is XYZ in metres. `position_covariance` is row-major 3×3 in metres squared, rotated into that map frame. Current front output has `includes_localization_uncertainty=false`: uncertainty from robot pose, extrinsics, calibration, model bias, and association is excluded. Downstream fusion must account for it; the value is not full map-position certainty. No object orientation is estimated.

The node publishes at most one position per increasing measurement timestamp per instance. If a new measurement from the other source has the same timestamp, selection feedback may change but the position is not republished. This avoids duplicate measurements entering a filter. Map positions are withheld without exact-time TF. No TF fallback uses `Time()`/latest.

## SourceSelection

Topic: `/perception/selection/<type>`, same reliable QoS. Every feedback message includes an instance ID. `selected_source` is the source that currently has a usable estimate; `preferred_source` is the distance-based desired mode. A fallback may make them differ. `valid=false` / empty source means no valid fresh measurement; no-target feedback can be absent until a track has first been observed.

- Far range prefers object detection.
- At or below `enter_color_m` (default 2 m), prefer color thresholding.
- Stay in color mode until range reaches `exit_color_m` (default 2.5 m).
- If the preferred source is unavailable or exceeds `max_position_stddev_m`, use another fresh valid source; if neither is usable, publish no position.
- The preferred range anchor is a usable ML estimate, otherwise a usable color estimate. Observations must be close in acquisition time (`max_source_skew_sec`, default 0.15 s); stale inputs expire after 0.5 s by default.
- `position_stddev_m` is the largest principal-axis one-sigma **stereo** uncertainty. It is not true error. Reprojection residual is a consistency check, not a depth-error guarantee.

Both detector owners should consume this feedback to prioritize work, but retain a low-rate probe from the inactive method so recovery and handover remain possible. Hard stopping the inactive producer can prevent handover. Switching is inside the implemented depth node; no additional selector node/folder is required.

## Planned localization interfaces

| Topic / transform | Type or meaning | Owner |
|---|---|---|
| `/sensors/imu` | `sensor_msgs/msg/Imu`: SI acceleration/angular velocity; orientation only if valid | Sensor bridge |
| `/sensors/magnetic_field` | `sensor_msgs/msg/MagneticField`, tesla | Sensor bridge |
| `/sensors/depth` | `geometry_msgs/msg/PointStamped`: z is positive-down depth in metres, x=y=0; sensor frame | Sensor bridge |
| `/bottom/flow/odometry` | `nav_msgs/msg/Odometry`: metric velocity with covariance; declared frames | Optical flow |
| `/localization/odometry` | `nav_msgs/msg/Odometry`: robot estimate with covariance | EKF/localization |
| `map → odom → base_link` | Dynamic localization transforms, as applicable to the estimator | EKF/localization |
| `base_link → camera raw optical frames` | Measured rigid extrinsics | Localization / calibration owners |
| `raw optical → rectified optical` | Rotation derived from saved rectification R, not assumed identity | Localization / calibration owners |

These planned interfaces have no live publishers in this scaffold. `PointStamped` depth must be converted into an appropriate filter measurement and vertical sign/frame by the localization owner; it is not directly a standard robot_localization input. Raw magnetic field also needs calibration/heading processing, not direct insertion as yaw. This contract intentionally does not claim IMU+pressure alone observes absolute X/Y.
