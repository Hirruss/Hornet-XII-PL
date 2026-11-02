# Object detection node — to implement

**Owner:** unassigned. This folder is intentionally README-only. Its task is detection and stereo association, not 3D estimation.

## Files to add

- `object_detection_node.py`: ROS subscriptions, inference scheduling, observations and selection feedback.
- Model loading/inference helper `.py`, class mapping and inference configuration.
- `stereo_association.py`: left/right matching and stable physical-instance identities, shared with the color owner as a documented contract.
- Package/launch, tests, and model/dataset version documentation. Keep weights and datasets outside Git.

## Interface

Subscribe to `/front/left/image_rect` and `/front/right/image_rect` (Image, sensor-data QoS), pairing the original identical acquisition stamps. Consume `/perception/selection/<type>` (SourceSelection, reliable depth 10). Read saved calibration YAML at startup for dimensions, calibration ID and epipolar geometry; do not subscribe to focal-length/baseline topics.

Publish `/front/detections/<type>` (StereoObservation, reliable depth 10) with `source=object_detection`, left rectified frame, original acquisition stamp, a shared physical `instance_id`, both image centers, confidence and calibrated pixel uncertainty. Types initially include gate; yellow/orange/blue/red flares; blue/red drums. See [interfaces](../docs/interfaces.md).

Do not publish independent left/right centers and expect depth to discover associations. Match the same object and a consistent geometric center/keypoint. If bounding-box centers do not correspond under occlusion/perspective, reject or improve the keypoint definition. Multi-instance same-class tests are mandatory.

## Selection and acceptance

Prioritize ML at long range as indicated by feedback; retain a low-rate probe when inactive. Do not rename timestamps to inference completion time. Publish no observation for a missing/ambiguous match.

Done when class mapping/model provenance is documented; pairs, identities and centers are verified on held-out images/bags; known-position bias and pixel uncertainty are measured; multiple targets and source handover work with the depth node. Model confidence is not pixel standard deviation.
