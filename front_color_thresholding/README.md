# Front color thresholding node 

**Owner:** unassigned. README-only work package for close-range stereo segmentation.

## Files to add

- `front_color_thresholding_node.py`: ROS image/feedback IO and observation publishing.
- Object-specific helper `.py` files such as `yellow_flare.py`, `blue_drum.py`, `gate.py` (only for classes color can distinguish).
- Saved HSV YAML files, a tuning utility, stereo-association/identity helper or documented shared association method, tests, package/launch.

## Interface and behavior

Subscribe to both `/front/{left,right}/image_rect` (Image, sensor-data QoS), synchronized by acquisition stamp, and `/perception/selection/<type>` (SourceSelection). Publish `/front/color/<type>` (StereoObservation, reliable depth 10), `source=color_threshold`. Read the same calibration YAML and reuse stable physical `instance_id` values agreed with ML; use correct calibration ID/frame and local per-lens coordinates.

Apply HSV bounds, morphological cleanup, shape/area gates and consistent corresponding centers/keypoints in both views. Tune underwater for the actual exposure/white balance and object appearance. Do not assume every bright contour is a target. Emit no observation for missing, clipped, occluded or ambiguous matches. Document centroid/bounding-center bias and measured pixel uncertainty.

Prioritize close targets per feedback and keep a low-rate inactive probe for recovery. Depth performs the actual source selection; do not create a second incompatible selector. Acceptance: object-specific bounds, labelled left/right masks/centers, multi-instance identity, false-positive handling, near/far handover and metric-depth bias measurements with depth owner.
