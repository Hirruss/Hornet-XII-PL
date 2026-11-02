# Bottom color thresholding / object localization node — to implement

**Owner:** unassigned. README-only work package, separate from front stereo thresholding.

Add `bottom_color_thresholding_node.py`, object-specific HSV helpers, saved thresholds, ray/plane geometry helper, package/launch and tests. Subscribe to `/bottom/image_rect` and timestamped localization/TF; load the saved bottom YAML once. Identify class and stable instance, then publish `/bottom/objects/<type>/position` (hornet_interfaces/ObjectPosition) in `map` with covariance. Use `source=bottom_color_threshold` and preserve image time.

One monocular center cannot provide depth. Before implementing metric output, agree a surveyed floor plane or measured altitude/normal and its uncertainty with localization. Pressure depth by itself is not altitude above the floor. Intersect the rectified camera ray with the valid map plane; reject parallel, behind-camera or otherwise invalid intersections. Account for camera mounting and pose at image time. If geometry/TF is unavailable, withhold metric output; optionally define a separate labelled pixel-observation diagnostic with the interface reviewer.

This is not connected to the front stereo depth node. Keep bottom output topics separate until identity-aware object-map fusion is implemented. Any front selection feedback used to coordinate targets must reference the same physical identity and must not be treated as a bottom depth measurement.

Done when underwater thresholds, floor/altitude assumptions, mask/centroid bias, world-coordinate accuracy, uncertainty, motion invariance and missing-TF handling are measured on known targets.
