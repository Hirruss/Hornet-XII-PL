# Frames, map origin, and geometry

Use metres, seconds, radians, and ROS conventions. `base_link`: X forward, Y left, Z up. Optical frames: X right, Y down, Z forward. `map` is fixed to a documented pool origin and axes; it is not a normalized camera ray or a 0–1 coordinate grid. The localization owner must document how the initial origin/heading/depth are established. A locally initialized map will drift without absolute references or loop/landmark corrections.

```text
map → odom → base_link → front_left_optical_frame → front_left_rect_optical_frame
                       → front_right_optical_frame → front_right_rect_optical_frame
                       → bottom_optical_frame → bottom_rect_optical_frame
```

The exact TF tree is an integration deliverable. Do not publish duplicate parent links or make up mounting transforms. Depth asks TF for `T_map_left_rect(t_image)` and computes:

```text
p_map = R_map_left_rect(t_image) * p_left_rect + translation_map_left_rect(t_image)
C_map = R_map_left_rect * C_stereo * transpose(R_map_left_rect)
```

This covariance rotation conditions on exact TF. It does not add pose/extrinsic uncertainty. TF messages do not themselves provide pose covariance. A future fusion component can propagate localization covariance using the pose Jacobian; consumers must not treat the current output as a complete covariance.

Saved `rectification_matrix` R maps raw optical coordinates into rectified optical coordinates: `p_rect = R * p_raw`. Therefore a TF whose parent is raw optical and child is rectified optical uses rotation **R transpose**. Confirm the direction with a known point. For nonidentity R, publishing an identity link shifts map coordinates incorrectly.

For canonical horizontal stereo:

```text
B = -P_right[0,3] / P_right[0,0]
d_effective = (u_left - u_right) - (cx_left - cx_right)
Z = fx * B / d_effective
```

The implementation uses DLT with both complete rectified P matrices and checks reprojection. The principal-point correction above explains the geometry; raw K focal lengths are not substituted for rectified P focal lengths. No dense disparity image or point cloud is needed when associated pixel centers are provided.

A single bottom-camera pixel defines a ray, not a metric XYZ point. Bottom object localization needs a known plane in `map`, or a measured altitude and floor normal plus correct camera extrinsics. Pressure depth below the water surface does not by itself give altitude above the floor. Optical flow also needs scale/altitude and rotation compensation to become metric translational velocity.
