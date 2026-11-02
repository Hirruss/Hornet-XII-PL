# Localization 

**Owner:** unassigned. README-only work package. Evaluate an existing ROS estimator such as robot_localization and supply an adapter/configuration if suitable; do not duplicate it without a reason.

## Files and responsibilities

Place the EKF node, map-management nodes/tools, filter YAML, sensor measurement adapters, launch files, extrinsic configuration and simulation integration in this component. Fuse IMU, corrected depth and metric optical-flow odometry, publish `/localization/odometry` with covariance, and own the dynamic TF chain. Document map origin, heading reference, vertical datum and initialization.

Supply `map → odom → base_link` as appropriate and measured mounting/extrinsic links through the camera raw and rectified optical frames. Saved R matrices need the correct TF direction; see [frames](../docs/frames.md). Depth estimation requests `map ← front_left_rect_optical_frame` at image time. Publishing odometry without the corresponding TF chain will not satisfy that interface.

IMU and pressure do not observe absolute X/Y; optical flow needs altitude/plane scale. A map initialized at startup can drift. Define known-landmark or other absolute corrections if stable surveyed coordinates are required. Do not feed object positions derived from this same filter's pose back into it as independent absolute measurements; that double-counts correlated information. A future mapping module must handle object identities and covariance/correlation explicitly.

Do not fuse raw MagneticField or the agreed depth PointStamped directly into robot_localization; adapt to supported, correctly framed measurements. Avoid duplicate TF authorities.

## Done when

Initialization, stationary drift, measured paths, depth sign, yaw/rotation, covariance, sensor dropout and bag clock jumps are tested. A fixed known target remains stable in map while the robot moves. Publish pose uncertainty and agree how downstream object fusion will include it; the current depth output explicitly excludes localization uncertainty.
