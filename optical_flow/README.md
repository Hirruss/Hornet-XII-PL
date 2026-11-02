# Optical flow node 

**Owner:** unassigned. README-only work package; no motion estimator is supplied.

Add `optical_flow_node.py`, tracking/robust-estimation helpers, configuration, package/launch and quantitative tests. Input `/bottom/image_rect` (Image, sensor-data QoS), saved bottom calibration, timestamped camera/robot orientation and a valid scale/altitude source. Output `/bottom/flow/odometry` (nav_msgs/Odometry) with metric velocity, declared reference/child frames, acquisition timestamps and covariance; agree the velocity frame with the EKF owner.

Pixel flow alone is not metres/second. Pressure depth below the surface is not bottom altitude. Supply a known surveyed floor/plane or a valid altitude/range measurement. Compensate camera rotation using synchronized IMU/pose information and measured extrinsics. Do not claim a complete independent 6-DOF pose or angular velocity/quaternion from one unscaled planar flow field.

Done when stationary-camera noise, known translation, in-place rotation, texture loss, changing altitude, timestamp gaps and outliers are tested. Invalid/scale-unknown output must be withheld or explicitly marked unusable; never publish fabricated zero motion as a confident measurement.
