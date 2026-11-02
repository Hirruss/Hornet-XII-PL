# Sensor bridge node — to implement

**Owner:** unassigned. README-only integration folder for existing IMU and pressure/depth drivers.

Add an adapter `.py` only if needed, package/launch, sensor configuration/calibration and recording tests. Publish `/sensors/imu` (sensor_msgs/Imu), `/sensors/magnetic_field` (sensor_msgs/MagneticField), and `/sensors/depth` (geometry_msgs/PointStamped: z=positive-down metres, x=y=0) with real sensor frames/acquisition timestamps. See [planned interfaces](../docs/interfaces.md).

Validate SI units, axes, gravity convention, covariances, zeroing, clock alignment, magnetic hard/soft iron correction and pressure-to-depth assumptions. Mark unavailable orientation appropriately instead of setting a fake valid quaternion. Raw magnetic field is not yaw. Coordinate depth-sign conversion and filter measurement adaptation with localization. Hardware drivers can remain external; this folder owns their integration contract.
