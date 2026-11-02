# Bottom camera node — to implement

**Owner:** unassigned. README-only work package. Reuse an existing camera driver where possible.

Add the package/launch/config and any required `.py` adapter here. Publish `/bottom/image_raw` (`sensor_msgs/msg/Image`, sensor-data QoS) with acquisition timestamps and the real raw optical frame. Match the bottom YAML dimensions, fixed focus and housing configuration. Record stable USB identity, encoding, resolution and FPS.

The implemented calibration node's `bottom_mono` instance produces `/bottom/image_rect`; this driver should not duplicate rectification. Agree time synchronization with the IMU/localization owner. Acceptance: a live and replayable stream feeds rectification, optical flow, and bottom color processing without conflicting publishers or stale timestamps.
