# Front camera node — to implement

**Owner:** unassigned. This folder intentionally contains only this README. Integrate an existing Jazzy camera driver (the tutorials used `usb_cam`) rather than writing an unnecessary USB driver.

## Add here

- A node adapter `.py` only if the driver needs adaptation, a package manifest, launch file and camera configuration YAML.
- Hardware/device notes: serial identity, stable device path, supported format/resolution/FPS, exposure synchronization and focus.
- A short reproducible live-camera and bag test.

## Interface

Publish `/front/image_raw`, `sensor_msgs/msg/Image`, sensor-data QoS. It is horizontal side-by-side stereo, equal halves, no separator. Use image acquisition timestamps and a consistent raw frame. Do not publish rectified images, depth, or scalar calibration.

Match the chosen per-lens calibration exactly. For 320×240 per-lens YAML use 640×240 combined; 1280×480 capture needs matching 640×480 per-lens calibration. Portability matters: avoid developer-specific `/home/...` paths and assumed `/dev/video0` identity. Confirm stereo half order with the calibration owner.

## Done when

Stable capture and correct encoding/size/stamps are demonstrated at the chosen mode; reconnection/device selection is documented; exposure synchronization limitations are measured; calibration node accepts the stream. Do not resize a different capture mode to hide mismatched geometry.
