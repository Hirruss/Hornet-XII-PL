# Design decisions and scope

- Component folders can contain several nodes and tools. Front and bottom cameras own separate calibration nodes, offline commands, models and YAMLs. Depth estimation reads stereo geometry from the front camera package.
- The bottom camera pipeline preserves BGR8 color. It has no stereo mode or grayscale output branch.
- Consumers read immutable calibration YAMLs at startup. They check a shared calibration ID and per-lens image size. Changing calibration requires coordinated producer/consumer restarts. No scalar focal-length/baseline publishers or CameraInfo publishers are introduced.
- The calibration CLI writes measured geometry to a new directory. Example YAMLs remain explicitly synthetic and blocked by default. Legacy ROS calibration files can be imported by adding `units: metres`, a shared calibration ID, correct rectified-frame IDs, and truthful example metadata after verifying provenance/scale.
- Source selection is per class AND physical instance. No cross-method association or averaging is invented. Hysteresis plus uncertainty gating controls preferred source, with a fresh valid fallback. The distances/limits are starting configuration, not experimentally validated vehicle thresholds.
- Images use sensor-data QoS; sparse observation/result topics use reliable depth 10. Timestamps remain image acquisition time. TF is looked up at that time, with bounded observation lifetime and no latest-pose shortcut.
- The runtime estimates uncertainty from pixel noise only. It cannot know actual error without truth, nor remove localization drift by changing coordinate frames. The message explicitly marks omitted localization uncertainty.
- No full bringup, ML model, HSV detector, EKF, optical-flow implementation, fake localization, or sample bags are supplied. Their owners have concrete interface and acceptance requirements.
- No upstream source files were copied. Hornet 9 is an architectural reference; the Jazzy tutorial helpers in the surrounding workspace informed the geometry. Do not reuse Hornet 9's camera calibration for Hornet XII hardware.
