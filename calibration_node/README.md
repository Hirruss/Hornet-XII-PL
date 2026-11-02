# Calibration node — implemented

**ROS package:** `hornet_calibration` · **runtime executable:** `calibration_node` · **owner:** to assign for vehicle calibration and integration.

This folder owns calibration estimation and application. `hornet_calibration/calibrate.py` is the offline checkerboard CLI; `node.py` is the runtime rectifier; `models.py` validates and loads saved YAML. `config/examples/` contains guarded synthetic left/right/bottom YAMLs. `launch/rectify.launch.py` launches one runtime instance, front or bottom. It does not start cameras.

## Inputs and outputs

- `mode=front_stereo`: `/front/image_raw` → `/front/left/image_rect` and `/front/right/image_rect`.
- `mode=bottom_mono`: `/bottom/image_raw` → `/bottom/image_rect`; set the input topic explicitly when using the launch file.
- All image messages are `sensor_msgs/msg/Image`, sensor-data QoS. Outputs are BGR8, keep original timestamps, and use the YAML rectified frame IDs.
- YAML paths are startup parameters. There are no runtime focal-length, baseline, or CameraInfo publishers. The node does not compute disparity or depth.

Two running instances share this one node-role folder. Use unique node names (`front_calibration`, `bottom_calibration`). Front input must be equal-width horizontal stereo halves matching the YAML dimensions, not independent camera topics. `swap_input_halves=true` only fixes acquisition half ordering relative to the saved labels; it does not repair a backwards calibration.

## Offline procedure

1. Fix focus/exposure/capture resolution and mount cameras rigidly. Recalibrate if those change. Calibrate in the intended medium/optical housing configuration; validate refraction effects underwater.
2. Capture varied checkerboard poses covering image corners, distances and tilt. Use physical square size in metres. Stereo left/right captures must show the same board at the same time; save each per-lens pair with the same filename in separate directories.
3. Run the `calibrate_cameras` commands in the root README with your real inner-corner counts and square size. For imported side-by-side captures, split and label the two halves consistently before calibration.
4. The tool calibrates intrinsics, fixes those during stereo extrinsic fitting, and calls `stereoRectify`. It supports horizontal pinhole stereo and pinhole mono; fisheye/vertical stereo is out of scope. It rejects high RMS under the configured gate and refuses an existing output directory.
5. Archive the generated YAMLs and calibration provenance (camera serials, mode, focus, housing, medium, square dimensions, date). Use new run directories. Check calibration ID agreement in the left/right files.
6. Check held-out rectified pairs: corresponding features should align horizontally. Check depth at independently measured distances; low RMS alone does not establish correct stereo scale.

Generated YAML contains ROS K/D/R/P fields plus `units: metres`, `calibration_id`, `rectified_frame_id`, `is_example: false`, timestamp, and reprojection RMS. Legacy ROS calibration can be adopted only after verifying its units, stereo pairing, dimensions and provenance, then adding the required metadata. Never deploy the synthetic example values by toggling their flag.

For front depth, consumers use the **rectified P** matrices; K describes the original lens. The localization owner must establish measured camera mounting TF and raw-to-rectified rotation links from R. This node does not invent mounting transforms. See [frames](../docs/frames.md).

## Acceptance criteria

- Real calibration YAMLs for the chosen front and bottom modes, with provenance and measured baseline.
- Exactly two front rectified image topics and one bottom image topic across two instances; no continuous scalar geometry publications.
- Matching stamps, correct encoding/frames, invalid input dimensions rejected.
- Held-out image alignment and pool range validation logged in the integration issue.
- Measured camera extrinsics reviewed with the localization owner.
