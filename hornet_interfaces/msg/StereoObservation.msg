# Exactly one associated object in a synchronized rectified stereo pair.
# header.stamp is image acquisition time; frame_id is the LEFT rectified frame.
std_msgs/Header header
string calibration_id
string object_type
# Stable physical-object identity shared by the two methods. Not just class ID.
string instance_id
# object_detection or color_threshold; must agree with the input topic.
string source
# Local pixel coordinates in each per-lens image, not combined-image coordinates.
float64 u_left
float64 v_left
float64 u_right
float64 v_right
# Positive one-sigma uncertainty per pixel coordinate; not model confidence.
float64 pixel_stddev_px
# Detection/segmentation confidence in [0,1].
float64 confidence
