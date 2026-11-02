# Measurement time and fixed map frame; this is a position, not object orientation.
std_msgs/Header header
string calibration_id
string object_type
string instance_id
string source
geometry_msgs/Point position
# Row-major XYZ covariance, metres squared, expressed in header.frame_id.
# Current stereo implementation conditions on exact pose and calibration.
float64[9] position_covariance
bool includes_localization_uncertainty
float64 camera_range_m
float64 reprojection_rms_px
