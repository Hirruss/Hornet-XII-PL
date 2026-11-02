# Feedback publication time; frame_id is the left rectified optical frame.
std_msgs/Header header
builtin_interfaces/Time measurement_stamp
string object_type
string instance_id
# Empty selected_source means no valid fresh stereo measurement.
string selected_source
string preferred_source
bool valid
bool map_transform_available
# NaN when valid=false. Uncertainty is not true ground-truth position error.
float64 camera_range_m
float64 position_stddev_m
string reason
