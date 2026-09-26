# Six-leg contact state, in canonical leg order (see hexapod_locomotion's
# LEGS tuple: front_left, middle_left, rear_left, front_right, middle_right,
# rear_right). force_n is 0.0 on real hardware until a force-sensitive
# limit switch is fitted -- treat it as sim-only until then.
std_msgs/Header header
bool[6] contact
float64[6] force_n
