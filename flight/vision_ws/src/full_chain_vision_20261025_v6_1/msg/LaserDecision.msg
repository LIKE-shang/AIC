# Camera capture header, never replaced with processing time.
std_msgs/Header header
bool enable_laser
bool observation_valid
bool request_on
uint32 hit_count
uint32 miss_count
# Additional independent three-ring/aim interlocks checked by the bridge.
bool target_valid
bool aim_valid
bool calibration_valid
# Image evidence only, NOT service ACK and NOT competition acceptance.
bool observed_hit
float64 continuous_seconds
string feedback_mode
string reason
