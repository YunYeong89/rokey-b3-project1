std_msgs/Header header
string run_id
uint64 sequence
float64 sampled_at
bool ready
bool active
bool probe_attached
string reason
geometry_msgs/Point position
float64[9] rotation
geometry_msgs/Twist velocity
float64[6] joint_positions
# Wrenches are expressed in BASE about the ProbeTCP. Measurement only.
geometry_msgs/Wrench measured_wrench
geometry_msgs/Wrench filtered_wrench
float64 joint_age_s
float64 sensor_age_s
float64 tracking_error_m
uint64 dropped_setpoints
bool pd_enabled
# Pose errors and P/D/offset contributions: [metres x/y/z, radians x/y/z], BASE frame.
float64[6] pd_error
float64[6] pd_p
float64[6] pd_d
float64[6] pd_offset
geometry_msgs/Twist pd_velocity_error
bool pd_saturated
