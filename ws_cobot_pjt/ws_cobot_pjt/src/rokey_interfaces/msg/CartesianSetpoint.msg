# BASE frame, metres/radians/seconds. Rotation is a row-major 3x3 matrix.
# The origin is the active ProbeTCP, never the ball centre or flange.
std_msgs/Header header
string run_id
uint64 sequence
uint64 source_sequence
float64 source_sampled_at
geometry_msgs/Point position
float64[9] rotation
geometry_msgs/Twist feedforward
float64 linear_speed
float64 linear_acceleration
float64 angular_speed
float64 angular_acceleration
float64 joint_speed
float64 joint_acceleration
float64 position_stiffness_scale 1.0
float64 position_feedback_limit 0.0005
# Deliberately no force target, contact condition, or compliance settings.
