# One sample from the physical robot's existing Doosan RT stream.
float64 time_stamp  # Controller acquisition time [s]; use for sample intervals.
builtin_interfaces/Time read_stamp  # Local ros2_control read cycle clock, separate from the controller clock.

# Joint order: J1 through J6. Unqualified position/velocity are motor-side.
float64[6] actual_joint_position_deg
float64[6] actual_joint_position_abs_deg  # Link-side absolute encoder.
float64[6] actual_joint_velocity_deg_s
float64[6] actual_joint_velocity_abs_deg_s
float64[6] target_joint_acceleration_deg_s2  # Commanded, derived actual acceleration is computed by the subscriber.

# BASE frame. Pose orientation is Doosan ZYZ. Velocity uses the SDK's linear/angular convention.
float64[6] actual_tcp_pose_mm_deg
float64[6] actual_tcp_velocity_mm_deg_s
float64[6] actual_flange_pose_mm_deg
float64[6] external_tcp_wrench_n_nm  # Estimated [Fx, Fy, Fz, Mx, My, Mz] at the TCP.

float64[6] raw_joint_torque_nm
float64[6] external_joint_torque_nm
float64[6] gravity_torque_nm
float64[6] joint_temperature_c
float64[36] jacobian_matrix  # Row-major BASE Jacobian in the SDK's native units.
float64 singularity  # Minimum singular value reported by the SDK.
uint16 solution_space
uint8 robot_mode
uint8 robot_state
uint16 control_mode
