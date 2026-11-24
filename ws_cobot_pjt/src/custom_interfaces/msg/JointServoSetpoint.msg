# Internal driver command. Controller sample time uses the Doosan RT clock.
string run_id
uint64 sequence
float64 source_time_s
float64[6] position_deg
float64[6] velocity_deg_s
float64 servo_time_s  # servoj_rt target time: the controller reaches each setpoint in this time (0.01-0.1 s)
