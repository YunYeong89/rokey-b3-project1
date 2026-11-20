# One sample of a contact scan: the ProbeTCP and the wrench of the same controller tick. BASE frame, SI units.
# Measurement only: nothing the scan commands depends on these values.
uint8 TRAVEL=0                     # away from the surface (approach, lift, HOME)
uint8 PRESS=1                      # closing in and ramping to the scan force
uint8 STROKE=2                     # moving along the segment in contact
uint8 RELEASE=3                    # force released, backing off
float64 sampled_at                 # controller (simulation) time of the sample, s
uint64 sequence                    # source sample number: gaps are dropped samples
uint8 phase
uint32 path                        # plan path (one pass), from 1
uint32 segment                     # segment of the run (one continuous contact run), from 1
geometry_msgs/Vector3 axis         # probe axis, unit, out of the surface (the fixed axis or the surface normal)
geometry_msgs/Pose tcp             # active ProbeTCP
geometry_msgs/Wrench wrench        # measured wrench about the ProbeTCP
float64 force_n                    # reaction along the axis, positive in contact
bool contact                       # force_n above the touch threshold
geometry_msgs/Point point          # surface point under the probe ball (tcp less the ball radius); valid in contact
