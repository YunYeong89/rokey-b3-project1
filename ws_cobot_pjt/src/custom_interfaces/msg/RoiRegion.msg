# One region of interest of a scan (part_scan/roi.py ScanRoi): an axis-aligned box in BASE. SI units.
string id                          # roi_001, ...
string type                        # dent, crack, deformation (geometry); force_high, force_low (force only)
bool confirmed                     # both kinds of evidence, or one strong kind; otherwise a candidate
bool geometry                      # evidence: deviation of the contact points from the ball-bottom reference
bool force                         # evidence: force residual against the known shape
geometry_msgs/Point low            # box corners, padded by the probe ball radius
geometry_msgs/Point high
geometry_msgs/Point centre
geometry_msgs/Vector3 axis         # probe axis the region was measured along
float64 area_m2                    # geometry evidence only (0 without it), as the three below
float64 length_m
float64 width_m
float64 max_deviation_m
float64 force_residual_n           # signed, the largest magnitude; 0 without force evidence
uint32 force_samples
