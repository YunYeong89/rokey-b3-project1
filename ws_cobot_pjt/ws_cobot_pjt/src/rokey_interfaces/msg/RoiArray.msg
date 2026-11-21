# The regions of one scan: confirmed first, then candidates (pointcloud_damage --roi takes the confirmed ones).
std_msgs/Header header             # frame_id base_link
string run_id                      # scan run: <part>_<time>
string part
float64 force_limit_n              # force residual limit of this scan
RoiRegion[] regions
