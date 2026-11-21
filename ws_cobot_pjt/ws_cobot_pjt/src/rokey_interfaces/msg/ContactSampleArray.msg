# Contact samples in time order, published in batches (~10 Hz) so that a 100-200 Hz sampler does not need one
# message per sample.
std_msgs/Header header             # frame_id base_link
string run_id                      # scan run: <part>_<time>, as in the output file names
ContactSample[] samples
