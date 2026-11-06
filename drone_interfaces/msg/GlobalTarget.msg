# Geographic position target for /<name>/cmd/global (GUIDED mode).
std_msgs/Header header
float64 latitude                  # deg (WGS84)
float64 longitude                 # deg (WGS84)
float32 altitude                  # m ABOVE HOME (relative altitude, like QGC "fly here")
float32 yaw                       # rad, ENU (0 = East, counter-clockwise +). NaN = keep the current heading
