#It is recommeded to use Point (which float64) wherever possible instead of Point32.
#
#header contain header information from current image frame (of time t = n)
#array prev_feature contain position of features we want to track from frame at time t = n-1
#array found_feature contain position of features from previous frame (t = n-1) found in current frame (t = 1)

Header header
geometry_msgs/Point[] prev_feature    
geometry_msgs/Point[] found_feature
geometry_msgs/Point[] feature_vel
int16[]               direction
#feature_speed_mag (feature_mag) is magnitude of velocity of the feature
#feature_speed_dir (direction_mag) is direction of velocity in radian, calculate using atan2 from math library.
float64[]             feature_speed_mag
float64[]             feature_speed_dir
#for DB clustering
bool[]                    visited
bool[]                    isnoise
