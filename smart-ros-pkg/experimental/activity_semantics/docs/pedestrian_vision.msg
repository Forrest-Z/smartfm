# Which cluster are we considering?
feature_detection/cluster cluster
geometry_msgs/Point32 local_centroid

# data association
int32 object_label
bool complete_flag
float32 confidence
float64[] image_hash

# result from the vision
bool decision_flag

# distance from the camera to the pedestrian
float32 disz

# coordinates of the rectangle: (x,y) is the upper left corner
int32 x
int32 y
int32 height
int32 width

# coordinates of the rectangle: 1 - upper left corner, 2 - lower left corner
float32 cvRect_x1
float32 cvRect_y1
float32 cvRect_x2
float32 cvRect_y2
