# Self-sufficient localmap packet
Header header

# map resolution, width, height in meters
float32 res
float32 width
float32 height
# xsize, ysize = width/res
int32 xsize
int32 ysize
# xorigin in terms of xsize/2 etc.
int32 xorigin
int32 yorigin
# position of the car in odom frame for this map
geometry_msgs/Point origin
# actual data
float32[] vals

