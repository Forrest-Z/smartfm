# Self-sufficient localmap packet
Header header

# map resolution, width, height in meters
float32 res
float32 width
float32 height
# xsize, ysize = width/res
uint32 xsize
uint32 ysize
# xorigin in terms of xsize/2 etc.
uint32 xorigin
uint32 yorigin
# actual data
uint8[] vals
