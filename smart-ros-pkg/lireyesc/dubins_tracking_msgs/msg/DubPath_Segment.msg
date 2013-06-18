# Zero, if segment is straight line
# Negative, if segment is rightward circular arc
# Positive, if segment is leftward circular arc

float32 signed_curvature

# Center of circular arc, if it exists

geometry_msgs/Point32 center

# Starting and ending states of segment (x, y, theta)

geometry_msgs/Point32 state_starting

geometry_msgs/Point32 state_ending

# Length of segment

float32 length

