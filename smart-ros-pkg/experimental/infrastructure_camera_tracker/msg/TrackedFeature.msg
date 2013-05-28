# header not needed as the tracked feature message shall not be used by itself
geometry_msgs/Point   position  # the position of the feature
float64               time      # the time at which the feture was detected
uint64                id        # the feature's unique id
bool                  active    # is the feature (still) being tracked?