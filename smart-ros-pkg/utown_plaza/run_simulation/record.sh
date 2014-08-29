#!/bin/sh
#rosbag record -a -j
rosbag record -a -x /golfcart/ped_data_assoc -j -o $(uname -n)_
