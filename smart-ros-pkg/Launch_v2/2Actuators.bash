#!/bin/bash
ssh -t linuxcnc@arm 'source /opt/ros/fuerte/setup.bash; export ROBOT=golfcart;export ROS_NAMESPACE=golfcart;. ~/linuxcnc/scripts/rip-environment; roslaunch lowlevel_hal lowlevel_hal_ros_adapter.launch; $SHELL'
