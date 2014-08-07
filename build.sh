#!/bin/bash

source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=~/smartfm/smart-ros-pkg:$ROS_PACKAGE_PATH

rosmake ped_momdp_sarsop ped_pathplan golfcar_ppc
