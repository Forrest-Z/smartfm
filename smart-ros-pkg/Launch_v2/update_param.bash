#!/bin/bash
package_path=`rospack find Launch_v2`
robot_name=`echo $ROBOT`
rosrun Launch_v2 rename_frame.py $package_path/parameters.yaml $package_path/parameters_temp.yaml env_ROBOT
rosparam load $package_path/parameters_temp.yaml $robot_name
roslaunch Launch_v2 $1
