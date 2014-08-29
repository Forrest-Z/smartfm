#!/bin/bash
set -e

SIMTYPE=$1
SIMPARAM=sim_${SIMTYPE}.yaml

cd ~/smartfm/smart-ros-pkg/utown_plaza/run_simulation
[ -e $SIMPARAM ] || ( echo $SIMPARAM does not exist! && exit -1 )

# setup env
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=~/smartfm/smart-ros-pkg:$ROS_PACKAGE_PATH
source py/bin/activate
source aws_access.sh

while true; do
    rm -f *.bag
    roslaunch all.launch simparam:=${SIMPARAM} > pomdp.log
    python utils.py count
    s3put -b golfcar.test -p $(pwd) --region ap-southeast-1 -c 100 -k $SIMTYPE *.bag
    rm -f *.bag
    sleep 1
done
