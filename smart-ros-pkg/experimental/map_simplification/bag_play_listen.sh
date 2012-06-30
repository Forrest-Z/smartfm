#!/bin/bash
sleep 5
rosbag_alive=`rosnode list | grep bag_play`
echo $rosbag_alive
while [ "$rosbag_alive" = "/bag_play" ]
do
    sleep 1
    rosbag_alive=`rosnode list | grep bag_play`
done
echo "Done playback rosbag!"
rosnode kill -a   
