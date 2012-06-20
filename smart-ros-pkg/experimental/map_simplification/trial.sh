#!/bin/bash
for j in {1..5}
do
    echo "Start roscore $j"
    roscore &
    for i in {5..1}
    do
        echo "Sleep for $i before killing"
        sleep 1
    done
    echo "Kill now!"
    pkill roscore
    roscore_alive=`ps -a | grep roscore`
    echo $rosbag_alive
    if [ "$rosbag_alive" = "" ]
        then 
            echo "Hurray! roscore killed"
    else 
        echo "Nah, roscore not killed"
    fi    
    sleep 1
done
