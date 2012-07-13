#!/bin/bash

thread_n=12
total_it=8191
rootfolder=`rospack find map_simplification`
echo "root folder found at:"
echo $map_simplification_folder

#sleep 5
for ((i = 0; i <= $total_it; i+=$thread_n ))
do
    roscore &
    sleep 2
    for ((j = i; j < i+$thread_n; j++ ))
    do
      if [ $j -le $total_it ]
        then
            echo start $j map
            #crate map file
            map_yaml="image: "
            map_yaml=$map_yaml$j".png\nresolution: 0.100000\norigin: [0.000000, 0.00000, 0.000000]\nnegate: 0\noccupied_thresh: 0.30\nfree_thresh: 0.10"
            echo -e $map_yaml > $rootfolder/maps/comb/$j.yaml
            cmd="./single_sim.sh $j"
            echo $cmd
            `gnome-terminal -x $cmd`
      fi
    done
    ./play_bag.sh
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


