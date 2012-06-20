#!/bin/bash


rootfolder=`rospack find map_simplification`
echo $rootfolder

for i in {0..170}
do
    echo start $i map
    #crate map file
    map_yaml="image: "
    map_yaml=$map_yaml$i".png\nresolution: 0.100000\norigin: [0.000000, 0.00000, 0.000000]\nnegate: 0\noccupied_thresh: 0.30\nfree_thresh: 0.10"
    echo -e $map_yaml > $rootfolder/maps/comb/curb_map.yaml
    
    #sumfile=`$rootfolder/log/summary`
    `gnome-terminal -x ./bag_play_listen.sh`
    `rosparam delete /amcl_node`
    `roslaunch map_simplification localization_sim.launch > $rootfolder/log/detailed_sim.$i`
    sum_error=`cat $rootfolder/log/detailed_sim.$i | grep "_error ="`
    sum_error=${i}${sum_error}
    echo $sum_error >> $rootfolder/log/summary.csv
done

