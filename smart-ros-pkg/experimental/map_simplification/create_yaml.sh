#!/bin/bash

rootfolder=`rospack find map_simplification`
echo $rootfolder
#crate map file
map_yaml="image: 1.png\nresolution: 0.100000\norigin: [0.000000, 0.00000, 0.000000]\nnegate: 0\noccupied_thresh: 0.30\nfree_thresh: 0.10"
echo -e $map_yaml > $rootfolder/maps/comb/curb_map.yaml
