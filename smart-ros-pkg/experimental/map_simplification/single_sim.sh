rootfolder=`rospack find map_simplification`
echo $rootfolder
echo start $1 map

#`rosparam delete MAP_$1/amcl_node`
`roslaunch map_simplification localization_sim.launch map_no:=$1 > $rootfolder/log/detailed_sim.$1`
sum_error=`cat $rootfolder/log/detailed_sim.$1 | grep "_error ="`
sum_error=${1}${sum_error}
echo $sum_error >> $rootfolder/log/summary.csv
rm $rootfolder/maps/comb/$1.yaml
