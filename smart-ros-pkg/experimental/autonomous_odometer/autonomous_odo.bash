#!/bin/bash
rosparam set use_sim_time true 
ros_pkg=`rospack find autonomous_odometer`
# bag_files=0
# find . -name '*.bag' | while IFS=$'\n' read -r FILE; do
#   echo $FILE
#   (( bag_files++ ))
#   echo $bag_files
# done
FILELIST=(`find . -type f -name '*.bag'`);
TotalFile=${#FILELIST[@]}
for (( i=0; i<$TotalFile; i++ ))
do
  FILE=${FILELIST[$i]}
  echo Processing$i/$TotalFile $FILE
  filename=$(basename "$FILE")
  dir="${FILE:0:${#FILE} - ${#filename}}"
  extension="${filename##*.}"
  filename="${filename%.*}"
  $ros_pkg"/bin/iMiev_autonomous_odo" $FILE
  echo "Done playback"
done

