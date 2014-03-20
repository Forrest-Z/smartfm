#!/bin/bash 
find . -name '*.bag' | while IFS=$'\n' read -r FILE; do
	filename=$(basename "$FILE")
	dir="${FILE:0:${#FILE} - ${#filename}}"
	extension="${filename##*.}"
	filename="${filename%.*}"
	#echo $FILE
	rosrun "autonomous_odometer" "iMiev_autonomous_odo" $FILE
done

