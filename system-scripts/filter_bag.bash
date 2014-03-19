#!/bin/bash 
filter_expr="( (topic!='/iMiev/single_pcl') and (topic!='/iMiev/pc_legacy_out') )"
filter_name="_filter.bag"
find . -name '*.bag' | while IFS=$'\n' read -r FILE; do
	filename=$(basename "$FILE")
	dir="${FILE:0:${#FILE} - ${#filename}}"
	extension="${filename##*.}"
	filename="${filename%.*}"
	filter_filename=$dir$filename$filter_name
	rosbag "filter" "$FILE" "$filter_filename" "$filter_expr"
	mv "$filter_filename" "$FILE"
done

