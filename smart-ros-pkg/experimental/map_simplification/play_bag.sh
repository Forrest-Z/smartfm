rootfolder=`rospack find map_simplification`
echo $rootfolder
echo "Wait for 3 sec to get everything started"
sleep 3
rosbag play $rootfolder/2_filtered_clock.bag -s 20 -r 5
echo "Done playback rosbag!"
rosnode kill -a
