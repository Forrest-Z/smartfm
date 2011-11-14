#!/bin/sh

if [ $# -ne 1 ]; then
    echo "You must give the name of the bag file to process." >&2
    exit 1
fi

infile=$1
if [ ! -e $infile ]; then
    echo "Error: bag file $infile does not exist." >&2
    exit 1
fi

base=`basename $infile .bag`
sensor_bag="$base"_sensors.bag
laser_bag="$base"_lasers.bag
video_bag="$base"_video.bag

echo "Splitting:"

rosbag filter $infile $sensor_bag "topic in ['/encoders','/fix','/ms/imu/data','/ms/imu/is_calibrated']" || exit 1
rosbag filter $infile $laser_bag "topic in ['/scan','/sick_scan','/sick_scan2','/tf']" || exit 1
rosbag filter $infile $video_bag "topic in ['/usb_cam/image_raw/compressed','/tf']" || exit 1

echo "Compressing:"
rosbag compress $sensor_bag $laser_bag || exit 1
# No need to compress the video bag, as it contain mostly video data, which
# is already compessed. Compressing takes time but does not save much space.

exit 0