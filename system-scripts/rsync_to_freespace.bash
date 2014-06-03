sudo mount -t nfs 192.168.0.2:/mnt/Storage/fmautonomy fmautonomy
rsync -azP /media/rosbag/ fmautonomy/rosbag/Rudolph
