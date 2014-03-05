gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,format="YUY2",width=640,height=360' ! videoconvert ! x264enc bitrate=600 tune=zerolatency intra-refresh=true bframes=0 ref=1 key-int-max=30  ! rtph264pay ! udpsink host=119.234.131.195  port=1234

