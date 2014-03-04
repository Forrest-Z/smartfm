gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,format="YUY2",width=640,height=360' ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink port=1234

