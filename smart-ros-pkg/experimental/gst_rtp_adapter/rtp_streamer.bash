gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,format="YUY2",width=640,height=360' ! videoconvert ! x264enc bitrate=200 tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=1234

