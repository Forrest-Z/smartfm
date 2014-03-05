gst-launch-1.0 -v udpsrc port=1234 ! "application/x-rtp, payload=127" ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! xvimagesink sync=false

