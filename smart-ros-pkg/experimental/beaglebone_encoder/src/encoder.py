#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('beaglebone_encoder')
import Adafruit_BBIO.GPIO as GPIO
from phidget_encoders.msg import Encoders

counter = 0

def encoderCallback(channel):
	global counter
	counter = counter + 1

def Encoder():
	GPIO.setup("P9_12", GPIO.IN)
	GPIO.add_event_detect("P9_12", GPIO.BOTH, callback=encoderCallback)
	pub = rospy.Publisher('beaglebone_encoder', Encoders)
	rospy.init_node('beaglebone_encoder')
	r = rospy.Rate(50)
	
	encodersMsg = Encoders()
	lastPub = rospy.get_time()+10
	wheel_size = 1.32212398060626
	pps = 80
	distance_per_pulse = wheel_size / pps
	last_count = 0
	global counter
	while not rospy.is_shutdown():
		current_count = counter
		dcount = current_count - last_count
		last_count = current_count
		encodersMsg.stamp = rospy.Time.now()
		stamp_sec = encodersMsg.stamp.to_sec()
		encodersMsg.dt = stamp_sec - lastPub
		lastPub = stamp_sec	
		encodersMsg.d_dist = dcount * distance_per_pulse
		encodersMsg.d_left = dcount
		encodersMsg.d_count_left = current_count
		encodersMsg.v = encodersMsg.d_dist/encodersMsg.dt
		pub.publish(encodersMsg)
		
		r.sleep()
		
		
if __name__ == '__main__':
	try:
		Encoder()
	except rospy.ROSInterruptException:
		pass
