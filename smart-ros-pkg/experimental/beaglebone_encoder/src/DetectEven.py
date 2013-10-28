import Adafruit_BBIO.GPIO as GPIO 
import time 

GPIO.setup("P9_12", GPIO.IN) 
GPIO.add_event_detect("P9_12", GPIO.BOTH) 

start_time = time.time() 
count = 0 
while True:
	current_time = time.time()
	delta_time = current_time - start_time
#	print delta_time
	if GPIO.event_detected("P9_12"):
		count = count + 1
	
	if delta_time >= 0.1:
		start_time = current_time
		print "Count: ", count, "Current time: ", current_time
		count = 0
