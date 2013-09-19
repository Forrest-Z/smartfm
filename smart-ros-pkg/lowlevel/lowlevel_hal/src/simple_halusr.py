#!/usr/bin/python
import hal, time
import rospy
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32

pub = rospy.Publisher('hal_publisher', Float32)
h = hal.component("ros_publisher")

if __name__ == '__main__':
  rospy.init_node('lowlevel_hal')
  h.newpin("in", hal.HAL_FLOAT, hal.HAL_IN)
  #h.newpin("out", hal.HAL_FLOAT, hal.HAL_OUT)
  h.ready()
  try:
      r = rospy.Rate(50)
      
      while not rospy.is_shutdown():
	pub.publish(Float32(h['in']))
	r.sleep()
  except KeyboardInterrupt:
      raise SystemExit
    
    
