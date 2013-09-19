#!/usr/bin/python
import hal, time
import rospy
import sys
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32

def callback(data):
  global h
  h['in'] = data.data

if __name__ == '__main__':
  try:
    rospy.init_node('lowlevel_hal')
    float_topics = rospy.get_param('~float_topics')
    bin_topics = rospy.get_param('~bin_topics')
    uint_topics = rospy.get_param('~uint_topics')
    int_topics = rospy.get_param('~int_topics')
    global h
    h = hal.component("rospublisher")
    for f in float_topics:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_IN)
      rospy.Subscriber(f, Float32, callback)
    h.ready()
    rospy.spin()
  except KeyboardInterrupt:
      raise SystemExit
    
    
