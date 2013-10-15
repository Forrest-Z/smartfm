#!/usr/bin/python
import hal, time
import rospy
import sys
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32, Bool, UInt32, Int32
import functools

def callback(topic, data):
  global h
  h[topic] = data.data

if __name__ == '__main__':
  try:
    global h
    h = hal.component("rossubscriber")
    rospy.init_node('hal_subscriber')
    float_topics = rospy.get_param('~float_topics')
    bool_topics = rospy.get_param('~bool_topics')
    uint_topics = rospy.get_param('~uint_topics')
    int_topics = rospy.get_param('~int_topics')
    
    for f in float_topics:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_OUT)
      rospy.Subscriber(f, Float32, functools.partial(callback, f))
    for b in bool_topics:
      h.newpin(b, hal.HAL_BIT, hal.HAL_OUT)
      rospy.Subscriber(b, Bool, functools.partial(callback, b))
    for u in uint_topics:
      h.newpin(u, hal.HAL_U32, hal.HAL_OUT)
      rospy.Subscriber(u, UInt32, functools.partial(callback, u))
    for i in int_topics:
      h.newpin(i, hal.HAL_S32, hal.HAL_OUT)
      rospy.Subscriber(i, Int32, functools.partial(callback, i))
    h.ready()
    rospy.spin()
  except KeyboardInterrupt:
      raise SystemExit
    
    
