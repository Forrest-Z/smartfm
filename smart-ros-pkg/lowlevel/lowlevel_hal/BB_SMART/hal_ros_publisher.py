#!/usr/bin/python
import hal, time
import rospy
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32, Bool, UInt32, Int32

if __name__ == '__main__':
  try:
    rospy.init_node('hal_publisher')
    h = hal.component("rospublisher")
    float_topics = rospy.get_param('~float_topics')
    bool_topics = rospy.get_param('~bool_topics')
    uint_topics = rospy.get_param('~uint_topics')
    int_topics = rospy.get_param('~int_topics')
    pubs = []
    
    for f in float_topics:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_IN)
      pubs.append(rospy.Publisher(f, Float32))
    for b in bool_topics:
      h.newpin(b, hal.HAL_BIT, hal.HAL_IN)
      pubs.append(rospy.Publisher(b, Bool))
    for u in uint_topics:
      h.newpin(u, hal.HAL_U32, hal.HAL_IN)
      pubs.append(rospy.Publisher(u, UInt32))
    for i in int_topics:
      h.newpin(i, hal.HAL_S32, hal.HAL_IN)
      pubs.append(rospy.Publisher(i, Int32))
    h.ready()

    r = rospy.Rate(100)
    
    while not rospy.is_shutdown():
	counter = 0
	for f in float_topics:
	  pubs[counter].publish(Float32(h[f]))
	  counter+=1
	for b in bool_topics:
	  pubs[counter].publish(Bool(h[b]))
	  counter+=1
	for u in uint_topics:
	  pubs[counter].publish(UInt32(h[u]))
	  counter+=1
	for i in int_topics:
	  pubs[counter].publish(Int32(h[i]))
	  counter+=1
	r.sleep()
  except KeyboardInterrupt:
    raise SystemExit
    
    
