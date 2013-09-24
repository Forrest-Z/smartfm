#!/usr/bin/python
import hal, time
import rospy
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32, Bool, UInt32, Int32
import functools

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
    
State = enum('Manual', 'Auto', 'Emergency')

def callback(topic, data):
  global current_state
  if(topic == 'throttle'):
    if(current_state == State.Emergency or current_state == State.Manual): h[topic] = 0
    else: h[topic] = data.data
  elif(topic == 'brake_angle'):
    if(current_state == State.Emergency or current_state == State.Manual): h[topic] = 0
    else: h[topic] = data.data
  else:
    h[topic] = data.data

def publishData(event):
  global current_state
  counter = 0
  for f in float_topics_pub:
    pubs[counter].publish(Float32(h[f]))
    counter+=1
  for b in bool_topics_pub:
    pubs[counter].publish(Bool(h[b]))
    counter+=1
  for u in uint_topics_pub:
    pubs[counter].publish(UInt32(h[u]))
    counter+=1
  for i in int_topics_pub:
    pubs[counter].publish(Int32(h[i]))
    counter+=1
  if(h["button_state_emergency"]):
    current_state = State.Emergency
  if(h["button_state_automode"] == False):
    current_state = State.Manual
  if(h["button_state_automode"] == True and current_state == State.Manual):
    current_state = State.Auto
  

if __name__ == '__main__':
  global current_state
  try:
    rospy.init_node('hal_ros_adapater')
    h = hal.component("rosadapter")
    float_topics_pub = rospy.get_param('~float_topics_pub')
    bool_topics_pub = rospy.get_param('~bool_topics_pub')
    uint_topics_pub = rospy.get_param('~uint_topics_pub')
    int_topics_pub = rospy.get_param('~int_topics_pub')
    pubs = []
    
    float_topics_sub = rospy.get_param('~float_topics_sub')
    bool_topics_sub = rospy.get_param('~bool_topics_sub')
    uint_topics_sub = rospy.get_param('~uint_topics_sub')
    int_topics_sub = rospy.get_param('~int_topics_sub')
    
    current_state = State.Manual
    
    for f in float_topics_pub:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_IN)
      pubs.append(rospy.Publisher(f, Float32))
    for b in bool_topics_pub:
      h.newpin(b, hal.HAL_BIT, hal.HAL_IN)
      pubs.append(rospy.Publisher(b, Bool))
    for u in uint_topics_pub:
      h.newpin(u, hal.HAL_U32, hal.HAL_IN)
      pubs.append(rospy.Publisher(u, UInt32))
    for i in int_topics_pub:
      h.newpin(i, hal.HAL_S32, hal.HAL_IN)
      pubs.append(rospy.Publisher(i, Int32))
   
    for f in float_topics_sub:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_OUT)
      rospy.Subscriber(f, Float32, functools.partial(callback, f))
    for b in bool_topics_sub:
      h.newpin(b, hal.HAL_BIT, hal.HAL_OUT)
      rospy.Subscriber(b, Bool, functools.partial(callback, b))
    for u in uint_topics_sub:
      h.newpin(u, hal.HAL_U32, hal.HAL_OUT)
      rospy.Subscriber(u, UInt32, functools.partial(callback, u))
    for i in int_topics_sub:
      h.newpin(i, hal.HAL_S32, hal.HAL_OUT)
      rospy.Subscriber(i, Int32, functools.partial(callback, i))
      
    h.ready()

    rospy.Timer(rospy.Duration(1.0/100), publishData)
    rospy.spin()
   

  except KeyboardInterrupt:
    raise SystemExit
    
    
