#!/usr/bin/python
import hal, time
import rospy
import roslib; roslib.load_manifest('lowlevel_hal')
from std_msgs.msg import Float32, Bool, UInt32, Int32
import functools
from datetime import datetime

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
    
State = enum('Manual', 'Auto', 'Emergency')

def callback(topic, data):
  global current_state
  global throttle_topic, brake_topic
  global response_enabled
  global last_throttle_time
  
  if(response_enabled and topic == throttle_topic):
    last_throttle_time = datetime.now()
    if(current_state == State.Emergency or current_state == State.Manual): h[topic] = 0
    else: h[topic] = data.data
  elif(response_enabled and topic == brake_topic):
    if(current_state == State.Emergency or current_state == State.Manual): h[topic] = 0
    else: h[topic] = data.data
  else:
    h[topic] = data.data

def publishData(event):
  global pre_state, current_state
  global emergency_topic, auto_topic
  global response_enabled
  global throttle_topic, brake_topic
  counter = 0
  bool_count = 0
  for f in float_topics_pub:
    pubs[counter].publish(Float32(h[f]))
    counter+=1
  for b in bool_topics_pub:
    current_value = h[b];
    if(current_value != pre_bool_value[bool_count]):
      pre_bool_value[bool_count] = current_value
      pubs[counter].publish(Bool(current_value))
    counter+=1
    bool_count+=1
  for u in uint_topics_pub:
    pubs[counter].publish(UInt32(h[u]))
    counter+=1
  for i in int_topics_pub:
    pubs[counter].publish(Int32(h[i]))
    counter+=1
  if(response_enabled):
    msg_time = (datetime.now() - last_throttle_time).total_seconds()
    if(h[emergency_topic]):
      current_state = State.Emergency
    if(h[auto_topic] == False):
      current_state = State.Manual
    if(h[auto_topic] == True and current_state == State.Manual):
      current_state = State.Auto
    if( msg_time > 0.1): 
      current_state = State.Emergency
      h[throttle_topic] = 0
      h[brake_topic] = 0
  if(pre_state != current_state):
    state_pub.publish(Int32(current_state))
    pre_state = current_state

if __name__ == '__main__':
  global pre_state, current_state
  global emergency_topic, auto_topic, throttle_topic, brake_topic
  global response_enabled
  global last_throttle_time
  global msg_time
  
  last_throttle_time = datetime.now()
  emergency_count =0
  throttle_count = 0
  brake_count = 0
  auto_count = 0
  msg_time = 0
  response_enabled = False
  current_state = State.Manual
  pre_state = -1
  try:
    rospy.init_node('hal_ros_adapater')
    h = hal.component("rosadapter")
    float_topics_pub = rospy.get_param('~float_topics_pub')
    bool_topics_pub = rospy.get_param('~bool_topics_pub')
    uint_topics_pub = rospy.get_param('~uint_topics_pub')
    int_topics_pub = rospy.get_param('~int_topics_pub')
    pubs = []
    pre_bool_value = []
    float_topics_sub = rospy.get_param('~float_topics_sub')
    bool_topics_sub = rospy.get_param('~bool_topics_sub')
    uint_topics_sub = rospy.get_param('~uint_topics_sub')
    int_topics_sub = rospy.get_param('~int_topics_sub')
    
    
    for f in float_topics_pub:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_IN)
      pubs.append(rospy.Publisher(f, Float32))
    for b in bool_topics_pub:
      h.newpin(b, hal.HAL_BIT, hal.HAL_IN)
      pubs.append(rospy.Publisher(b, Bool, latch=True))
      pre_bool_value.append(-1)
      if "emergency" in b:
	emergency_topic = b
	emergency_count += 1
      if "auto" in b:
	auto_topic = b
	auto_count += 1
    for u in uint_topics_pub:
      h.newpin(u, hal.HAL_U32, hal.HAL_IN)
      pubs.append(rospy.Publisher(u, UInt32))
    for i in int_topics_pub:
      h.newpin(i, hal.HAL_S32, hal.HAL_IN)
      pubs.append(rospy.Publisher(i, Int32))
    state_pub = rospy.Publisher("bb_state", Int32, latch=True)
    for f in float_topics_sub:
      h.newpin(f, hal.HAL_FLOAT, hal.HAL_OUT)
      rospy.Subscriber(f, Float32, functools.partial(callback, f))
      if "throttle" in f: 
	throttle_topic = f
	throttle_count += 1
      if "brake" in f:
	brake_topic = f
	brake_count += 1
    for b in bool_topics_sub:
      h.newpin(b, hal.HAL_BIT, hal.HAL_OUT)
      rospy.Subscriber(b, Bool, functools.partial(callback, b))
    for u in uint_topics_sub:
      h.newpin(u, hal.HAL_U32, hal.HAL_OUT)
      rospy.Subscriber(u, UInt32, functools.partial(callback, u))
    for i in int_topics_sub:
      h.newpin(i, hal.HAL_S32, hal.HAL_OUT)
      rospy.Subscriber(i, Int32, functools.partial(callback, i))
    
    if(throttle_count == brake_count == emergency_count == auto_count == 1):
      response_enabled = True
      print("Response enabled")
    else: 
      print("Response disabled")
      print("Found topics:")
      print("throttle: ",throttle_count," brake: ", brake_count)
      print("emergency: ", emergency_count, " auto: ", auto_count)
    h.ready()

    rospy.Timer(rospy.Duration(1.0/50), publishData)
    rospy.spin()
   

  except KeyboardInterrupt:
    raise SystemExit
    
    
