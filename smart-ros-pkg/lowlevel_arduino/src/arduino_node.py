#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the arduino. Messages from the different
arduino topics are cached and sent to the arduino at a fixed rate. This rate can
be specified by parameter 'rate' (defaults to 10Hz).
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from std_msgs.msg import Bool, Float64
from lowlevel_arduino.msg import Arduino


rospy.init_node('arduino_node')
arduino_cmd_msg = Arduino()
arduino_pub = rospy.Publisher('arduino/arduino_cmd', Arduino)

def throttleCB(msg):
    arduino_cmd_msg.throttle = msg.data

def brakeCB(msg):
    arduino_cmd_msg.brake_angle = msg.data

def steerCB(msg):
    arduino_cmd_msg.steer_angle = msg.data

def lblinkerCB(msg):
    arduino_cmd_msg.left_blinker = msg.data

def rblinkerCB(msg):
    arduino_cmd_msg.right_blinker = msg.data

def cmd_msg_CB(event):
    arduino_pub.publish( arduino_cmd_msg )


def setupSub(topic,msg_t,cb):
    sub = rospy.Subscriber(topic, msg_t, cb)
    rospy.loginfo('Setup Subscriber on %s [%s]' % (topic, msg_t._type))
    return sub


throttle_sub = setupSub('arduino/throttle', Float64, throttleCB)
brake_sub = setupSub('arduino/brake_angle', Float64, brakeCB)
steer_sub = setupSub('arduino/steer_angle', Float64, steerCB)
lblinker_sub = setupSub('arduino/left_blinker', Bool, lblinkerCB)
rblinker_sub = setupSub('arduino/right_blinker', Bool, rblinkerCB)

rate = rospy.get_param('~rate',10)
rospy.loginfo('Sending messages to the Arduino board every %dms' % int(1000.0/rate))

while not rospy.is_shutdown():
    arduino_pub.publish(arduino_cmd_msg)
    rospy.sleep(1.0/rate)
