#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the arduino. Messages from the different
arduino topics are cached and sent to the arduino at a fixed rate. This rate can
be specified by parameter 'rate' (defaults to 10Hz).
'''

import roslib; roslib.load_manifest('lowlevel_arduino')
import rospy
from std_msgs.msg import Bool, Float64
from lowlevel_arduino.msg import Arduino


rospy.init_node('arduino_node')

brake_change_threshold = rospy.get_param('~brake_change_threshold', 0)

arduino_cmd_msg = Arduino()
arduino_pub = rospy.Publisher('arduino_cmd', Arduino)

def throttleCB(msg):
    arduino_cmd_msg.throttle = msg.data

def brakeCB(msg):
    # in speed controller, braking is done in the negative direction,
    # but in arduino it is in the positive direction.
    a = -msg.data

    # Reduce a bit the amount of changes on the brake: fine positioning is not
    # required, and the brake makes strange noise when there are many small
    # steps
    # TODO: verify that this works fine.
    if abs(arduino_cmd_msg.brake_angle-a) > brake_change_threshold:
        arduino_cmd_msg.brake_angle = a

def steerCB(msg):
    arduino_cmd_msg.steer_angle = msg.data


def setupSub(topic,msg_t,cb):
    sub = rospy.Subscriber(topic, msg_t, cb)
    rospy.loginfo('Setup Subscriber on %s [%s]' % (topic, msg_t._type))
    return sub


throttle_sub = setupSub('throttle', Float64, throttleCB)
brake_sub = setupSub('brake_angle', Float64, brakeCB)
steer_sub = setupSub('steer_angle', Float64, steerCB)

rate = rospy.get_param('~rate',10)
rospy.loginfo('Sending messages to the Arduino board every %dms' % int(1000.0/rate))

while not rospy.is_shutdown():
    arduino_pub.publish(arduino_cmd_msg)
    rospy.sleep(1.0/rate)