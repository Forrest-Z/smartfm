#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' 
ROS node in charge of interfacing with the arduino.
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from std_msgs.msg import Bool, Float64
from lowlevel.msg import Arduino


rospy.init_node('arduino_node')
arduino_cmd_msg = Arduino()
arduino_pub = rospy.Publisher('arduino_cmd', Arduino)

def throttleCB(msg):
    arduino_cmd_msg.throttle_volt = msg.data

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


throttle_sub = rospy.Subscriber('throttle_volt', Float64, throttleCB)
brake_sub = rospy.Subscriber('brake_angle', Float64, brakeCB)
steer_sub = rospy.Subscriber('steer_angle', Float64, steerCB)
lblinker_sub = rospy.Subscriber('left_blinker', Float64, lblinkerCB)
rblinker_sub = rospy.Subscriber('right_blinker', Float64, rblinkerCB)

rospy.Timer(rospy.Duration(0.25), cmd_msg_CB)

rospy.spin()
